import random
import re
import unittest
import numpy as np

from opendbc.car import Bus, DT_CTRL
from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.carcontroller import HCAMitigation
from opendbc.car.volkswagen.values import CAR, CarControllerParams as CCP, FW_QUERY_CONFIG, WMI
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVolkswagenHCAMitigation(unittest.TestCase):
  STUCK_TORQUE_FRAMES = round(CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP))

  def test_same_torque_mitigation(self):
    """Same-torque nudge fires at the threshold, in the correct direction, and resets cleanly."""
    hca_mitigation = HCAMitigation(CCP)

    for actuator_value in (-CCP.STEER_MAX, -1, 0, 1, CCP.STEER_MAX):
      hca_mitigation.update(0, 0)  # Reset mitigation state
      for frame in range(self.STUCK_TORQUE_FRAMES + 2):
        should_nudge = actuator_value != 0 and frame == self.STUCK_TORQUE_FRAMES
        expected_torque = actuator_value - (1, -1)[actuator_value < 0] if should_nudge else actuator_value
        assert hca_mitigation.update(actuator_value, actuator_value) == expected_torque, f"{frame=}"

class TestVolkswagenPlatformConfigs(unittest.TestCase):
  def test_spare_part_fw_pattern(self):
    # Relied on for determining if a FW is likely VW
    for platform, ecus in FW_VERSIONS.items():
      with self.subTest(platform=platform.value):
        for fws in ecus.values():
          for fw in fws:
            assert SPARE_PART_FW_PATTERN.match(fw) is not None, f"Bad FW: {fw}"

  def test_chassis_codes(self):
    for platform in CAR:
      with self.subTest(platform=platform.value):
        assert len(platform.config.wmis) > 0, "WMIs not set"
        assert len(platform.config.chassis_codes) > 0, "Chassis codes not set"
        assert all(CHASSIS_CODE_PATTERN.match(cc) for cc in
                   platform.config.chassis_codes), "Bad chassis codes"

        # No two platforms should share chassis codes
        for comp in CAR:
          if platform == comp:
            continue
          assert set() == platform.config.chassis_codes & comp.config.chassis_codes, \
                           f"Shared chassis codes: {comp}"

  def test_custom_fuzzy_fingerprinting(self):
    all_radar_fw = list({fw for ecus in FW_VERSIONS.values() for fw in ecus[Ecu.fwdRadar, 0x757, None]})

    for platform in CAR:
      with self.subTest(platform=platform.name):
        for wmi in WMI:
          for chassis_code in platform.config.chassis_codes | {"00"}:
            vin = ["0"] * 17
            vin[0:3] = wmi
            vin[6:8] = chassis_code
            vin = "".join(vin)

            # Check a few FW cases - expected, unexpected
            for radar_fw in random.sample(all_radar_fw, 5) + [b'\xf1\x875Q0907572G \xf1\x890571', b'\xf1\x877H9907572AA\xf1\x890396']:
              should_match = ((wmi in platform.config.wmis and chassis_code in platform.config.chassis_codes) and
                              radar_fw in all_radar_fw)

              live_fws = {(0x757, None): [radar_fw]}
              matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fws, vin, FW_VERSIONS)

              expected_matches = {platform} if should_match else set()
              assert expected_matches == matches, "Bad match"


class TestVolkswagenMebCurvatureSafety(unittest.TestCase):
  # the EPS settles at its own curvature, which reads ratio times the curvature the car drives
  EPS_TAU = 0.3  # s
  EPS_DELAY = 4  # frames

  def _drive(self, gen2, v_fn, accel_fn, ratio, yaw_noise, cs_lag, seconds=8.):
    from collections import defaultdict
    from types import SimpleNamespace
    from opendbc.can import CANPacker
    from opendbc.car import structs
    from opendbc.car.volkswagen.carcontroller import CarController
    from opendbc.car.volkswagen.interface import CarInterface
    from opendbc.car.volkswagen.values import DBC, VolkswagenSafetyFlags
    from opendbc.safety.tests.libsafety import libsafety_py

    platform = CAR.VOLKSWAGEN_ID4_MK2 if gen2 else CAR.VOLKSWAGEN_ID4_MK1
    CP = CarInterface.get_non_essential_params(platform)
    CP.openpilotLongitudinalControl = False
    cc = CarController(DBC[platform], CP)
    packer = CANPacker(DBC[platform][Bus.pt])
    safety = libsafety_py.libsafety
    safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMeb, VolkswagenSafetyFlags.MEB_ALT_CRC if gen2 else 0)
    safety.init_tests()

    def rx(name, values):
      addr, dat, _ = packer.make_can_msg(name, 0, values)
      assert safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, 0, dat)), name

    rng = random.Random(0)
    eps_curvature, desired, sent, seen = 0., 0., [0.] * (self.EPS_DELAY + 1), []
    max_accel = 0.
    for frame in range(int(seconds / DT_CTRL)):
      t = frame * DT_CTRL
      safety.set_timer(int(t * 1e6))
      v = v_fn(t)
      curvature = eps_curvature / ratio
      yaw_rate = float(curvature * v + np.radians(rng.gauss(0, yaw_noise)))
      if frame % 2 == 0:
        rx("QFK_01", {"Curvature": abs(eps_curvature), "Curvature_VZ": eps_curvature > 0})
        rx("ESC_50", {"Yaw_Rate": abs(np.degrees(yaw_rate)), "Yaw_Rate_Sign": yaw_rate > 0})
        rx("ESC_51", {f"{w}_Radgeschw": v * 3.6 for w in ("HL", "HR", "VL", "VR")})
        rx("ESP_21", {"ESP_v_Signal": v * 3.6})
      seen.append((eps_curvature, yaw_rate, v, curvature))
      cs_eps, cs_yaw, cs_v, cs_curvature = seen[max(len(seen) - 1 - cs_lag, 0)]
      safety.set_controls_allowed(True)

      # the model plan, at the lateral jerk limit
      target = accel_fn(t) / max(v, 1.) ** 2
      step = 5. / max(v, 1.) ** 2 * DT_CTRL
      desired = float(np.clip(target, desired - step, desired + step))
      CC = structs.CarControl(latActive=True, enabled=True)
      CC.actuators.curvature = desired
      CC.currentCurvature = float(cs_curvature)
      CS = SimpleNamespace(out=structs.CarState(vEgo=float(cs_v), vEgoRaw=float(cs_v), yawRate=cs_yaw), curvature_meas=float(cs_eps),
                           eps_stock_values=defaultdict(int), klr_stock_values=defaultdict(int), acc_type=0,
                           gra_stock_values=defaultdict(int), ldw_stock_values=defaultdict(int), gra_stock_counter=0)
      _, sends = cc.update(CC.as_reader(), CS, int(t * 1e9))
      for addr, dat, _ in sends:
        if addr == 0x303:
          assert safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, 0, dat)), f"HCA_03 blocked at {t=:.2f} {v=:.1f} {ratio=}"
          sent.append(cc.apply_curvature_last)
          sent.pop(0)
      eps_curvature += DT_CTRL / self.EPS_TAU * (0.95 * sent[0] - eps_curvature)
      max_accel = max(max_accel, abs(curvature) * v ** 2)
    return max_accel

  def test_never_blocked(self):
    # the carcontroller stays inside the safety limit, also where it allows the EPS curvature above the yaw rate curvature
    for gen2 in (False, True):
      for v in (10., 25., 35.):
        for ratio in (0.8, 1.0, 1.4, 1.8, 2.3):
          for sign in (1, -1):
            for yaw_noise, cs_lag in ((0., 1), (0.1, 3)):
              with self.subTest(gen2=gen2, v=v, ratio=ratio, sign=sign, yaw_noise=yaw_noise, cs_lag=cs_lag):
                self._drive(gen2, lambda t, v=v: v, lambda t, s=sign: 4.5 * s if 1. < t < 6. else 0., ratio, yaw_noise, cs_lag)

    # braking through a curve and S-curves
    self._drive(True, lambda t: max(35. - 5. * max(t - 2., 0.), 8.), lambda t: 4.5 if t > 1. else 0., 1.6, 0.1, 2)
    self._drive(True, lambda t: 30., lambda t: 4.5 * np.sin(2 * np.pi * t / 4.), 1.5, 0.1, 2)

  def test_reaches_lateral_accel(self):
    # with the EPS curvature reading high, the car still reaches the ISO lateral accel the plan asks for
    for ratio in (1.0, 1.3, 1.75):
      with self.subTest(ratio=ratio):
        reached = self._drive(True, lambda t: 33., lambda t: 3. if t > 1. else 0., ratio, 0.05, 1)
        assert reached > 2.85, f"{ratio=} {reached=}"
