import random
import re
import unittest
from unittest.mock import MagicMock

from opendbc.car import DT_CTRL
from opendbc.car.structs import CarControl, CarParams, CarState
from opendbc.car.volkswagen.carcontroller import CarController, HCAMitigation
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, CarControllerParams as CCP, FW_QUERY_CONFIG, VolkswagenFlags, WMI
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu
VisualAlert = CarControl.HUDControl.VisualAlert

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


class TestVolkswagenMEBLateral(unittest.TestCase):
  """MEB HCA_03 power envelope ("torque bar") and LDW_02 steering-limit warning."""

  def setUp(self):
    CP = CarInterface.get_non_essential_params(CAR.VOLKSWAGEN_ID4_MK1.value)
    assert CP.flags & VolkswagenFlags.MEB, "ID.4 MK1 must have MEB flag"
    self.cc = CarController(CAR.VOLKSWAGEN_ID4_MK1.config.dbc_dict, CP)
    self.CCP = self.cc.CCP

    # Capture make_can_msg input values per message name without skipping real CAN encoding.
    self.captured = {}
    original_make = self.cc.packer_pt.make_can_msg
    def capture(name_or_addr, bus, values):
      self.captured[name_or_addr] = dict(values)
      return original_make(name_or_addr, bus, values)
    self.cc.packer_pt.make_can_msg = capture

  def _build_cc(self, lat_active=True, visual_alert=VisualAlert.none,
                left_lane_visible=False, right_lane_visible=False,
                left_lane_depart=False, right_lane_depart=False,
                actuator_curvature=0.0, current_curvature=0.0):
    CC = CarControl.new_message()
    CC.latActive = lat_active
    CC.enabled = lat_active
    CC.currentCurvature = current_curvature
    CC.actuators.curvature = actuator_curvature
    CC.hudControl.visualAlert = visual_alert
    CC.hudControl.leftLaneVisible = left_lane_visible
    CC.hudControl.rightLaneVisible = right_lane_visible
    CC.hudControl.leftLaneDepart = left_lane_depart
    CC.hudControl.rightLaneDepart = right_lane_depart
    return CC.as_reader()

  def _build_cs(self, steering_torque=0., v_ego=10., steering_pressed=False, measured_curvature=0.0):
    out = CarState.new_message()
    out.steeringTorque = steering_torque
    out.vEgo = v_ego
    out.steeringPressed = steering_pressed
    cs = MagicMock()
    cs.out = out
    cs.measured_curvature = measured_curvature
    cs.ldw_stock_values = {}
    cs.gra_stock_values = {"COUNTER": 0}
    return cs

  def _step(self, **kwargs):
    """Run one update cycle and return captured signal dicts."""
    self.captured.clear()
    cc = self._build_cc(**{k: v for k, v in kwargs.items() if k in (
      'lat_active', 'visual_alert', 'left_lane_visible', 'right_lane_visible',
      'left_lane_depart', 'right_lane_depart', 'actuator_curvature', 'current_curvature')})
    cs = self._build_cs(**{k: v for k, v in kwargs.items() if k in (
      'steering_torque', 'v_ego', 'steering_pressed', 'measured_curvature')})
    self.cc.update(cc, cs, 0)
    return self.captured

  def _run_until_steady(self, n, **kwargs):
    """Step N times and return the latest HCA_03 and LDW_02 values observed."""
    last = {"HCA_03": None, "LDW_02": None}
    for _ in range(n):
      msgs = self._step(**kwargs)
      for name in last:
        if name in msgs:
          last[name] = msgs[name]
    return last

  # ----- HCA_03 power envelope ("torque bar") -----

  def test_power_ramps_up_to_max_when_hands_off(self):
    """latActive + zero driver torque + cruise speed → steady-state power = STEERING_POWER_MAX."""
    ramp_frames = 2 * self.CCP.STEERING_POWER_MAX // self.CCP.STEERING_POWER_STEP + 4
    hca = self._run_until_steady(ramp_frames, lat_active=True, steering_torque=0., v_ego=10.)["HCA_03"]
    assert hca["Power"] == self.CCP.STEERING_POWER_MAX
    assert hca["RequestStatus"] == 4
    assert hca["HighSendRate"] == 1

  def test_power_ramps_down_to_min_at_full_driver_torque(self):
    """Driver torque ≥ STEER_DRIVER_MAX → steady-state power = STEERING_POWER_MIN."""
    ramp_frames = 2 * self.CCP.STEERING_POWER_MAX // self.CCP.STEERING_POWER_STEP + 4
    self._run_until_steady(ramp_frames, lat_active=True, steering_torque=0., v_ego=10.)
    hca = self._run_until_steady(ramp_frames, lat_active=True, steering_torque=self.CCP.STEER_DRIVER_MAX, v_ego=10.)["HCA_03"]
    assert hca["Power"] == self.CCP.STEERING_POWER_MIN

  def test_power_below_driver_allowance_holds_max(self):
    """Driver torque ≤ STEER_DRIVER_ALLOWANCE → power stays at MAX."""
    ramp_frames = 2 * self.CCP.STEERING_POWER_MAX // self.CCP.STEERING_POWER_STEP + 4
    hca = self._run_until_steady(ramp_frames, lat_active=True,
                                 steering_torque=self.CCP.STEER_DRIVER_ALLOWANCE - 1, v_ego=10.)["HCA_03"]
    assert hca["Power"] == self.CCP.STEERING_POWER_MAX

  def test_low_speed_cushion_holds_min(self):
    """At v_ego = 0 the low-speed cushion clamps target to STEERING_POWER_MIN regardless of driver torque."""
    ramp_frames = 2 * self.CCP.STEERING_POWER_MAX // self.CCP.STEERING_POWER_STEP + 4
    hca = self._run_until_steady(ramp_frames, lat_active=True, steering_torque=0., v_ego=0.)["HCA_03"]
    assert hca["Power"] == self.CCP.STEERING_POWER_MIN

  def test_disengage_ramps_power_to_zero(self):
    """latActive False from a powered-up state → power decrements by STEP per HCA frame until 0."""
    ramp_frames = 2 * self.CCP.STEERING_POWER_MAX // self.CCP.STEERING_POWER_STEP + 4
    self._run_until_steady(ramp_frames, lat_active=True, steering_torque=0., v_ego=10.)
    last_power = self.CCP.STEERING_POWER_MAX
    for _ in range(2 * (self.CCP.STEERING_POWER_MAX // self.CCP.STEERING_POWER_STEP) + 4):
      msgs = self._step(lat_active=False, steering_torque=0., v_ego=10.)
      if "HCA_03" in msgs:
        new_power = msgs["HCA_03"]["Power"]
        assert new_power == max(last_power - self.CCP.STEERING_POWER_STEP, 0), f"{new_power=} {last_power=}"
        last_power = new_power
    assert last_power == 0

  def test_request_status_falls_to_standby_when_disengaged(self):
    """After full disengage ramp, RequestStatus drops to 2 (standby) and HighSendRate to 0."""
    ramp_frames = 2 * self.CCP.STEERING_POWER_MAX // self.CCP.STEERING_POWER_STEP + 4
    self._run_until_steady(ramp_frames, lat_active=True, steering_torque=0., v_ego=10.)
    hca = self._run_until_steady(ramp_frames + 10, lat_active=False, steering_torque=0., v_ego=10.)["HCA_03"]
    assert hca["RequestStatus"] == 2
    assert hca["HighSendRate"] == 0
    assert hca["Power"] == 0

  def test_curvature_sign_bit_follows_command(self):
    """Curvature_VZ encodes sign: 1 for positive command, 0 for negative or zero."""
    hca = self._run_until_steady(2, lat_active=True, actuator_curvature=0.05)["HCA_03"]
    assert hca["Curvature_VZ"] == 1
    hca = self._run_until_steady(2, lat_active=True, actuator_curvature=-0.05)["HCA_03"]
    assert hca["Curvature_VZ"] == 0

  def test_curvature_clipped_to_max(self):
    """Curvature is clipped to ±CURVATURE_MAX."""
    hca = self._run_until_steady(2, lat_active=True, actuator_curvature=1.0)["HCA_03"]
    assert abs(hca["Curvature"] - self.CCP.CURVATURE_MAX) < 1e-6

  # ----- LDW_02 steering-limit warning -----

  def _ldw(self, **kwargs):
    """Step LDW_STEP frames and return the LDW_02 dict (sent once per LDW_STEP frames)."""
    msgs = {}
    for _ in range(self.CCP.LDW_STEP):
      msgs = self._step(**kwargs)
      if "LDW_02" in msgs:
        return msgs["LDW_02"]
    return msgs.get("LDW_02", {})

  def test_no_alert_default(self):
    """visualAlert=none → LDW_Texte=0."""
    ldw = self._ldw(lat_active=True, visual_alert=VisualAlert.none)
    assert ldw["LDW_Texte"] == 0

  def test_steer_required_alert(self):
    """visualAlert=steerRequired → LDW_Texte = laneAssistTakeOver (8)."""
    ldw = self._ldw(lat_active=True, visual_alert=VisualAlert.steerRequired)
    assert ldw["LDW_Texte"] == self.CCP.LDW_MESSAGES["laneAssistTakeOver"]

  def test_ldw_alert(self):
    """visualAlert=ldw → LDW_Texte = laneAssistTakeOver (8)."""
    ldw = self._ldw(lat_active=True, visual_alert=VisualAlert.ldw)
    assert ldw["LDW_Texte"] == self.CCP.LDW_MESSAGES["laneAssistTakeOver"]

  def test_yellow_led_when_pressed(self):
    """latActive + steeringPressed → LDW_Status_LED_gelb=1, gruen=0."""
    ldw = self._ldw(lat_active=True, steering_pressed=True)
    assert ldw["LDW_Status_LED_gelb"] == 1
    assert ldw["LDW_Status_LED_gruen"] == 0

  def test_green_led_when_not_pressed(self):
    """latActive + not pressed → LDW_Status_LED_gruen=1, gelb=0."""
    ldw = self._ldw(lat_active=True, steering_pressed=False)
    assert ldw["LDW_Status_LED_gruen"] == 1
    assert ldw["LDW_Status_LED_gelb"] == 0

  def test_no_leds_when_disengaged(self):
    """latActive=False → both LEDs off."""
    ldw = self._ldw(lat_active=False)
    assert ldw["LDW_Status_LED_gelb"] == 0
    assert ldw["LDW_Status_LED_gruen"] == 0

  def test_lane_visibility_passthrough(self):
    """leftLaneVisible/rightLaneVisible feed LDW_Lernmodus_* with display_mode offset."""
    # Active + visible: 1 (base) + 1 (visible) + 1 (display_mode active) = 3
    ldw = self._ldw(lat_active=True, left_lane_visible=True, right_lane_visible=True)
    assert ldw["LDW_Lernmodus_links"] == 3
    assert ldw["LDW_Lernmodus_rechts"] == 3

  def test_lane_depart_overrides_visibility(self):
    """leftLaneDepart/rightLaneDepart set LDW_Lernmodus_* to 3 + display_mode."""
    # Active + depart: 3 + 1 (display_mode) = 4
    ldw = self._ldw(lat_active=True, left_lane_depart=True, right_lane_depart=True)
    assert ldw["LDW_Lernmodus_links"] == 4
    assert ldw["LDW_Lernmodus_rechts"] == 4
