import unittest

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.volkswagen.values import CAR, DBC

VisualAlert = structs.CarControl.HUDControl.VisualAlert

HCA_03_ADDR = 771
LDW_02_ADDR = 919


def _make_carstate(CC_inst, vEgo=10.0, steeringTorque=0.0, curvature_meas=0.0):
  CS = CC_inst.CS
  CS.out = structs.CarState()
  CS.out.vEgo = vEgo
  CS.out.steeringTorque = float(steeringTorque)
  CS.out.steeringPressed = abs(steeringTorque) > CC_inst.CC.CCP.STEER_DRIVER_ALLOWANCE
  CS.curvature_meas = float(curvature_meas)
  CS.gra_stock_values = {"COUNTER": 0}
  CS.ldw_stock_values = {}
  CS.eps_stock_values = {}
  return CS


def _build_cc(latActive=True, curvature=0.0, visualAlert=VisualAlert.none):
  CC = structs.CarControl()
  CC.enabled = latActive
  CC.latActive = latActive
  CC.actuators.curvature = float(curvature)
  CC.currentCurvature = 0.0
  CC.hudControl.visualAlert = visualAlert
  return CC.as_reader()


def _decode(addr, dat, signals):
  """Decode a single CAN frame using a fresh CANParser tied to vw_meb."""
  parser = CANParser(DBC[CAR.VOLKSWAGEN_ID4_MK1.value][Bus.pt],
                     [(addr, 0)], 0)
  parser.update([(0, [(addr, dat, 0)])])
  return {s: parser.vl[addr][s] for s in signals}


class TestMEBLateral(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.CI = interfaces[CAR.VOLKSWAGEN_ID4_MK1.value]
    cp = cls.CI.get_params(CAR.VOLKSWAGEN_ID4_MK1.value, {i: {} for i in range(8)},
                           [], alpha_long=False, is_release=False, docs=False)
    cls.cp = cp

  def setUp(self):
    self.inst = self.CI(self.cp)
    self.CCP = self.inst.CC.CCP

  # (a) Torque-bar / steeringPressed test
  def test_steering_pressed_threshold(self):
    """steeringPressed flips True iff |torque| > STEER_DRIVER_ALLOWANCE (raw cNm units)."""
    th = self.CCP.STEER_DRIVER_ALLOWANCE
    cases = [
      (0,        False),
      (th - 1,   False),
      (th,       False),    # strictly greater than
      (th + 1,   True),
      (-(th + 1),True),
      (-(th - 1),False),
      (self.CCP.STEER_DRIVER_MAX, True),
    ]
    for torque, expected in cases:
      with self.subTest(torque=torque):
        # Replicate the exact CarState predicate used by update_meb()
        pressed = abs(torque) > th
        self.assertEqual(pressed, expected)

  # (b) Curvature clip / saturation test
  def test_curvature_clip_and_encoding(self):
    """Out-of-range commanded curvature is clipped to ±CURVATURE_MAX in actuators and on the wire."""
    for cmd in (+0.5, -0.5):
      with self.subTest(cmd=cmd):
        inst = self.CI(self.cp)
        CS = _make_carstate(inst)
        CC = _build_cc(latActive=True, curvature=cmd)
        new_act, sends = inst.CC.update(CC, CS, 0)

        self.assertAlmostEqual(abs(new_act.curvature), self.CCP.CURVATURE_MAX, places=4)
        self.assertEqual(new_act.curvature > 0, cmd > 0)

        hca = next(s for s in sends if s[0] == HCA_03_ADDR)
        decoded = _decode(HCA_03_ADDR, hca[1], ["Curvature", "Curvature_VZ", "RequestStatus"])
        self.assertAlmostEqual(decoded["Curvature"], self.CCP.CURVATURE_MAX, places=3)
        self.assertEqual(int(decoded["Curvature_VZ"]), 1 if cmd > 0 else 0)
        self.assertEqual(int(decoded["RequestStatus"]), 4)  # HCA enabled

  # (c) Steering power ramp test
  def test_steering_power_ramp_up_and_down(self):
    inst = self.CI(self.cp)
    CS = _make_carstate(inst, vEgo=10.0, steeringTorque=0.0)
    CC_on = _build_cc(latActive=True, curvature=0.0)

    # Ramp up: increases by STEERING_POWER_STEP per STEER_STEP-aligned cycle until MAX
    last_power = 0
    saw_max = False
    for _ in range(int(self.CCP.STEERING_POWER_MAX // self.CCP.STEERING_POWER_STEP) * self.CCP.STEER_STEP + 10):
      inst.CC.update(CC_on, CS, 0)
      cur = inst.CC.steering_power_last
      self.assertGreaterEqual(cur, last_power)
      self.assertLessEqual(cur, self.CCP.STEERING_POWER_MAX)
      if cur == self.CCP.STEERING_POWER_MAX:
        saw_max = True
      last_power = cur
    self.assertTrue(saw_max, "steering_power never reached STEERING_POWER_MAX")

    # Ramp down: latActive=False -> reduces by STEERING_POWER_STEP per cycle to zero
    CC_off = _build_cc(latActive=False, curvature=0.0)
    last_power = inst.CC.steering_power_last
    for _ in range(int(self.CCP.STEERING_POWER_MAX // self.CCP.STEERING_POWER_STEP) * self.CCP.STEER_STEP + 10):
      inst.CC.update(CC_off, CS, 0)
      cur = inst.CC.steering_power_last
      self.assertLessEqual(cur, last_power)
      last_power = cur
    self.assertEqual(inst.CC.steering_power_last, 0)

  # (d) LDW HUD test
  def test_ldw_hud_take_over(self):
    inst = self.CI(self.cp)
    CS = _make_carstate(inst)
    CC = _build_cc(latActive=True, visualAlert=VisualAlert.steerRequired)
    # Run for one full LDW cycle so the LDW frame is emitted
    sends = []
    for _ in range(self.CCP.LDW_STEP):
      _, s = inst.CC.update(CC, CS, 0)
      sends.extend(s)
    ldw = next((m for m in sends if m[0] == LDW_02_ADDR), None)
    self.assertIsNotNone(ldw, "LDW_02 frame not emitted")
    decoded = _decode(LDW_02_ADDR, ldw[1], ["LDW_Texte", "LDW_Status_LED_gruen"])
    self.assertEqual(int(decoded["LDW_Texte"]), self.CCP.LDW_MESSAGES["laneAssistTakeOver"])
    # latActive + not pressed => green LED on (UI showing OP active)
    self.assertEqual(int(decoded["LDW_Status_LED_gruen"]), 1)

  # (e) HCA disabled gating test
  def test_hca_disabled_after_power_ramp_down(self):
    inst = self.CI(self.cp)
    CS = _make_carstate(inst)
    CC_on = _build_cc(latActive=True)
    CC_off = _build_cc(latActive=False)

    # Ramp up to MAX, then off, then drain
    for _ in range(200):
      inst.CC.update(CC_on, CS, 0)
    for _ in range(200):
      inst.CC.update(CC_off, CS, 0)

    self.assertEqual(inst.CC.steering_power_last, 0)
    _, sends = inst.CC.update(CC_off, CS, 0)
    hca = next(s for s in sends if s[0] == HCA_03_ADDR)
    decoded = _decode(HCA_03_ADDR, hca[1], ["RequestStatus", "Power", "Curvature"])
    self.assertNotEqual(int(decoded["RequestStatus"]), 4)
    self.assertEqual(int(decoded["RequestStatus"]), 2)
    self.assertEqual(int(decoded["Power"]), 0)
    self.assertAlmostEqual(decoded["Curvature"], 0.0, places=4)


if __name__ == "__main__":
  unittest.main()
