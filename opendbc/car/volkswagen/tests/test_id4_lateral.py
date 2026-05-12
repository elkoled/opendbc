import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import CurvatureSteeringLimits, apply_std_curvature_limits
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, CanBus, CarControllerParams, DBC, VolkswagenFlags


def _build_cp():
  CP = CarInterface.get_non_essential_params(CAR.VOLKSWAGEN_ID4_MK1)
  return CP


def _msg(packer, name, bus, values):
  return packer.make_can_msg(name, bus, values)


def _bytes(b):
  return b if isinstance(b, (bytes, bytearray)) else bytes(b)


class TestId4TorqueBar(unittest.TestCase):
  """The 'torque bar' in the UI reflects ret.steeringTorque and ret.steeringPressed.
  We verify that LH_EPS_03 is parsed with correct sign and that the steeringPressed
  threshold (STEER_DRIVER_ALLOWANCE = 0.6 Nm) is applied correctly."""

  def setUp(self):
    self.CP = _build_cp()
    self.CS = CarState(self.CP)
    self.parsers = CarState.get_can_parsers(self.CP)
    self.packer = CANPacker(DBC[self.CP.carFingerprint][Bus.pt])
    self.CAN = CanBus(self.CP)
    self.CCP = CarControllerParams(self.CP)

  def _tick(self, eps_torque_signed):
    # Feed enough messages so the parser sees all required signals
    sign = 1 if eps_torque_signed < 0 else 0  # EPS_VZ_Lenkmoment: 1 means negative
    msgs = [
      _msg(self.packer, "LH_EPS_03", self.CAN.pt, {
        "EPS_Lenkmoment": abs(eps_torque_signed),
        "EPS_VZ_Lenkmoment": sign,
      }),
      _msg(self.packer, "QFK_01", self.CAN.pt, {"LatCon_HCA_Status": 2}),
      _msg(self.packer, "Motor_51", self.CAN.pt, {"TSK_Status": 2, "Accel_Pedal_Pressure": 0}),
      _msg(self.packer, "Motor_14", self.CAN.pt, {"MO_Fahrer_bremst": 0}),
      _msg(self.packer, "ESC_51", self.CAN.pt, {
        "VL_Radgeschw": 0, "VR_Radgeschw": 0, "HL_Radgeschw": 0, "HR_Radgeschw": 0,
        "Brake_Pressure": 0,
      }),
      _msg(self.packer, "ESC_50", self.CAN.pt, {"Yaw_Rate": 0, "Yaw_Rate_Sign": 0}),
      _msg(self.packer, "Getriebe_11", self.CAN.pt, {"GE_Fahrstufe": 8}),  # drive
      _msg(self.packer, "Airbag_02", self.CAN.pt, {"AB_Gurtschloss_FA": 3}),
      _msg(self.packer, "Gateway_72", self.CAN.pt, {
        "ZV_02_alt": 0, "ZV_FT_offen": 0, "ZV_BT_offen": 0, "ZV_HFS_offen": 0,
        "ZV_HBFS_offen": 0, "ZV_HD_offen": 0,
      }),
      _msg(self.packer, "Gateway_73", self.CAN.pt, {"EPB_Status": 0, "GE_Fahrstufe": 8}),
      _msg(self.packer, "ESP_21", self.CAN.pt, {"ESP_Tastung_passiv": 0, "ESP_Eingriff": 0}),
      _msg(self.packer, "Blinkmodi_02", self.CAN.pt, {}),
      _msg(self.packer, "SMLS_01", self.CAN.pt, {"BH_Blinker_li": 0, "BH_Blinker_re": 0}),
      _msg(self.packer, "GRA_ACC_01", self.CAN.pt, {"GRA_Typ_Hauptschalter": 1}),
    ]
    cam_msgs = [_msg(self.packer, "LDW_02", self.CAN.cam, {})]
    # First pass: trigger lazy message subscription in the parser, then re-send to capture values.
    # Loop several times so the debounce in update_steering_pressed (5 frames) can latch.
    self.CS.update(self.parsers)
    ret = None
    for _ in range(10):
      self.parsers[Bus.pt].update([0, msgs])
      self.parsers[Bus.cam].update([0, cam_msgs])
      ret = self.CS.update(self.parsers)
    return ret

  def test_zero_torque(self):
    ret = self._tick(0)
    self.assertEqual(ret.steeringTorque, 0.0)
    self.assertFalse(ret.steeringPressed)

  def test_positive_torque_below_threshold(self):
    # 50 < STEER_DRIVER_ALLOWANCE (60) → not pressed
    ret = self._tick(50)
    self.assertEqual(ret.steeringTorque, 50.0)
    self.assertFalse(ret.steeringPressed)

  def test_positive_torque_above_threshold(self):
    ret = self._tick(100)
    self.assertEqual(ret.steeringTorque, 100.0)
    self.assertTrue(ret.steeringPressed)

  def test_negative_torque_above_threshold(self):
    ret = self._tick(-100)
    self.assertEqual(ret.steeringTorque, -100.0)
    self.assertTrue(ret.steeringPressed)

  def test_threshold_exact(self):
    # Equal to threshold: NOT pressed (strict >)
    ret = self._tick(self.CCP.STEER_DRIVER_ALLOWANCE)
    self.assertFalse(ret.steeringPressed)
    ret = self._tick(self.CCP.STEER_DRIVER_ALLOWANCE + 1)
    self.assertTrue(ret.steeringPressed)


class TestId4SteeringLimitWarning(unittest.TestCase):
  """The 'steering limit warning' is driven by ret.steerFaultTemporary / steerFaultPermanent,
  which come from QFK_01 LatCon_HCA_Status via update_hca_state(). We verify each EPS state
  maps to the correct UI warning category once EPS has initialized."""

  def setUp(self):
    self.CP = _build_cp()
    self.CS = CarState(self.CP)
    self.CS.frame = 700  # past eps_init grace period
    self.CS.eps_init_complete = True

  def _check(self, hca_state):
    # drive_mode=True; eps_init_complete already True
    return self.CS.update_hca_state(hca_state, drive_mode=True)

  def test_ready_no_fault(self):
    temp, perm = self._check("READY")
    self.assertFalse(temp)
    self.assertFalse(perm)

  def test_active_no_fault(self):
    temp, perm = self._check("ACTIVE")
    self.assertFalse(temp)
    self.assertFalse(perm)

  def test_rejected_temp_fault(self):
    temp, perm = self._check("REJECTED")
    self.assertTrue(temp)
    self.assertFalse(perm)

  def test_preempted_temp_fault(self):
    temp, perm = self._check("PREEMPTED")
    self.assertTrue(temp)
    self.assertFalse(perm)

  def test_disabled_perm_fault(self):
    temp, perm = self._check("DISABLED")
    self.assertFalse(temp)
    self.assertTrue(perm)

  def test_fault_perm_fault_after_init(self):
    temp, perm = self._check("FAULT")
    self.assertFalse(temp)
    self.assertTrue(perm)

  def test_fault_before_init_is_temp(self):
    # Before EPS init, FAULT is treated as temporary (give EPS time to recover)
    self.CS.eps_init_complete = False
    self.CS.frame = 0
    temp, perm = self.CS.update_hca_state("FAULT", drive_mode=True)
    self.assertTrue(temp)
    self.assertFalse(perm)


class TestId4CurvatureLimits(unittest.TestCase):
  """The curvature output to HCA_03 must never exceed CURVATURE_MAX (0.195 rad/m),
  must respect ISO 11270 jerk and lateral-accel limits, and must follow the measured
  curvature when openpilot is inactive (steer_required=False)."""

  def setUp(self):
    self.CCP = CarControllerParams(_build_cp())
    self.LIMITS = self.CCP.CURVATURE_LIMITS

  def test_clamps_to_max(self):
    # At low speed the lateral-accel cap is way above CURVATURE_MAX, so the final hard clamp wins
    apply = apply_std_curvature_limits(1.0, 1.0, 1.0, 0.0, False, self.CCP.STEER_STEP, True, self.LIMITS)
    self.assertAlmostEqual(apply, self.LIMITS.CURVATURE_MAX, places=4)
    apply = apply_std_curvature_limits(-1.0, -1.0, 1.0, 0.0, False, self.CCP.STEER_STEP, True, self.LIMITS)
    self.assertAlmostEqual(apply, -self.LIMITS.CURVATURE_MAX, places=4)

  def test_inactive_returns_meas(self):
    # When lat_active is False, output follows current measured curvature
    measured = 0.07
    apply = apply_std_curvature_limits(0.15, 0.0, 30.0, measured, False, self.CCP.STEER_STEP, False, self.LIMITS)
    self.assertAlmostEqual(apply, measured, places=4)

  def test_jerk_rate_limited(self):
    # Starting from 0 with vEgo=30, single 20ms step → very small max delta
    apply = apply_std_curvature_limits(1.0, 0.0, 30.0, 0.0, False, self.CCP.STEER_STEP, True, self.LIMITS)
    self.assertLess(apply, self.LIMITS.CURVATURE_MAX, "should be rate-limited, not jump straight to max")

  def test_lat_accel_speed_dependent(self):
    # At very high speed, the lateral-acceleration cap kicks in well below CURVATURE_MAX
    apply_fast = apply_std_curvature_limits(1.0, 1.0, 50.0, 0.0, False, self.CCP.STEER_STEP, True, self.LIMITS)
    apply_slow = apply_std_curvature_limits(1.0, 1.0, 5.0, 0.0, False, self.CCP.STEER_STEP, True, self.LIMITS)
    self.assertLess(apply_fast, apply_slow,
                    "lateral-accel cap should reduce max curvature as speed grows")


class TestId4Blindspot(unittest.TestCase):
  """The blind-spot LEDs/warnings should reflect MEB_Side_Assist_01 info+warn bits
  on whichever side of the gateway relay the side-radar publishes (ext_cp)."""

  def setUp(self):
    self.CP = _build_cp()
    # Force enableBsm and gateway network location so ext_cp == cam bus
    self.CP.enableBsm = True
    self.CP.networkLocation = structs.CarParams.NetworkLocation.gateway
    self.CS = CarState(self.CP)
    self.parsers = CarState.get_can_parsers(self.CP)
    self.packer = CANPacker(DBC[self.CP.carFingerprint][Bus.pt])
    self.CAN = CanBus(self.CP)

  def _tick(self, info_left=0, warn_left=0, info_right=0, warn_right=0):
    pt_msgs = [
      _msg(self.packer, "LH_EPS_03", self.CAN.pt, {"EPS_Lenkmoment": 0, "EPS_VZ_Lenkmoment": 0}),
      _msg(self.packer, "QFK_01", self.CAN.pt, {"LatCon_HCA_Status": 2}),
      _msg(self.packer, "Motor_51", self.CAN.pt, {"TSK_Status": 2, "Accel_Pedal_Pressure": 0}),
      _msg(self.packer, "Motor_14", self.CAN.pt, {"MO_Fahrer_bremst": 0}),
      _msg(self.packer, "ESC_51", self.CAN.pt, {
        "VL_Radgeschw": 0, "VR_Radgeschw": 0, "HL_Radgeschw": 0, "HR_Radgeschw": 0,
        "Brake_Pressure": 0,
      }),
      _msg(self.packer, "ESC_50", self.CAN.pt, {"Yaw_Rate": 0, "Yaw_Rate_Sign": 0}),
      _msg(self.packer, "Getriebe_11", self.CAN.pt, {"GE_Fahrstufe": 8}),
      _msg(self.packer, "Airbag_02", self.CAN.pt, {"AB_Gurtschloss_FA": 3}),
      _msg(self.packer, "Gateway_72", self.CAN.pt, {}),
      _msg(self.packer, "Gateway_73", self.CAN.pt, {"EPB_Status": 0, "GE_Fahrstufe": 8}),
      _msg(self.packer, "ESP_21", self.CAN.pt, {}),
      _msg(self.packer, "Blinkmodi_02", self.CAN.pt, {}),
      _msg(self.packer, "SMLS_01", self.CAN.pt, {}),
      _msg(self.packer, "GRA_ACC_01", self.CAN.pt, {"GRA_Typ_Hauptschalter": 1}),
    ]
    # BSM message lives on ext_cp; for gateway-harness ID.4 that's the cam bus
    cam_msgs = [
      _msg(self.packer, "LDW_02", self.CAN.cam, {}),
      _msg(self.packer, "MEB_Side_Assist_01", self.CAN.cam, {
        "Blind_Spot_Info_Left": info_left,
        "Blind_Spot_Warn_Left": warn_left,
        "Blind_Spot_Info_Right": info_right,
        "Blind_Spot_Warn_Right": warn_right,
      }),
    ]
    self.CS.update(self.parsers)  # prime lazy subscription
    self.parsers[Bus.pt].update([0, pt_msgs])
    self.parsers[Bus.cam].update([0, cam_msgs])
    return self.CS.update(self.parsers)

  def test_no_blindspot(self):
    ret = self._tick()
    self.assertFalse(ret.leftBlindspot)
    self.assertFalse(ret.rightBlindspot)

  def test_left_info_only(self):
    ret = self._tick(info_left=1)
    self.assertTrue(ret.leftBlindspot)
    self.assertFalse(ret.rightBlindspot)

  def test_left_warn_only(self):
    ret = self._tick(warn_left=1)
    self.assertTrue(ret.leftBlindspot)
    self.assertFalse(ret.rightBlindspot)

  def test_right_info_only(self):
    ret = self._tick(info_right=1)
    self.assertFalse(ret.leftBlindspot)
    self.assertTrue(ret.rightBlindspot)

  def test_right_warn_only(self):
    ret = self._tick(warn_right=1)
    self.assertFalse(ret.leftBlindspot)
    self.assertTrue(ret.rightBlindspot)

  def test_both_sides(self):
    ret = self._tick(info_left=1, warn_right=1)
    self.assertTrue(ret.leftBlindspot)
    self.assertTrue(ret.rightBlindspot)

  def test_bsm_disabled(self):
    # When enableBsm=False, the carstate should not touch the blindspot fields
    self.CP.enableBsm = False
    self.CS = CarState(self.CP)
    self.parsers = CarState.get_can_parsers(self.CP)
    ret = self._tick(info_left=1, info_right=1)
    self.assertFalse(ret.leftBlindspot)
    self.assertFalse(ret.rightBlindspot)


class TestId4HCAMessage(unittest.TestCase):
  """The HCA_03 message that hits the wire must encode the right curvature, sign,
  power, and request status. Critical for the EPS to accept the command."""

  def setUp(self):
    self.CP = _build_cp()
    self.packer = CANPacker(DBC[self.CP.carFingerprint][Bus.pt])

  def _decode(self, msg, packer):
    from opendbc.can import CANParser
    parser = CANParser(DBC[self.CP.carFingerprint][Bus.pt], [("HCA_03", 0)], 0)
    parser.update([0, [msg]])
    return parser.vl["HCA_03"]

  def test_steering_control_enabled(self):
    from opendbc.car.volkswagen import mebcan
    msg = mebcan.create_steering_control(self.packer, 0, 0.05, True, 50)
    vals = self._decode(msg, self.packer)
    self.assertAlmostEqual(vals["Curvature"], 0.05, places=4)
    self.assertEqual(vals["Curvature_VZ"], 1)
    self.assertEqual(vals["Power"], 50)
    self.assertEqual(vals["RequestStatus"], 4)
    self.assertEqual(vals["HighSendRate"], 1)

  def test_steering_control_negative(self):
    from opendbc.car.volkswagen import mebcan
    msg = mebcan.create_steering_control(self.packer, 0, -0.05, True, 50)
    vals = self._decode(msg, self.packer)
    self.assertAlmostEqual(vals["Curvature"], 0.05, places=4)
    self.assertEqual(vals["Curvature_VZ"], 0)

  def test_steering_control_disabled(self):
    from opendbc.car.volkswagen import mebcan
    msg = mebcan.create_steering_control(self.packer, 0, 0.05, False, 50)
    vals = self._decode(msg, self.packer)
    self.assertEqual(vals["Power"], 0)
    self.assertEqual(vals["RequestStatus"], 2)
    self.assertEqual(vals["HighSendRate"], 0)


if __name__ == "__main__":
  unittest.main()
