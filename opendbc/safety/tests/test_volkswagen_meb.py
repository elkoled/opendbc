#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, MAX_SAMPLE_VALS, sign_of


class CANPackerSafetyMEB(CANPackerSafety):
  pass

# MEB message IDs
MSG_ESC_51        = 0xFC
MSG_QFK_01        = 0x13D
MSG_MOTOR_51      = 0x10B
MSG_HCA_03        = 0x303
MSG_GRA_ACC_01    = 0x12B
MSG_LDW_02        = 0x397
MSG_MOTOR_14      = 0x3BE
MSG_LH_EPS_03     = 0x9F


class TestVolkswagenMebSafety(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_GRA_ACC_01, 0], [MSG_GRA_ACC_01, 2], [MSG_LH_EPS_03, 2]]
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02), 2: (MSG_LH_EPS_03,)}
  FWD_BLACKLISTED_ADDRS = {0: [MSG_LH_EPS_03], 2: [MSG_HCA_03, MSG_LDW_02]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  # AngleSteeringSafetyTest required attrs (curvature mode: values are rad/m)
  STEER_ANGLE_MAX = 0.195
  DEG_TO_CAN = 149253.7313  # rad/m to CAN
  ANGLE_RATE_BP = [5., 25., 25.]
  ANGLE_RATE_UP = [0.02, 0.008, 0.008]
  ANGLE_RATE_DOWN = [0.02, 0.008, 0.008]

  def setUp(self):
    self.packer = CANPackerSafetyMEB("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMeb, 0)
    self.safety.init_tests()

  # Wheel speeds
  def _speed_msg(self, speed):
    spd_kph = speed * 3.6
    values = {"HL_Radgeschw": spd_kph, "HR_Radgeschw": spd_kph, "VL_Radgeschw": spd_kph, "VR_Radgeschw": spd_kph}
    return self.packer.make_can_msg_safety("ESC_51", 0, values)

  def _vehicle_moving_msg(self, speed):
    return self._speed_msg(speed)

  # Brake pedal switch
  def _user_brake_msg(self, brake):
    values = {"MO_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("Motor_14", 0, values)

  # Driver throttle input. Motor_51 also carries TSK_Status; keep main switch on to avoid
  # spuriously dropping controls_allowed when only testing the gas signal.
  def _user_gas_msg(self, gas):
    values = {"Accel_Pedal_Pressure": gas, "TSK_Status": 3}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  # Measured curvature
  def _angle_meas_msg(self, angle):
    values = {"Curvature": abs(angle), "Curvature_VZ": angle > 0}
    return self.packer.make_can_msg_safety("QFK_01", 0, values)

  # Commanded curvature
  def _angle_cmd_msg(self, angle, enabled, increment_timer: bool = True):
    values = {
      "Curvature": abs(angle),
      "Curvature_VZ": angle > 0,
      "RequestStatus": 4 if enabled else 0,
      "Power": 50,
    }
    return self.packer.make_can_msg_safety("HCA_03", 0, values)

  # ACC engagement status
  def _tsk_status_msg(self, enable, main_switch=True):
    if main_switch:
      tsk_status = 3 if enable else 2
    else:
      tsk_status = 0
    values = {"TSK_Status": tsk_status}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  # Cruise control buttons
  def _gra_acc_01_msg(self, cancel=0, resume=0, _set=0, bus=2):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": _set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("GRA_ACC_01", bus, values)

  # Override degree-based ranges with curvature-appropriate ranges (max ~0.195 rad/m)
  def test_steering_angle_measurements(self):
    # values are curvature in rad/m
    msg_func = self._angle_meas_msg
    min_val = -self.STEER_ANGLE_MAX
    max_val = self.STEER_ANGLE_MAX
    factor = self.DEG_TO_CAN
    meas_min_func = self.safety.get_angle_meas_min
    meas_max_func = self.safety.get_angle_meas_max
    for val in (min_val, max_val):
      for _ in range(MAX_SAMPLE_VALS):
        self.assertTrue(self._rx(msg_func(val)))
      self.assertAlmostEqual(meas_min_func() / factor, val, delta=0.01)
      self.assertAlmostEqual(meas_max_func() / factor, val, delta=0.01)
      # reset
      for _ in range(MAX_SAMPLE_VALS):
        self.assertTrue(self._rx(msg_func(0)))

  def test_angle_cmd_when_enabled(self):
    # Only test at low speeds; at higher speeds ISO lateral accel caps curvature below STEER_ANGLE_MAX
    speeds = [0., 1.]
    # leave headroom so a + max_delta stays within max curvature
    angles = np.concatenate((np.arange(-self.STEER_ANGLE_MAX + 0.03, self.STEER_ANGLE_MAX - 0.03, 0.02), [0]))
    for a in angles:
      for s in speeds:
        max_delta_up = np.interp(s, self.ANGLE_RATE_BP, self.ANGLE_RATE_UP)
        max_delta_down = np.interp(s, self.ANGLE_RATE_BP, self.ANGLE_RATE_DOWN)

        self._reset_angle_measurement(a)
        self._reset_speed_measurement(s)

        self._set_prev_desired_angle(a)
        self.safety.set_controls_allowed(1)

        self.assertTrue(self._tx(self._angle_cmd_msg(a + sign_of(a) * max_delta_up, True)))
        self.assertTrue(self.safety.get_controls_allowed())

        self.assertTrue(self._tx(self._angle_cmd_msg(a, True)))
        self.assertTrue(self.safety.get_controls_allowed())

        self.assertTrue(self._tx(self._angle_cmd_msg(a - sign_of(a) * max_delta_down, True)))
        self.assertTrue(self.safety.get_controls_allowed())

        # Inject too high rates (use a small absolute over-step in curvature units)
        over = max(max_delta_up, max_delta_down) + 0.005
        self.assertFalse(self._tx(self._angle_cmd_msg(a + sign_of(a) * (max_delta_up + 0.005), True)))

        self.safety.set_controls_allowed(1)
        self._set_prev_desired_angle(a)
        self.assertTrue(self.safety.get_controls_allowed())
        self.assertTrue(self._tx(self._angle_cmd_msg(a, True)))
        self.assertTrue(self.safety.get_controls_allowed())

        self.assertFalse(self._tx(self._angle_cmd_msg(a - sign_of(a) * (max_delta_down + 0.005), True)))

        self.safety.set_controls_allowed(0)
        # inactive_angle_is_zero: only zero curvature allowed when disabled
        should_tx = (a == 0)
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(a, False)))
        _ = over  # silence linter

  def test_angle_cmd_when_disabled(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      for steer_control_enabled in (True, False):
        for angle_meas in np.arange(-self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX + 0.001, 0.02):
          self._reset_angle_measurement(angle_meas)

          for angle_cmd in np.arange(-self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX + 0.001, 0.02):
            self._set_prev_desired_angle(angle_cmd)

            # inactive_angle_is_zero: when steer_control_enabled is False, only zero curvature is allowed
            should_tx = controls_allowed if steer_control_enabled else angle_cmd == 0
            self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(angle_cmd, steer_control_enabled)))

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._gra_acc_01_msg(cancel=1)))
    self.assertFalse(self._tx(self._gra_acc_01_msg(resume=1)))
    self.assertFalse(self._tx(self._gra_acc_01_msg(_set=1)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._gra_acc_01_msg(resume=1)))


if __name__ == "__main__":
  unittest.main()
