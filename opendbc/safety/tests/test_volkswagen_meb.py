#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

MAX_ACCEL = 2.0
MIN_ACCEL = -3.5
INACTIVE_ACCEL = 3.01

MAX_CURVATURE = 0.195
ANGLE_DEG_TO_CAN = 10  # 0.1 deg per CAN unit (steering wheel angle space inside the safety check)

# ID.4 vehicle model parameters, must match volkswagen_meb.h
SLIP_FACTOR = -0.0006055171512345705
STEER_RATIO = 15.6
WHEELBASE = 2.77


def curvature_to_angle_can(curvature, speed):
  """Convert curvature in rad/m to steering wheel angle in 0.1 deg using the bicycle model."""
  speed = max(speed, 1.0)
  cf = 1.0 / (1.0 - SLIP_FACTOR * speed * speed) / WHEELBASE
  angle_deg = curvature * STEER_RATIO / cf * 57.295779513
  return round(angle_deg * ANGLE_DEG_TO_CAN)

MSG_LH_EPS_03 = 0x9F
MSG_ESC_51 = 0xFC
MSG_MOTOR_51 = 0x10B
MSG_GRA_ACC_01 = 0x12B
MSG_QFK_01 = 0x13D
MSG_ACC_18 = 0x14D
MSG_MEB_ACC_01 = 0x300
MSG_HCA_03 = 0x303
MSG_LDW_02 = 0x397
MSG_MOTOR_14 = 0x3BE


class TestVolkswagenMebSafetyBase(common.CarSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02)}
  STANDSTILL_THRESHOLD = 0

  MAX_POWER = 125

  # Wheel speeds _esc_51_msg
  def _speed_msg(self, speed):
    values = {f"{s}_Radgeschw": speed * 3.6 for s in ("HL", "HR", "VL", "VR")}
    return self.packer.make_can_msg_safety("ESC_51", 0, values)

  def _speed_msg_2(self, speed):
    return None

  # Brake pedal switch
  def _motor_14_msg(self, brake):
    values = {"MO_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("Motor_14", 0, values)

  def _user_brake_msg(self, brake):
    return self._motor_14_msg(brake)

  # Driver throttle and ACC engagement status share the same frame
  def _motor_51_msg(self, gas=0, tsk_status=3):
    values = {"Accel_Pedal_Pressure": gas, "TSK_Status": tsk_status}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  def _user_gas_msg(self, gas):
    return self._motor_51_msg(gas=gas)

  def _tsk_status_msg(self, enable, main_switch=True):
    if main_switch:
      tsk_status = 3 if enable else 2
    else:
      tsk_status = 0
    return self._motor_51_msg(tsk_status=tsk_status)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  # Driver steering input torque
  def _torque_driver_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": torque < 0}
    return self.packer.make_can_msg_safety("LH_EPS_03", 0, values)

  # Measured curvature feedback
  def _curvature_meas_msg(self, curvature):
    values = {"Curvature": abs(curvature), "Curvature_VZ": curvature > 0}
    return self.packer.make_can_msg_safety("QFK_01", 0, values)

  # openpilot curvature command
  def _hca_03_msg(self, curvature, steer_req=True, power=125):
    values = {"Curvature": abs(curvature), "Curvature_VZ": curvature > 0,
              "RequestStatus": 4 if steer_req else 2, "Power": power * 0.4}
    return self.packer.make_can_msg_safety("HCA_03", 0, values)

  # Cruise control buttons
  def _gra_acc_01_msg(self, cancel=0, resume=0, _set=0, bus=0):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": _set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("GRA_ACC_01", bus, values)

  # Acceleration request to drivetrain coordinator
  def _acc_18_msg(self, accel):
    values = {"ACC_Sollbeschleunigung_02": accel}
    return self.packer.make_can_msg_safety("ACC_18", 0, values)

  def test_acc_status(self):
    # All TSK engaged states (3, 4, 5) enter controls; main switch on for 2; faulted for 0/6/7
    for status in (0, 2, 3, 4, 5, 6, 7):
      self._rx(self._motor_51_msg(tsk_status=status))
      self.assertEqual(status in (2, 3, 4, 5), self.safety.get_acc_main_on(), status)

  def test_torque_measurements(self):
    self._rx(self._torque_driver_msg(50))
    self._rx(self._torque_driver_msg(-50))
    for _ in range(4):
      self._rx(self._torque_driver_msg(0))

    self.assertEqual(-50, self.safety.get_torque_driver_min())
    self.assertEqual(50, self.safety.get_torque_driver_max())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(-50, self.safety.get_torque_driver_min())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(0, self.safety.get_torque_driver_min())

  def test_curvature_measurements(self):
    self._reset_speed_measurement(1)
    for curvature in (0.05, -0.05, 0.0):
      for _ in range(6):
        self._rx(self._curvature_meas_msg(curvature))
      expected = curvature_to_angle_can(curvature, 1.0)
      self.assertEqual(expected, self.safety.get_angle_meas_min())
      self.assertEqual(expected, self.safety.get_angle_meas_max())

  def test_steering_curvature_inactive(self):
    # Curvature must be exactly 0 when not actuating, regardless of controls_allowed
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertTrue(self._tx(self._hca_03_msg(0, steer_req=False, power=0)))
      for curvature in (-MAX_CURVATURE, -0.05, 0.05, MAX_CURVATURE):
        self.assertFalse(self._tx(self._hca_03_msg(curvature, steer_req=False, power=0)),
                         (controls_allowed, curvature))

  def test_steering_curvature_disallowed(self):
    # No curvature actuation allowed when controls are not allowed
    self.safety.set_controls_allowed(False)
    self._reset_speed_measurement(20)
    self._reset_curvature_measurement(0)
    self.assertFalse(self._tx(self._hca_03_msg(0, steer_req=True, power=125)))

  def test_steering_curvature_max(self):
    # Curvature limit is enforced; at low speed, ISO lateral accel allows up to MAX_CURVATURE
    self.safety.set_controls_allowed(True)
    self._reset_speed_measurement(0)
    self._reset_curvature_measurement(MAX_CURVATURE)
    self.safety.set_desired_angle_last(curvature_to_angle_can(MAX_CURVATURE, 1.0))
    self.assertTrue(self._tx(self._hca_03_msg(MAX_CURVATURE, steer_req=True, power=125)))
    # at 30 m/s, ISO lateral accel limit is well below 0.195 rad/m
    self._reset_speed_measurement(30)
    self.assertFalse(self._tx(self._hca_03_msg(MAX_CURVATURE, steer_req=True, power=125)))

  def test_steering_power_safety_check(self):
    self.safety.set_controls_allowed(True)
    self._reset_speed_measurement(20)
    self._reset_curvature_measurement(0)

    # Power must be within cap
    self.assertTrue(self._tx(self._hca_03_msg(0, steer_req=True, power=self.MAX_POWER)))
    self.assertFalse(self._tx(self._hca_03_msg(0, steer_req=True, power=self.MAX_POWER + 1)))

    # Power must be 0 when not actuating
    self.assertTrue(self._tx(self._hca_03_msg(0, steer_req=False, power=0)))
    self.assertFalse(self._tx(self._hca_03_msg(0, steer_req=False, power=1)))

  def _reset_speed_measurement(self, speed):
    for _ in range(6):
      self._rx(self._speed_msg(speed))

  def _reset_curvature_measurement(self, curvature):
    for _ in range(6):
      self._rx(self._curvature_meas_msg(curvature))


class TestVolkswagenMebStockSafety(TestVolkswagenMebSafetyBase):
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_GRA_ACC_01, 0], [MSG_GRA_ACC_01, 2]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_03, MSG_LDW_02]}

  def setUp(self):
    self.packer = CANPackerSafety("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMeb, 0)
    self.safety.init_tests()

  def test_spam_cancel_safety_check(self):
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._gra_acc_01_msg(cancel=1)))
    self.assertFalse(self._tx(self._gra_acc_01_msg(resume=1)))
    self.assertFalse(self._tx(self._gra_acc_01_msg(_set=1)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._gra_acc_01_msg(resume=1)))


class TestVolkswagenMebLongSafety(TestVolkswagenMebSafetyBase):
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_ACC_18, 0], [MSG_MEB_ACC_01, 0]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_03, MSG_LDW_02, MSG_ACC_18, MSG_MEB_ACC_01]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02, MSG_ACC_18, MSG_MEB_ACC_01)}

  def setUp(self):
    self.packer = CANPackerSafety("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMeb, VolkswagenSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  # stock cruise controls are entirely bypassed under openpilot longitudinal control
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_and_resume_buttons(self):
    for button in ("set", "resume"):
      # ACC main switch must be on, engage on falling edge
      self.safety.set_controls_allowed(0)
      self._rx(self._tsk_status_msg(False, main_switch=False))
      self._rx(self._gra_acc_01_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} with main switch off")
      self._rx(self._tsk_status_msg(False, main_switch=True))
      self._rx(self._gra_acc_01_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} rising edge")
      self._rx(self._gra_acc_01_msg(bus=0))
      self.assertTrue(self.safety.get_controls_allowed(), f"controls not allowed on {button} falling edge")

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._gra_acc_01_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")

  def test_main_switch(self):
    # Disable as soon as main switch turns off
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._tsk_status_msg(False, main_switch=False))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after ACC main switch off")

  def test_accel_safety_check(self):
    for controls_allowed in (True, False):
      for accel in np.concatenate((np.arange(MIN_ACCEL - 1, MAX_ACCEL + 1, 0.05), [0.0, INACTIVE_ACCEL])):
        accel = round(accel, 2)
        is_inactive = accel == INACTIVE_ACCEL
        # MEB allows accel=0 during driver gas override while controls remain allowed
        is_override = controls_allowed and accel == 0.0
        in_range = MIN_ACCEL <= accel <= MAX_ACCEL
        send = is_inactive or is_override or (controls_allowed and in_range)
        self.safety.set_controls_allowed(controls_allowed)
        self.assertEqual(send, self._tx(self._acc_18_msg(accel)), (controls_allowed, accel))


if __name__ == "__main__":
  unittest.main()
