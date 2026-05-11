#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

MSG_ESC_51     = 0xFC    # RX, for wheel speeds
MSG_LH_EPS_03  = 0x9F    # RX from EPS, for driver steering torque
MSG_Motor_51   = 0x10B   # RX, for TSK state and accel pedal
MSG_GRA_ACC_01 = 0x12B   # TX by OP, ACC control buttons for cancel/resume
MSG_QFK_01     = 0x13D   # RX, for measured curvature
MSG_HCA_03     = 0x303   # TX by OP, Heading Control Assist curvature
MSG_LDW_02     = 0x397   # TX by OP, Lane line recognition and text alerts
MSG_MOTOR_14   = 0x3BE   # RX, for brake switch status


class TestVolkswagenMebSafetyBase(common.CarSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02)}

  # Wheel speeds
  def _speed_msg(self, speed):
    values = {f"{s}_Radgeschw": speed for s in ["HL", "HR", "VL", "VR"]}
    return self.packer.make_can_msg_safety("ESC_51", 0, values)

  # Brake pedal switch
  def _motor_14_msg(self, brake):
    values = {"MO_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("Motor_14", 0, values)

  def _user_brake_msg(self, brake):
    return self._motor_14_msg(brake)

  # Driver throttle input. Motor_51 also carries TSK_Status; keep cruise engaged so
  # pcm_cruise_check doesn't force-disable on every Motor_51 rx during gas tests.
  def _user_gas_msg(self, gas):
    values = {"Accel_Pedal_Pressure": gas, "TSK_Status": 3}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  # ACC engagement status via TSK
  def _tsk_status_msg(self, enable, main_switch=True):
    if main_switch:
      tsk_status = 3 if enable else 2
    else:
      tsk_status = 0
    values = {"TSK_Status": tsk_status}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  # Driver steering input torque
  def _torque_driver_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": torque < 0}
    return self.packer.make_can_msg_safety("LH_EPS_03", 0, values)

  # Cruise control buttons
  def _gra_acc_01_msg(self, cancel=0, resume=0, _set=0, bus=2):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": _set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("GRA_ACC_01", bus, values)


class TestVolkswagenMebStockSafety(TestVolkswagenMebSafetyBase):
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_GRA_ACC_01, 0], [MSG_GRA_ACC_01, 2]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_03, MSG_LDW_02]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def setUp(self):
    self.packer = CANPackerSafety("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMeb, 0)
    self.safety.init_tests()

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self._rx(self._tsk_status_msg(False))
    self.safety.set_controls_allowed(1)
    self._rx(self._gra_acc_01_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")


if __name__ == "__main__":
  unittest.main()
