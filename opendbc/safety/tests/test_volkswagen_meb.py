#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

MSG_LH_EPS_03 = 0x9F    # RX from EPS, for driver steering torque
MSG_ESC_51 = 0xFC       # RX from ABS, for wheel speeds
MSG_Motor_51 = 0x10B    # RX from ECU, for ACC status / accel pedal
MSG_GRA_ACC_01 = 0x12B  # TX by OP, ACC control buttons for cancel/resume
MSG_QFK_01 = 0x13D      # RX from EPS, for steering curvature
MSG_HCA_03 = 0x303      # TX by OP, Heading Control Assist curvature
MSG_LDW_02 = 0x397      # TX by OP, Lane line recognition and text alerts
MSG_MOTOR_14 = 0x3BE    # RX from ECU, for brake switch status


class TestVolkswagenMebStockSafety(common.CarSafetyTest):
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02)}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_03, MSG_LDW_02]}
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_GRA_ACC_01, 0], [MSG_GRA_ACC_01, 2]]

  def setUp(self):
    self.packer = CANPackerSafety("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMeb, 0)
    self.safety.init_tests()

  # Wheel speeds
  def _speed_msg(self, speed):
    # speed is m/s; ESC_51 wheel speeds use 0.0075 / 3.6 m/s per LSB based on safety hook
    val = int(speed * 3.6 / 0.0075)
    values = {f"{s}_Radgeschw": val for s in ["VL", "VR", "HL", "HR"]}
    return self.packer.make_can_msg_safety("ESC_51", 0, values)

  def _user_brake_msg(self, brake):
    values = {"MO_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("Motor_14", 0, values)

  def _user_gas_msg(self, gas):
    # Gas and TSK share Motor_51; assume cruise stays engaged when gas tests run
    values = {"Accel_Pedal_Pressure": 1 if gas else 0, "TSK_Status": 3}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"TSK_Status": 3 if enable else 2}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  def _gra_acc_01_msg(self, cancel=0, resume=0, _set=0, bus=2):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": _set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("GRA_ACC_01", bus, values)

  def _hca_03_msg(self, curvature, steer_req=True):
    # Curvature in rad/m
    values = {
      "Curvature": abs(curvature),
      "Curvature_VZ": 1 if curvature > 0 else 0,
      "Power": 50 if steer_req else 0,
      "RequestStatus": 4 if steer_req else 2,
    }
    return self.packer.make_can_msg_safety("HCA_03", 0, values)

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
