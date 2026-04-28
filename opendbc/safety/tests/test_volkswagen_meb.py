#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags

# MEB message IDs
MSG_ESC_51 = 0xFC
MSG_QFK_01 = 0x13D
MSG_Motor_51 = 0x10B
MSG_HCA_03 = 0x303
MSG_GRA_ACC_01 = 0x12B
MSG_LDW_02 = 0x397
MSG_MOTOR_14 = 0x3BE
MSG_LH_EPS_03 = 0x9F


class TestVolkswagenMebSafetyBase(common.CarSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02)}
  STANDSTILL_THRESHOLD = 0

  # Wheel speeds (ESC_51, MEB ABS message)
  def _speed_msg(self, speed):
    spd_kph = speed * 3.6
    values = {s: spd_kph for s in ("HL_Radgeschw", "HR_Radgeschw", "VL_Radgeschw", "VR_Radgeschw")}
    return self.packer.make_can_msg_safety("ESC_51", 0, values)

  def _speed_msg_2(self, speed):
    return None

  # Brake pedal switch
  def _motor_14_msg(self, brake):
    values = {"MO_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("Motor_14", 0, values)

  def _user_brake_msg(self, brake):
    return self._motor_14_msg(brake)

  # Driver throttle input lives on Motor_51 alongside ACC status. We emit TSK_Status=3 (ACC active)
  # so the cruise-engaged gating doesn't disengage between gas-only assertions.
  def _user_gas_msg(self, gas):
    values = {"Accel_Pedal_Pressure": gas, "TSK_Status": 3}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  # ACC engagement status (Motor_51.TSK_Status: 0=off, 2=ready, 3-5=active, 6-7=fault)
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
  def _gra_acc_01_msg(self, cancel=0, resume=0, _set=0, bus=0):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": _set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("GRA_ACC_01", bus, values)


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


class TestVolkswagenMebGen2StockSafety(TestVolkswagenMebStockSafety):
  def setUp(self):
    self.packer = CANPackerSafety("vw_meb_2024")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMeb, VolkswagenSafetyFlags.ALT_CRC_VARIANT_1)
    self.safety.init_tests()


if __name__ == "__main__":
  unittest.main()
