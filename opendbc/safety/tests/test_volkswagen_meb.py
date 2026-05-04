#!/usr/bin/env python3
import math
import unittest
import numpy as np

from opendbc.car.structs import CarParams
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety


# Mirror of safety/modes/volkswagen_meb.h volkswagen_meb_curvature_to_angle
def curvature_to_angle(curvature_can, speed=1.0):
  slip = -0.0006055171512345705
  steer_ratio = 15.6
  wheelbase = 2.77
  cf = 1.0 / (1.0 - slip * speed * speed) / wheelbase
  curv = curvature_can * 6.7e-6
  return round(curv * steer_ratio / cf * math.degrees(1.0) * 10)

MSG_ESC_51    = 0xFC
MSG_QFK_01    = 0x13D
MSG_Motor_51  = 0x10B
MSG_GRA_ACC_01 = 0x12B
MSG_ACC_18    = 0x14D
MSG_TA_01     = 0x26B
MSG_MEB_ACC_01 = 0x300
MSG_HCA_03    = 0x303
MSG_LDW_02    = 0x397
MSG_MOTOR_14  = 0x3BE
MSG_LH_EPS_03 = 0x9F

MAX_ACCEL = 2.0
MIN_ACCEL = -3.5
INACTIVE_ACCEL = 3.01

CURVATURE_TO_CAN = 149253.7313  # 1 / 6.7e-06
MAX_CURVATURE = 29105            # 0.195 rad/m
MAX_POWER = 125                  # 50% duty


class TestVolkswagenMebSafetyBase(common.CarSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02)}
  STANDSTILL_THRESHOLD = 0

  def _speed_msg(self, speed):
    spd_kph = speed * 3.6
    values = {s: spd_kph for s in ("HL_Radgeschw", "HR_Radgeschw", "VL_Radgeschw", "VR_Radgeschw")}
    return self.packer.make_can_msg_safety("ESC_51", 0, values)

  def _speed_msg_2(self, speed):
    return None

  def _motor_14_msg(self, brake):
    values = {"MO_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("Motor_14", 0, values)

  def _user_brake_msg(self, brake):
    return self._motor_14_msg(brake)

  # driver throttle is on Motor_51
  def _user_gas_msg(self, gas):
    values = {"Accel_Pedal_Pressure": gas, "TSK_Status": 3}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  def _tsk_status_msg(self, enable, main_switch=True):
    if main_switch:
      tsk_status = 3 if enable else 2
    else:
      tsk_status = 0
    values = {"TSK_Status": tsk_status}
    return self.packer.make_can_msg_safety("Motor_51", 0, values)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  def _gra_acc_01_msg(self, cancel=0, resume=0, _set=0, bus=0):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": _set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("GRA_ACC_01", bus, values)

  def _acc_18_msg(self, accel):
    values = {"ACC_Sollbeschleunigung_02": accel}
    return self.packer.make_can_msg_safety("ACC_18", 0, values)

  def _lh_eps_03_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": 1 if torque < 0 else 0}
    return self.packer.make_can_msg_safety("LH_EPS_03", 0, values)

  def _hca_03_msg(self, curvature_raw, steer_req, power):
    values = {
      "Curvature":     abs(curvature_raw) / CURVATURE_TO_CAN,
      "Curvature_VZ":  1 if curvature_raw > 0 else 0,
      "Power":         power * 0.4,
      "RequestStatus": 4 if steer_req else 2,
    }
    return self.packer.make_can_msg_safety("HCA_03", 0, values)

  def _qfk_01_msg(self, curvature_raw):
    values = {
      "Curvature":    abs(curvature_raw) / CURVATURE_TO_CAN,
      "Curvature_VZ": 1 if curvature_raw > 0 else 0,
    }
    return self.packer.make_can_msg_safety("QFK_01", 0, values)

  def test_steer_power_safety_check(self):
    self.safety.set_controls_allowed(True)
    for p in (0, 1, MAX_POWER, MAX_POWER + 1, 200):
      self._tx(self._hca_03_msg(0, False, 0))  # reset prev power
      ok = self._tx(self._hca_03_msg(0, True, p))
      self.assertEqual(ok, p <= MAX_POWER, p)
    # power non-zero with steer_req off is always blocked
    self._tx(self._hca_03_msg(0, False, 0))
    self.assertFalse(self._tx(self._hca_03_msg(0, False, 1)))

  def test_steer_power_zero_when_controls_disabled(self):
    # vm-based check rejects steer_req=True while controls are off, no soft disengage path
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._hca_03_msg(0, True, 50)))
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._hca_03_msg(0, True, 1)))
    self.assertTrue(self._tx(self._hca_03_msg(0, False, 0)))

  def test_steer_curvature_max(self):
    # absolute curvature bound applied directly on HCA_03 payload
    self.safety.set_controls_allowed(True)
    for c in (0, MAX_CURVATURE, -MAX_CURVATURE, MAX_CURVATURE + 5, -MAX_CURVATURE - 5):
      # stage prev angle to the bicycle-model equivalent so rate check passes for in-range values
      self.safety.set_desired_angle_last(curvature_to_angle(c))
      ok = self._tx(self._hca_03_msg(c, True, 1))
      self.assertEqual(ok, abs(c) <= MAX_CURVATURE, c)

  def test_steer_inactive_curvature(self):
    # when steer_req is off, curvature must be zero
    for ca in (True, False):
      self.safety.set_controls_allowed(ca)
      self.assertTrue(self._tx(self._hca_03_msg(0, False, 0)))
      self.assertFalse(self._tx(self._hca_03_msg(100, False, 0)))
      self.assertFalse(self._tx(self._hca_03_msg(-100, False, 0)))

  def test_steer_curvature_rate_limit(self):
    # at speed, curvature step is limited by ISO lateral jerk (5 m/s^3)
    self.safety.set_controls_allowed(True)
    for v in (1.0, 10.0, 30.0):
      for _ in range(common.MAX_SAMPLE_VALS):
        self._rx(self._speed_msg(v))
      self._tx(self._hca_03_msg(0, True, 1))  # reset rate state
      # 5 / (v-1)^2 * send_rate * curvature_to_can — generous step always allowed near 0
      small_step = max(1, int(5.0 / max(v - 1, 1.0) ** 2 * 0.02 * CURVATURE_TO_CAN) // 2)
      self.assertTrue(self._tx(self._hca_03_msg(small_step, True, 1)), v)
      # huge jump always blocked
      self._tx(self._hca_03_msg(0, True, 1))
      self.assertFalse(self._tx(self._hca_03_msg(MAX_CURVATURE, True, 1)), v)

  def test_curvature_measurement(self):
    # QFK_01 RX hook updates internal curvature sample
    for c in (0, 100, -100, MAX_CURVATURE, -MAX_CURVATURE):
      self._rx(self._qfk_01_msg(c))

  def test_driver_torque_measurement(self):
    # LH_EPS_03 RX hook updates driver torque sample
    for t in (-200, 0, 200):
      self._rx(self._lh_eps_03_msg(t))


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
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._gra_acc_01_msg(resume=1)))


class TestVolkswagenMebLongSafety(TestVolkswagenMebSafetyBase):
  TX_MSGS = [[MSG_HCA_03, 0], [MSG_LDW_02, 0], [MSG_ACC_18, 0], [MSG_TA_01, 0], [MSG_MEB_ACC_01, 0]]
  FWD_BLACKLISTED_ADDRS = {2: [MSG_HCA_03, MSG_LDW_02, MSG_ACC_18, MSG_TA_01, MSG_MEB_ACC_01]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_03, MSG_LDW_02, MSG_ACC_18, MSG_TA_01, MSG_MEB_ACC_01)}
  INACTIVE_ACCEL = INACTIVE_ACCEL

  def setUp(self):
    self.packer = CANPackerSafety("vw_meb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenMeb, VolkswagenSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_and_resume_buttons(self):
    for button in ("set", "resume"):
      # Mmin switch off:  button has no effect
      self.safety.set_controls_allowed(0)
      self._rx(self._tsk_status_msg(False, main_switch=False))
      self._rx(self._gra_acc_01_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} with main switch off")
      # rising edge: no engage
      self._rx(self._tsk_status_msg(False, main_switch=True))
      self._rx(self._gra_acc_01_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} rising edge")
      # falling edge: engage
      self._rx(self._gra_acc_01_msg(bus=0))
      self.assertTrue(self.safety.get_controls_allowed(), f"controls not allowed on {button} falling edge")

  def test_cancel_button(self):
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._gra_acc_01_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")

  def test_main_switch(self):
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._tsk_status_msg(False, main_switch=False))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after ACC main switch off")

  def test_accel_safety_check(self):
    for controls_allowed in (True, False):
      for accel in np.concatenate((np.arange(MIN_ACCEL - 1, MAX_ACCEL + 1, 0.05), [0.0, INACTIVE_ACCEL])):
        accel = round(accel, 2)
        is_inactive = accel == INACTIVE_ACCEL
        # accel=0 (override) is allowed whenever controls are allowed
        is_override = controls_allowed and accel == 0.0
        in_range = MIN_ACCEL <= accel <= MAX_ACCEL
        send = is_inactive or is_override or (controls_allowed and in_range)
        self.safety.set_controls_allowed(controls_allowed)
        self.assertEqual(send, self._tx(self._acc_18_msg(accel)), (controls_allowed, accel))


if __name__ == "__main__":
  unittest.main()
