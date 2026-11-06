#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.lateral import AngleSteeringLimitsVM, get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.structs import CarParams
from opendbc.car.psa.carcontroller import get_safety_CP
from opendbc.car.psa.values import CarControllerParams, PSASafetyFlags
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common

LANE_KEEP_ASSIST = 0x3F2
REQ_DIAG_ARTIV = 0x6B6
HS2_DYN1_MDD_ETAT_2B6 = 0x2B6
HS2_DYN_MDD_ETAT_2F6 = 0x2F6


class SafetyLimits:
  # safety uses the ISO limits, openpilot steers within a lower lateral accel
  ANGLE_LIMITS = AngleSteeringLimitsVM(CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX)
  STEER_STEP = CarControllerParams.STEER_STEP


class TestPsaSafetyBase(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  DBC = "psa_aee2010_r3"
  SAFETY_MODEL = CarParams.SafetyModel.psa

  RELAY_MALFUNCTION_ADDRS = {0: (LANE_KEEP_ASSIST,)}
  FWD_BLACKLISTED_ADDRS = {2: [LANE_KEEP_ASSIST]}
  TX_MSGS = [[LANE_KEEP_ASSIST, 0]]

  MAIN_BUS = 0
  ADAS_BUS = 1
  CAM_BUS = 2
  EPS_BUS = MAIN_BUS

  STEER_ANGLE_MAX = 390
  DEG_TO_CAN = 10
  LATERAL_FREQUENCY = 100  # Hz

  def setUp(self):
    super().setUp()
    self.VM = VehicleModel(get_safety_CP())

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, SafetyLimits)

  cnt_angle_cmd = 0

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    values = {"SET_ANGLE": angle, "TORQUE_FACTOR": 100 if enabled else 0}
    return self.packer.make_can_msg_safety("LANE_KEEP_ASSIST", self.MAIN_BUS, values)

  def _angle_meas_msg(self, angle: float):
    values = {"ANGLE": angle}
    return self.packer.make_can_msg_safety("STEERING_ALT", self.EPS_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"RVV_ACC_ACTIVATION_REQ": enable}
    return self.packer.make_can_msg_safety("HS2_DAT_MDD_CMD_452", self.ADAS_BUS, values)

  def _speed_msg(self, speed):
    values = {"VITESSE_VEHICULE_ROUES": speed * 3.6}
    return self.packer.make_can_msg_safety("HS2_DYN_ABR_38D", self.MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"P013_MainBrake": brake}
    return self.packer.make_can_msg_safety("Dat_BSI", self.CAM_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": int(gas * 100)}
    return self.packer.make_can_msg_safety("DRIVER", self.CAM_BUS, values)

  # PSA uses get_max_angle_delta_vm and get_max_angle_vm for real lateral accel and jerk limits
  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  @staticmethod
  def _round_speed(speed):
    # match the 0.01 km/h speed on CAN
    return common.round_speed(common.away_round(speed * 3.6 / 0.01) * 0.01 / 3.6)

  @staticmethod
  def _max_can(limit):
    # safety allows one CAN unit of tolerance
    return int(limit * 10 + 1)

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = self._round_speed(max(speed, 1))
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        max_can = min(self._max_can(get_max_angle_vm(speed, self.VM, SafetyLimits)), self.STEER_ANGLE_MAX * self.DEG_TO_CAN)
        self.safety.set_desired_angle_last(max_can * sign)
        self.assertTrue(self._tx(self._angle_cmd_msg(max_can * sign / self.DEG_TO_CAN, True)))

        # one unit above the limit, at low speed the limit is above the max angle
        should_tx = max_can >= self.STEER_ANGLE_MAX * self.DEG_TO_CAN
        self.safety.set_desired_angle_last((max_can + 1) * sign)
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg((max_can + 1) * sign / self.DEG_TO_CAN, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 100):
      speed = self._round_speed(max(speed, 1))
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        max_delta_can = self._max_can(get_max_angle_delta_vm(speed, self.VM, SafetyLimits))
        max_delta_can = min(max_delta_can, self.STEER_ANGLE_MAX * self.DEG_TO_CAN)
        max_delta = max_delta_can * sign / self.DEG_TO_CAN

        # within limits up, hold, down
        self.assertTrue(self._tx(self._angle_cmd_msg(max_delta, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_delta, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # too fast up and down
        too_fast = min(self.STEER_ANGLE_MAX * self.DEG_TO_CAN, max_delta_can + 1)
        if too_fast > max_delta_can:
          self.assertFalse(self._tx(self._angle_cmd_msg(too_fast * sign / self.DEG_TO_CAN, True)))
          self.safety.set_desired_angle_last(too_fast * sign)
          self.assertTrue(self._tx(self._angle_cmd_msg(too_fast * sign / self.DEG_TO_CAN, True)))
          self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

  def test_openpilot_within_safety(self):
    car = CarControllerParams.ANGLE_LIMITS
    self.assertLessEqual(car.MAX_LATERAL_ACCEL, SafetyLimits.ANGLE_LIMITS.MAX_LATERAL_ACCEL)
    self.assertLessEqual(car.MAX_LATERAL_JERK, SafetyLimits.ANGLE_LIMITS.MAX_LATERAL_JERK)

  def test_rx_hook(self):
    # speed
    for _ in range(10):
      self.assertTrue(self._rx(self._speed_msg(0)))
    msg = self._speed_msg(0)
    # invalidate checksum
    msg[0].data[5] = 0x00
    self.assertFalse(self._rx(msg))

    # cruise
    for _ in range(10):
      self.assertTrue(self._rx(self._pcm_status_msg(0)))
    msg = self._pcm_status_msg(0)
    # invalidate checksum
    msg[0].data[5] = 0x00
    self.assertFalse(self._rx(msg))
    msg = self._pcm_status_msg(0)
    # write to unused payload byte
    msg[0].data[6] = 0xAB
    self.assertTrue(self._rx(msg))

  def _ign_msg(self, bus, counter, ign):
    return common.make_msg(bus, 0x432, dat=bytes([0, counter, 0, 0, 0, 0, ign, 0]))

  def test_ignition_on(self):
    for bus in (0, 2):
      for i in range(16):
        self.safety.init_tests()
        self.safety.ignition_can_hook(self._ign_msg(bus, i, 1))
        self.assertFalse(self.safety.get_ignition_can())
        self.safety.ignition_can_hook(self._ign_msg(bus, (i + 1) % 16, 1))
        self.assertTrue(self.safety.get_ignition_can())

  def test_ignition_off(self):
    for bus in (0, 2):
      self.safety.ignition_can_hook(self._ign_msg(bus, 0, 1))
      self.safety.ignition_can_hook(self._ign_msg(bus, 1, 1))
      self.assertTrue(self.safety.get_ignition_can())
      self.safety.ignition_can_hook(self._ign_msg(bus, 2, 0))
      self.assertFalse(self.safety.get_ignition_can())


class TestPsaStockSafety(TestPsaSafetyBase):
  pass


class TestPsaEpsCamBusSafety(TestPsaSafetyBase):
  SAFETY_PARAM = PSASafetyFlags.EPS_CAM_BUS.value
  EPS_BUS = TestPsaSafetyBase.CAM_BUS

  def test_eps_bus(self):
    # the angle is only read from the EPS bus, the relay shows it on both while open
    for bus, should_update in ((self.CAM_BUS, True), (self.MAIN_BUS, False)):
      self._reset_angle_measurement(0)
      self._rx(self.packer.make_can_msg_safety("STEERING_ALT", bus, {"ANGLE": 100}))
      self.assertEqual(should_update, self.safety.get_angle_meas_max() == 1000)


class TestPsaLongitudinalSafety(TestPsaSafetyBase, common.LongitudinalAccelSafetyTest):
  SAFETY_PARAM = PSASafetyFlags.LONG_CONTROL.value
  TX_MSGS = [[LANE_KEEP_ASSIST, 0], [REQ_DIAG_ARTIV, 1], [HS2_DYN1_MDD_ETAT_2B6, 1], [HS2_DYN_MDD_ETAT_2F6, 1]]

  INACTIVE_ACCEL = 2.05
  MAX_TORQUE = 1500
  MIN_TORQUE = -400
  INACTIVE_TORQUE = -4000

  def _acc_msg(self, decel=INACTIVE_ACCEL, torque=INACTIVE_TORQUE, potential_torque=INACTIVE_TORQUE, decel_req=False):
    values = {
      "MDD_DESIRED_DECELERATION": decel,
      "GMP_WHEEL_TORQUE": torque,
      "GMP_POTENTIAL_WHEEL_TORQUE": potential_torque,
      "MDD_DECEL_CONTROL_REQ": decel_req,
    }
    return self.packer.make_can_msg_safety("HS2_DYN1_MDD_ETAT_2B6", self.ADAS_BUS, values)

  def _accel_msg(self, accel: float):
    return self._acc_msg(decel=accel)

  def test_torque_limits(self):
    for torque in [*range(-1000, 2000, 50), self.INACTIVE_TORQUE]:
      for controls_allowed in (True, False):
        self.safety.set_controls_allowed(controls_allowed)
        should_tx = (controls_allowed and self.MIN_TORQUE <= torque <= self.MAX_TORQUE) or torque == self.INACTIVE_TORQUE
        self.assertEqual(should_tx, self._tx(self._acc_msg(torque=torque)))
        self.assertEqual(should_tx, self._tx(self._acc_msg(potential_torque=torque)))

  def test_decel_request(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      self.assertEqual(controls_allowed, self._tx(self._acc_msg(decel=-1.0, decel_req=True)))

  def test_standstill_hold(self):
    for speed in (0.0, 0.3, 0.9, 1.1, 10.0):
      for _ in range(6):
        self._rx(self._speed_msg(speed))
      for controls_allowed in (True, False):
        self.safety.set_controls_allowed(controls_allowed)
        should_tx = controls_allowed and speed < 1.0
        self.assertEqual(should_tx, self._tx(self._acc_msg(decel=-10.65, decel_req=True)))

  def test_radar_diagnostics(self):
    for should_tx, dat in ((True, b"\x02\x10\x02\x80\x00\x00\x00\x00"),
                           (True, b"\x02\x3E\x00\x00\x00\x00\x00\x00"),
                           (False, b"\x02\x10\x03\x00\x00\x00\x00\x00"),
                           (False, b"\x02\x11\x01\x00\x00\x00\x00\x00")):
      self.assertEqual(should_tx, self._tx(libsafety_py.make_CANPacket(REQ_DIAG_ARTIV, self.ADAS_BUS, dat)))

  def test_no_actuation_on_gas(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._user_gas_msg(1))
    self.assertFalse(self._tx(self._acc_msg(torque=500)))
    self.assertFalse(self._tx(self._acc_msg(decel=-1.0, decel_req=True)))
    self.assertTrue(self._tx(self._acc_msg()))


if __name__ == "__main__":
  unittest.main()
