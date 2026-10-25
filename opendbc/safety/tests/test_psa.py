#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py

LANE_KEEP_ASSIST = 0x3F2


class TestPsaSafetyBase(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  DBC = "psa_aee2010_r3"
  SAFETY_MODEL = CarParams.SafetyModel.psa

  RELAY_MALFUNCTION_ADDRS = {0: (LANE_KEEP_ASSIST,)}
  FWD_BLACKLISTED_ADDRS = {2: [LANE_KEEP_ASSIST]}
  TX_MSGS = [[1010, 0]]

  MAIN_BUS = 0
  ADAS_BUS = 1
  CAM_BUS = 2

  STEER_ANGLE_MAX = 390
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 25.]
  ANGLE_RATE_UP = [2.5, 1.5, .2]
  ANGLE_RATE_DOWN = [5., 2., .3]

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"SET_ANGLE": angle, "TORQUE_FACTOR": 100 if enabled else 0}
    return self.packer.make_can_msg_safety("LANE_KEEP_ASSIST", self.MAIN_BUS, values)

  def _angle_meas_msg(self, angle: float):
    values = {"ANGLE": angle}
    return self.packer.make_can_msg_safety("STEERING_ALT", self.MAIN_BUS, values)

  def _driver_torque_msg(self, torque: float):
    values = {"DRIVER_TORQUE": torque}
    return self.packer.make_can_msg_safety("STEERING", self.MAIN_BUS, values)

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
    values = {"P002_Com_rAPP": int(gas * 100)}
    return self.packer.make_can_msg_safety("Dyn_CMM", self.MAIN_BUS, values)

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

    # steering angle, XOR checksum in high nibble of byte 4
    for _ in range(10):
      self.assertTrue(self._rx(self._angle_meas_msg(12.3)))
    msg = self._angle_meas_msg(12.3)
    msg[0].data[4] ^= 0x10
    self.assertFalse(self._rx(msg))

    # driver torque, checksum in high nibble of byte 0
    for _ in range(10):
      self.assertTrue(self._rx(self._driver_torque_msg(1.5)))
    msg = self._driver_torque_msg(1.5)
    msg[0].data[0] ^= 0x10
    self.assertFalse(self._rx(msg))

  def test_rx_hook_recorded_frames(self):
    # frames recorded on the PSA_PEUGEOT_208 test route, checksum and counter must be accepted
    for addr, frames in ((0x305, ("00070087b30030", "00070007440030", "00070107450130", "00070007660030")),
                         (0x2F5, ("03000009800007", "14ff0009800007", "2500000980ffe7", "0600000980fff7"))):
      for frame in frames:
        self.assertTrue(self._rx(libsafety_py.make_CANPacket(addr, self.MAIN_BUS, bytes.fromhex(frame))))

  def test_power_cut_request(self):
    # DMD_DELEST_DA removes EPS assist at low speed on Stop&Start cars, never allowed
    for power_cut in (0, 1):
      values = {"SET_ANGLE": 0, "TORQUE_FACTOR": 0, "DMD_DELEST_DA": power_cut}
      msg = self.packer.make_can_msg_safety("LANE_KEEP_ASSIST", self.MAIN_BUS, values)
      self.assertEqual(not power_cut, self._tx(msg))


class TestPsaStockSafety(TestPsaSafetyBase):
  pass


if __name__ == "__main__":
    unittest.main()
