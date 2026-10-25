import unittest

from opendbc.can.dbc import DBC
from opendbc.car.psa.psacan import psa_checksum
from opendbc.testing import parameterized

# frames recorded on the PSA_PEUGEOT_208 test route 6a7075a4fdd765ee/0000004e--1f612006dd
STEERING_ALT_FRAMES = ['004d0107780130', 'fac310070c1030', '00670007600030', 'ff080407f40430',
                       'ffca0087180030', '000f00074c0030', '00330087f00030', 'ff1f0007d40030']
STEERING_FRAMES = ['07010009c60017', '4bf1000435fbf7', '1f000009e0fff7', '53f400087ffee7',
                   'a7020009430017', '0b000009880007', '3f000009ac0037', 'a301000898fff7']


class TestPSAChecksum(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.dbc = DBC("psa_aee2010_r3")

  def _check(self, msg_name: str, frame: str):
    msg = self.dbc.name_to_msg[msg_name]
    sig = msg.sigs["CHECKSUM"]
    dat = bytearray.fromhex(frame)
    expected = (dat[sig.start_bit // 8] >> (4 if sig.start_bit % 8 >= 4 else 0)) & 0xF
    self.assertEqual(psa_checksum(msg.address, sig, dat), expected)

  @parameterized("frame", STEERING_ALT_FRAMES)
  def test_steering_alt_xor_checksum(self, frame):
    self._check("STEERING_ALT", frame)

  @parameterized("frame", STEERING_FRAMES)
  def test_steering_checksum(self, frame):
    self._check("STEERING", frame)
