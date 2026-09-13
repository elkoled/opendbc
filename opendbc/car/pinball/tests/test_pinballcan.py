from types import SimpleNamespace

from opendbc.car.pinball.carcontroller import CarController
from opendbc.car.pinball.pinballcan import create_solenoid_cmd, crc8


def test_all_held_button_states():
  for state in range(8):
    msg = create_solenoid_cmd(None, state & 1, state & 2, state & 4, sequence=42)
    assert msg.address == 0x200
    assert msg.src == 0
    assert msg.dat[3] == 42
    assert msg.dat[4] == state
    assert msg.dat[5] == state ^ 0xff
    assert msg.dat[7] == crc8(msg.dat[:7])


def test_controller_sends_direct_states_while_disengaged():
  controller = CarController.__new__(CarController)
  controller.packer = None
  controller.frame = 0
  actuators = SimpleNamespace(gas=1.0, brake=0.0, accel=1.0, as_builder=lambda: None)
  _, sends = controller.update(SimpleNamespace(enabled=False, actuators=actuators), None, 0)
  assert sends[0].dat[4] == 0b101
