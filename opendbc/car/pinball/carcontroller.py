from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.pinball import pinballcan
from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])

  def update(self, CC, CS, now_nanos):
    left = 0
    right = 0
    start = 0

    if CC.enabled:
      left = int(CC.actuators.gas > 0.5)
      right = int(CC.actuators.brake > 0.5)
      start = int(CC.actuators.accel > 0.5)

    can_sends = [pinballcan.create_solenoid_cmd(self.packer, left, right, start, self.frame)]

    new_actuators = CC.actuators.as_builder()
    self.frame += 1
    return new_actuators, can_sends
