from opendbc.car import apply_std_steer_angle_limits, Bus
from opendbc.can.packer import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa import psacan
from opendbc.car.psa.values import CarControllerParams

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    self.CP = CP
    self.packer = CANPacker(dbc_names[Bus.cam])
    self.frame = 0
    self.apply_angle_last = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    ### lateral control ###
    if CC.latActive:
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                   CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)
    else:
      apply_angle = 0

    can_sends.append(psacan.create_lka_msg(self.packer, self.CP, self.frame, CC.latActive, apply_angle))

    self.apply_angle_last = apply_angle

    ### cruise buttons ###
    # TODO: find cruise buttons msg
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends
