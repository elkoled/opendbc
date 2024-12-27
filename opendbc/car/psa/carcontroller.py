from opendbc.car import structs, apply_std_steer_angle_limits, Bus
from opendbc.can.packer import CANPacker
from opendbc.car.common.numpy_fast import clip
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa import psacan
from opendbc.car.psa.values import CarControllerParams

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    self.CP = CP
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.frame = 0
    self.ramp_value = 0
    self.apply_angle_last = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # Ramp up/down logic for torque factor
    # TODO: these are stock values, test if faster is possible
    if CC.latActive:
      self.ramp_value = min(self.ramp_value + 3.5, 100)
    else:
      self.ramp_value = max(self.ramp_value - 1, 0)

    ### lateral control ###
    if CC.latActive:
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CarControllerParams)
      # EPS accepts any angle from at least -360 - 360° (STEER_MAX)
    else:
      apply_angle = CS.out.steeringAngleDeg

    apply_angle = clip(apply_angle, -CarControllerParams.STEER_MAX, CarControllerParams.STEER_MAX)

    # TODO: recuperation and driving mode is inactive when sending own messages
    can_sends.append(psacan.create_lka_msg(self.packer, self.CP, apply_angle, self.frame, CC.latActive, self.ramp_value))

    self.apply_angle_last = apply_angle

    ### cruise buttons ###
    # TODO: find cruise buttons msg
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends
