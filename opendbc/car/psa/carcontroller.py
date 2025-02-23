import numpy as np
from opendbc.car import structs, apply_driver_steer_torque_limits, Bus
from opendbc.can.packer import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa import psacan
from opendbc.car.psa.values import CarControllerParams

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    self.CP = CP
    self.packer = CANPacker(dbc_names[Bus.cam])
    self.frame = 0
    self.apply_steer_last = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    ### lateral control ###
    if CC.latActive:
      apply_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
      new_steer = int(round(apply_steer))
      apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, CarControllerParams)
    else:
      apply_steer = 0

    angle = CS.out.steeringAngleDeg
    can_sends.append(psacan.create_lka_msg_cc(self.packer, self.CP, self.frame, CC.latActive, apply_steer, angle))

    self.apply_steer_last = apply_steer

    ### cruise buttons ###
    # TODO: find cruise buttons msg
    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / CarControllerParams.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last
    self.frame += 1
    return new_actuators, can_sends
