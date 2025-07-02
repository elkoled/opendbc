from opendbc.car import apply_std_steer_angle_limits, Bus
from opendbc.can.packer import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering
from opendbc.car.psa.values import CarControllerParams
import numpy as np

TORQUE_DEADZONE = 0.5      # Ignore tiny torques (hands-off)
TORQUE_GAIN = 4.0          # How much driver torque affects steering angle
TORQUE_CLIP = 10.0         # Safety clamp

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.cam])
    self.apply_angle_last = 0

  def blend_angle(self, apply_angle, driver_torque):
    if abs(driver_torque) < TORQUE_DEADZONE:
      return apply_angle
    # Adjust angle in direction of driver torque
    delta = np.clip(driver_torque, -TORQUE_CLIP, TORQUE_CLIP) * TORQUE_GAIN
    return apply_angle + delta

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # lateral control
    if CC.latActive:
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                   CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)
      # Blend driver torque
      apply_angle = self.blend_angle(apply_angle, CS.out.steeringTorque)
    else:
      apply_angle = 0

    can_sends.append(create_lka_steering(self.packer, self.frame // 5, CC.latActive, apply_angle))

    self.apply_angle_last = apply_angle

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends
