from opendbc.car import apply_std_steer_angle_limits, Bus
from opendbc.can.packer import CANPacker
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering
from opendbc.car.psa.values import CarControllerParams
import numpy as np

# Torque blending parameters
TORQUE_TO_ANGLE_MULTIPLIER_OUTER = 0.04  # Higher = easier to influence when manually steering more than OP
TORQUE_TO_ANGLE_MULTIPLIER_INNER = 0.08  # Higher = easier to influence when manually steering less than OP
TORQUE_TO_ANGLE_DEADZONE = 5  # 0.5 Nm
TORQUE_TO_ANGLE_CLIP = 50  # 5 Nm
CONTINUED_OVERRIDE_ANGLE = 10  # The angle difference between OP and user to continue overriding steering

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.cam])
    self.apply_angle_last = 0
    self.steering_override = False

  def torque_blended_angle(self, apply_angle, driver_torque):
    deadzone = TORQUE_TO_ANGLE_DEADZONE
    if abs(driver_torque) < deadzone:
      return apply_angle

    limit = TORQUE_TO_ANGLE_CLIP
    if apply_angle * driver_torque >= 0:
      # user override in the same direction
      strength = TORQUE_TO_ANGLE_MULTIPLIER_OUTER
    else:
      # user override in the opposite direction
      strength = TORQUE_TO_ANGLE_MULTIPLIER_INNER

    torque = driver_torque - deadzone if driver_torque > 0 else driver_torque + deadzone
    return apply_angle + np.clip(torque, -limit, limit) * strength

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # lateral control
    if CC.latActive:
      apply_angle = actuators.steeringAngleDeg

      if not self.steering_override:
        apply_angle = self.torque_blended_angle(apply_angle, CS.out.steeringTorque)

      apply_angle = apply_std_steer_angle_limits(apply_angle, self.apply_angle_last, CS.out.vEgoRaw,
                                                CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)

      self.steering_override = (CS.out.steeringPressed and
                              abs(CS.out.steeringAngleDeg - apply_angle) > CONTINUED_OVERRIDE_ANGLE and
                              not CS.out.standstill)

    else:
      apply_angle = 0
      self.steering_override = False

    can_sends.append(create_lka_steering(self.packer, self.frame // 5, CC.latActive, apply_angle))

    self.apply_angle_last = float(apply_angle)

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends