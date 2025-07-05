from opendbc.car import Bus, AngleSteeringLimits, ACCELERATION_DUE_TO_GRAVITY, DT_CTRL, rate_limit
from opendbc.can.packer import CANPacker
from opendbc.car.interfaces import CarControllerBase, ISO_LATERAL_ACCEL
from opendbc.car.psa.psacan import create_lka_steering
from opendbc.car.psa.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel
import numpy as np
import math

# limit angle rate to both prevent a fault and for low speed comfort
MAX_ANGLE_RATE = 2  # deg/20ms frame

# Add extra tolerance for average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06
MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)
MAX_LATERAL_JERK = 3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)


def get_max_angle_delta(v_ego_raw, VM):
  max_curvature_rate_sec = MAX_LATERAL_JERK / (v_ego_raw ** 2)
  max_angle_rate_sec = math.degrees(VM.get_steer_from_curvature(max_curvature_rate_sec, v_ego_raw, 0))
  return max_angle_rate_sec * (DT_CTRL * CarControllerParams.STEER_STEP)


def get_max_angle(v_ego_raw, VM):
  max_curvature = MAX_LATERAL_ACCEL / (v_ego_raw ** 2)
  return math.degrees(VM.get_steer_from_curvature(max_curvature, v_ego_raw, 0))


def apply_psa_steer_angle_limits(apply_angle, apply_angle_last, v_ego_raw, steering_angle, lat_active, limits, VM):
  v_ego_raw = max(v_ego_raw, 1)
  max_angle_delta = get_max_angle_delta(v_ego_raw, VM)
  max_angle_delta = min(max_angle_delta, MAX_ANGLE_RATE)
  new_apply_angle = rate_limit(apply_angle, apply_angle_last, -max_angle_delta, max_angle_delta)
  max_angle = get_max_angle(v_ego_raw, VM)
  new_apply_angle = np.clip(new_apply_angle, -max_angle, max_angle)
  if not lat_active:
    new_apply_angle = steering_angle
  return float(np.clip(new_apply_angle, -limits.STEER_ANGLE_MAX, limits.STEER_ANGLE_MAX))


def get_safety_CP():
  # Use PSA platform for lateral limiting to match safety
  from opendbc.car.psa.interface import CarInterface
  return CarInterface.get_non_essential_params("PSA_PEUGEOT_208")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.cam])
    self.apply_angle_last = 0

    # Vehicle model used for lateral limiting
    self.VM = VehicleModel(get_safety_CP())

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # lateral control
    if CC.latActive:
      apply_angle = apply_psa_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                 CS.out.steeringAngleDeg, CC.latActive,
                                                 CarControllerParams.ANGLE_LIMITS, self.VM)
    else:
      apply_angle = 0

    can_sends.append(create_lka_steering(self.packer, self.frame // 5, CC.latActive, apply_angle))

    self.apply_angle_last = apply_angle

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends
