import math
import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, DT_CTRL, Bus, structs, make_tester_present_msg
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.lateral import apply_steer_angle_limits_vm, get_max_angle_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import AccMode, create_disable_radar, create_HS2_DYN1_MDD_ETAT_2B6, create_HS2_DYN_MDD_ETAT_2F6, create_lka_steering
from opendbc.car.psa.values import CarControllerParams, EpsState, PSAFlags
from opendbc.car.vehicle_model import VehicleModel

LongCtrlState = structs.CarControl.Actuators.LongControlState

# pandad blocks the radar messages until it leaves ELM327 mode
RADAR_DISABLE_FRAME = 100
# the stock radar holds the car from HOLD_SPEED and launches until LAUNCH_SPEED
HOLD_SPEED = 0.3  # m/s
LAUNCH_SPEED = 1.3  # m/s
# the lift-off regen of an EV motor clears ~0.2 s into a torque request
EV_LIFT_OFF_FRAMES = 25  # 0.5 s of radar frames after engaging or a gas override


def get_safety_CP():
  # the 208 has the shortest wheelbase, so the lowest angle limits for every platform
  from opendbc.car.psa.interface import CarInterface
  return CarInterface.get_non_essential_params("PSA_PEUGEOT_208")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.VM = VehicleModel(get_safety_CP())
    self.apply_angle_last = 0
    self.lat_active = False
    self.status = 2
    self.acc_mode = AccMode.TORQUE
    self.long_frames = 0
    # the body pitches with the car's own accel, unfiltered that feeds back into the grade compensation
    self.pitch = FirstOrderFilter(0., 1.0, DT_CTRL)
    self.torque_last = None

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])

    # lateral control
    if self.frame % CarControllerParams.STEER_STEP == 0:
      # don't take over a curve steered beyond the lateral accel limit, safety would block the unwind
      max_angle = get_max_angle_vm(max(CS.out.vEgoRaw, 1), self.VM, CarControllerParams)
      self.lat_active = CC.latActive and (self.lat_active or abs(CS.out.steeringAngleDeg) < max_angle)

      # aim at the wheel until the EPS takes over, so it starts from where the wheel is
      target = actuators.steeringAngleDeg if CS.eps_active else CS.out.steeringAngleDeg
      apply_angle = apply_steer_angle_limits_vm(target, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                self.lat_active, CarControllerParams, self.VM)

      # the EPS disengages on driver override, re-engage with READY -> AUTHORIZED -> ACTIVE
      # the 508 EPS needs each step held, and ACTIVE held while it reports authorized
      # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
      if not self.lat_active:
        self.status = 2
      elif CS.eps_state in (EpsState.AUTHORIZED, EpsState.ACTIVE):
        self.status = 4
      elif self.frame % CarControllerParams.STATUS_STEP == 0:
        self.status = 2 if self.status == 4 else self.status + 1

      can_sends.append(create_lka_steering(self.packer, self.lat_active, apply_angle, self.status))

      self.apply_angle_last = apply_angle

    # longitudinal control
    if self.CP.openpilotLongitudinalControl and self.frame >= RADAR_DISABLE_FRAME:
      # keep the radar silent, two senders of the radar messages fault the ESP
      if self.frame % 100 == 0:
        can_sends.append(create_disable_radar() if CS.radar_alive else make_tester_present_msg(0x6b6, 1))

      if not CS.radar_alive and self.frame % CarControllerParams.ACC_STEP == 0:
        can_sends.extend(self.update_long(CC, CS))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends

  def update_long(self, CC, CS):
    accel = float(np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    # the CMM takes a wheel torque, compensate it for the grade, the ESP brakes to the measured decel
    accel_pitch = accel + math.sin(self.pitch.x) * ACCELERATION_DUE_TO_GRAVITY
    ev = bool(self.CP.flags & PSAFlags.EV)
    torque_v = CarControllerParams.TORQUE_LOOKUP_V_EV if ev else CarControllerParams.TORQUE_LOOKUP_V
    torque = float(np.clip(np.interp(accel_pitch, CarControllerParams.TORQUE_LOOKUP_BP, torque_v),
                           CarControllerParams.TORQUE_MIN, CarControllerParams.TORQUE_MAX))

    stopping = CC.actuators.longControlState == LongCtrlState.stopping
    if not CC.longActive:
      self.acc_mode = AccMode.TORQUE
    elif stopping and CS.out.vEgo < HOLD_SPEED:
      self.acc_mode = AccMode.HOLD
    elif self.acc_mode == AccMode.HOLD:
      self.acc_mode = AccMode.LAUNCH
    elif accel_pitch < (CarControllerParams.EV_BRAKE_ON_ACCEL if ev else CarControllerParams.BRAKE_ON_ACCEL):
      # also leaves the launch, the lead can stop again before we are rolling
      self.acc_mode = AccMode.BRAKE
    elif self.acc_mode == AccMode.LAUNCH:
      if CS.out.vEgo >= LAUNCH_SPEED:
        self.acc_mode = AccMode.TORQUE
    elif accel_pitch > (CarControllerParams.EV_BRAKE_OFF_ACCEL if ev else CarControllerParams.BRAKE_OFF_ACCEL):
      # like the stock radar the ESP keeps control while pulling away at walking pace, torque alone faults it
      self.acc_mode = AccMode.LAUNCH if CS.out.vEgo < LAUNCH_SPEED else AccMode.TORQUE

    self.long_frames = self.long_frames + 1 if CC.longActive else 0
    if ev and self.acc_mode == AccMode.TORQUE:
      if self.torque_last is not None:
        torque = float(np.clip(torque, self.torque_last - CarControllerParams.EV_TORQUE_RATE, self.torque_last + CarControllerParams.EV_TORQUE_RATE))
      # an EV motor in its lift-off regen right after the driver's foot stopped taking our torque for the drive after a negative request
      if self.long_frames < EV_LIFT_OFF_FRAMES:
        torque = max(torque, CarControllerParams.EV_LIFT_OFF_TORQUE_MIN)
    self.torque_last = torque if CC.longActive and self.acc_mode == AccMode.TORQUE else None

    hud = CC.hudControl
    return [
      create_HS2_DYN1_MDD_ETAT_2B6(self.packer, CC.longActive, CC.cruiseControl.override, CS.out.brakePressed or CC.cruiseControl.cancel,
                                   self.acc_mode, accel, torque),
      create_HS2_DYN_MDD_ETAT_2F6(self.packer, CC.longActive and self.acc_mode != AccMode.TORQUE, hud.leadVisible, hud.leadDistanceBars),
    ]
