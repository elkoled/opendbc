from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering, create_resume_acc, create_drive_away_request
from opendbc.car.psa.values import CarControllerParams

LongCtrlState = structs.CarControl.Actuators.LongControlState

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_angle_last = 0
    self.status = 2
    self.resume = 0
    self.burst_left = 0
    self.last_acc_counter = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    # longitudinal
    starting = actuators.longControlState == LongCtrlState.starting and CS.out.vEgo <= self.CP.vEgoStarting
    stopping = actuators.longControlState == LongCtrlState.stopping

    # lateral control
    apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                                 CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)

    # EPS disengages on steering override, activation sequence 2->3->4 to re-engage
    # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
    if not CC.latActive:
      self.status = 2
    elif not CS.eps_active and not CS.out.steeringPressed:
      self.status = 2 if self.status == 4 else self.status + 1
    else:
      self.status = 4

    can_sends.append(create_lka_steering(self.packer, CC.latActive, apply_angle, self.status, 1 if starting else 0))

    # TODO: only trigger when lead car is present
    if starting and self.frame % 5 == 0:
      msg = CS.hs2_dat_mdd_cmd_452
      target_val = 1 if (self.frame % 20) < 10 else 0
      future_counter = (msg['COUNTER'] + 1) % 16
      can_sends.append(create_resume_acc(self.packer, future_counter, target_val, msg))

    self.apply_angle_last = apply_angle

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1
    return new_actuators, can_sends
