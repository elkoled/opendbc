from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs, make_tester_present_msg
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering, create_resume_acc, create_disable_radar, create_HS2_DYN1_MDD_ETAT_2B6, create_HS2_DYN_MDD_ETAT_2F6
from opendbc.car.psa.values import CarControllerParams

LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_angle_last = 0
    self.radar_disabled = 0
    self.status = 2

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    # longitudinal
    starting = actuators.longControlState == LongCtrlState.starting and CS.out.vEgo <= self.CP.vEgoStarting
    # stopping = actuators.longControlState == LongCtrlState.stopping

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

    # # emulate resume button every 4 seconds to prevent autohold timeout
    # if CC.latActive and CS.out.standstill and CC.hudControl.leadVisible:
    #   # map: {frame:status} - 0, 0, 1, 1
    #   status = {0: 0, 5: 0, 10: 1, 15: 1}.get(self.frame % 400)
    #   if status is not None:
    #     msg = CS.hs2_dat_mdd_cmd_452
    #     counter = (msg['COUNTER'] + 1) % 16
    #     can_sends.append(create_resume_acc(self.packer, counter, status, msg))

    self.apply_angle_last = apply_angle

    # longitudinal control
    # TUNING
    brake_accel = -0.5 # below this accel, go into brake mode
    torque_raw = actuators.accel * 10 * 70 # accel in m/s^2 to torque in Nm * 10 for CAN
    torque = max(-300, min(torque_raw, 2000)) # apply torque CAN Nm limits
    braking = actuators.accel<brake_accel and not CS.out.gasPressed

    # # twitchy on gas/accel transition but ok car following and braking
    # torque = actuators.accel * 1000
    # braking = torque < -300 and not CS.out.gasPressed
    if self.CP.openpilotLongitudinalControl:
      # disable radar ECU by setting to programming mode
      # if self.frame > 1000:
      if self.radar_disabled == 0:
        can_sends.append(create_disable_radar())
        self.radar_disabled = 1

      # keep radar ECU disabled by sending tester present
      if self.frame % 100 == 0 and self.frame>0: # TODO check if disable_radar is sent 100 frames before
        can_sends.append(make_tester_present_msg(0x6b6, 1, suppress_response=False))

      # TODO: tune torque multiplier
      # TODO: tune braking threshold
      # TODO: check if disengage on accelerator is already in CC.longActive
      # Highest torque seen without gas input: ~1000
      # Lowest torque seen without break mode: -560 (but only when transitioning from brake to accel mode, else -248)
      # Lowest brake mode accel seen: -4.85m/s²

      if self.frame % 2 == 0: # 50 Hz
        can_sends.append(create_HS2_DYN1_MDD_ETAT_2B6(self.packer, self.frame // 2, actuators.accel, CC.latActive, CS.out.gasPressed, braking, CS.out.brakePressed, torque))
        can_sends.append(create_HS2_DYN_MDD_ETAT_2F6(self.packer))

      # if self.frame % 10 == 0: # 10 Hz
      #   can_sends.append(create_HS2_DAT_ARTIV_V2_4F6(self.packer, CC.longActive))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1
    return new_actuators, can_sends
