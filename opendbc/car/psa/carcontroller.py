from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs, make_tester_present_msg
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering, create_resume_acc, create_disable_radar, create_HS2_DYN1_MDD_ETAT_2B6, create_HS2_DYN_MDD_ETAT_2F6, create_HS2_DAT_ARTIV_V2_4F6, create_HS2_SUPV_ARTIV_796
from opendbc.car.psa.values import CarControllerParams
from numpy import interp

LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_angle_last = 0
    self.radar_disabled = 0
    self.status = 2

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    # longitudinal
    # starting = actuators.longControlState == LongCtrlState.starting and CS.out.vEgo <= self.CP.vEgoStarting
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

    # OP long
    # TUNING
    # >=-0.5: Engine brakes only
    # <-0.5: Add friction brakes
    brake_accel = -0.5

    # torque lookup
    ACCEL_LOOKUP = [-1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0]
    TORQUE_LOOKUP = [-400, -150, 150, 350, 550, 800, 1000]

    # calculate Torque
    torque_nm = interp(actuators.accel, ACCEL_LOOKUP, TORQUE_LOOKUP)
    torque = max(-400, min(torque_nm, 1000))

    # engine/friction brake transition
    braking = actuators.accel < brake_accel and not CS.out.gasPressed

    if self.CP.openpilotLongitudinalControl:
      # disable radar ECU by setting to programming mode
      if self.radar_disabled == 0:
        can_sends.append(create_disable_radar())
        self.radar_disabled = 1

      # keep radar ECU disabled by sending tester present
      if self.frame % 100 == 0 and self.frame>0: # TODO check if disable_radar is sent 100 frames before
        can_sends.append(make_tester_present_msg(0x6b6, 1, suppress_response=False))

      # Highest torque seen without gas input: ~1000
      # Lowest torque seen without break mode: -560 (but only when transitioning from brake to accel mode, else -248)
      # Lowest brake mode accel seen: -4.85m/s²
      long_enabled = CC.longActive and CS.drive

      if self.frame % 2 == 0:
        can_sends.append(create_HS2_DYN1_MDD_ETAT_2B6(self.packer, self.frame // 2, actuators.accel, CS.out.cruiseState.enabled, CS.out.gasPressed, braking, CS.out.brakePressed, CS.out.standstill, CS.drive, torque))
        can_sends.append(create_HS2_DYN_MDD_ETAT_2F6(self.packer, braking, CC.hudControl.leadVisible))

      if self.frame % 10 == 0:
        can_sends.append(create_HS2_DAT_ARTIV_V2_4F6(self.packer, CS.out.cruiseState.enabled))

      if self.frame % 100 == 0:
        can_sends.append(create_HS2_SUPV_ARTIV_796(self.packer))

    # stock long
    # emulate resume button every 3 seconds to prevent autohold timeout
    elif CC.latActive and CS.out.standstill and CC.hudControl.leadVisible:
      # map: {frame:status} - 0, 1
      status = {0: 0, 5: 0}.get(self.frame % 300)
      if status is not None:
        msg = CS.hs2_dat_mdd_cmd_452
        counter = (msg['COUNTER'] + 1) % 16
        can_sends.append(create_resume_acc(self.packer, counter, status, msg))

    can_sends.append(create_lka_steering(self.packer, CC.latActive, apply_angle, self.status))
    self.apply_angle_last = apply_angle

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1
    return new_actuators, can_sends
