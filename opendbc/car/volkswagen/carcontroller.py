import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_steer_angle_limits_vm
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.volkswagen import mebcan, mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
AudibleAlert = structs.CarControl.HUDControl.AudibleAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# Stock VW Emergency Assist replication for ID.4 (measured rlogs f3/f4)
DM_PHASE2_INITIAL = -1.83  # m/s², PHASE2 entry (f3 peak)
DM_PHASE2_HOLD    = -1.0   # m/s², PHASE2 settled (both routes)
DM_PHASE3_ACCEL   = -2.0   # m/s², PHASE3 (exact in both)
DM_JERK_ACCEL     = -3.5   # m/s², ESC wake-up spike (~30% Brake_Pressure, ACCEL_MIN floor)
DM_JERK_ONSET     = 2.0    # s after red-alert rising edge
DM_JERK_DURATION  = 0.2    # s
DM_PHASE3_ONSET   = 5.0    # s after red-alert rising edge
DM_JERK_GRAD      = 30.0   # m/s³ comfort-jerk bypass while DM brake active


class HCAMitigation:
  """
  Manages HCA fault mitigations for VW/Audi EPS racks:
    * Reduces torque by 1 for a single frame after commanding the same torque value for too long
  """

  def __init__(self, CCP):
    self._max_same_torque_frames = CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP)
    self._same_torque_frames = 0

  def update(self, apply_torque, apply_torque_last):
    if apply_torque != 0 and apply_torque_last == apply_torque:
      self._same_torque_frames += 1
      if self._same_torque_frames > self._max_same_torque_frames:
        apply_torque -= (1, -1)[apply_torque < 0]
        self._same_torque_frames = 0
    else:
      self._same_torque_frames = 0

    return apply_torque


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    else:
      self.CCS = mqbcan

    self.apply_torque_last = 0
    self.gra_acc_counter_last = None
    self.hca_mitigation = HCAMitigation(self.CCP) if not (CP.flags & VolkswagenFlags.MEB) else None

    self.apply_angle_last = 0.0
    self.steer_power_last = 0
    self.accel_last = 0.0
    self.long_override_counter = 0
    self.long_disabled_counter = 0
    self.long_stopping_counter = 0
    self.klr_counter_last = None
    self.dm_red_start_frame: int | None = None
    self.VM = VehicleModel(CP) if CP.flags & VolkswagenFlags.MEB else None

  def update(self, CC, CS, now_nanos):
    if self.CP.flags & VolkswagenFlags.MEB:
      return self.update_meb(CC, CS, now_nanos)

    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      apply_torque = 0
      if CC.latActive:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)

      apply_torque = self.hca_mitigation.update(apply_torque, self.apply_torque_last)
      hca_enabled = apply_torque != 0
      self.apply_torque_last = apply_torque
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_torque, hca_enabled))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl:
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
        stopping = actuators.longControlState == LongCtrlState.stopping
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.longActive, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation))

      #if self.aeb_available:
      #  if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
      #  if self.frame % self.CCP.AEB_HUD_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      # FIXME: PQ may need to use the on-the-wire mph/kmh toggle to fix rounding errors
      # FIXME: Detect clusters with vEgoCluster offsets and apply an identical vCruiseCluster offset
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                       lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends

  def update_meb(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    override = CC.cruiseControl.override or CS.out.gasPressed
    acc_control = mebcan.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, override)

    # DM red alert (driverDistracted3 / driverUnresponsive3) — yellow stays UI-only.
    # Stages anchored at rising edge: PHASE2 init -> jerk -> PHASE2 hold -> PHASE3.
    dm_red = (CC.enabled and not override and
              hud_control.visualAlert == VisualAlert.steerRequired and
              hud_control.audibleAlert == AudibleAlert.warningImmediate)
    if dm_red and self.dm_red_start_frame is None:
      self.dm_red_start_frame = self.frame
    elif not dm_red:
      self.dm_red_start_frame = None
    dm_t = (self.frame - self.dm_red_start_frame) * DT_CTRL if self.dm_red_start_frame is not None else None
    dm_phase3 = dm_t is not None and dm_t >= DM_PHASE3_ONSET
    if dm_t is None:
      dm_brake = None
    elif dm_phase3:
      dm_brake = DM_PHASE3_ACCEL
    elif DM_JERK_ONSET <= dm_t < DM_JERK_ONSET + DM_JERK_DURATION:
      dm_brake = DM_JERK_ACCEL
    elif dm_t < DM_JERK_ONSET:
      dm_brake = DM_PHASE2_INITIAL
    else:
      dm_brake = DM_PHASE2_HOLD

    # **** Steering ********************************************************* #

    if self.frame % self.CCP.STEER_STEP == 0:
      v_ego = max(CS.out.vEgoRaw, 1)
      measured_angle = math.degrees(self.VM.get_steer_from_curvature(CS.measured_curvature, v_ego, 0))
      # Power ramp prevents EPS REJECTED/FAULT on engage/disengage; stay-alive on disengage
      # ramps power down to 0 before dropping HCA so the rack sees a continuous transition.
      if CC.latActive:
        hca_enabled = True
        target_curvature = actuators.curvature + (CS.measured_curvature - CC.currentCurvature)
        target_angle = math.degrees(self.VM.get_steer_from_curvature(target_curvature, v_ego, 0))
        apply_angle = apply_steer_angle_limits_vm(target_angle, self.apply_angle_last, v_ego, measured_angle,
                                                  CC.latActive, self.CCP, self.VM)
        min_power = max(self.steer_power_last - self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MIN)
        max_power = min(self.steer_power_last + self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MAX)
        target_power_driver = int(np.interp(abs(CS.out.steeringTorque),
                                            [self.CCP.STEER_DRIVER_ALLOWANCE, self.CCP.STEER_DRIVER_MAX],
                                            [self.CCP.STEERING_POWER_MAX, self.CCP.STEERING_POWER_MIN]))
        target_power = int(np.interp(CS.out.vEgo, [0., 0.5], [self.CCP.STEERING_POWER_MIN, target_power_driver]))
        steering_power = min(max(target_power, min_power), max_power)
      elif self.steer_power_last > 0:
        hca_enabled = True
        apply_angle = measured_angle
        steering_power = max(self.steer_power_last - self.CCP.STEERING_POWER_STEP, 0)
      else:
        hca_enabled = False
        apply_angle = measured_angle
        steering_power = 0

      apply_curvature = self.VM.calc_curvature(math.radians(apply_angle), v_ego, 0)
      self.apply_angle_last = apply_angle
      self.steer_power_last = steering_power
      can_sends.append(mebcan.create_steering_control(self.packer_pt, self.CAN.pt, apply_curvature, hca_enabled, steering_power))

    # Emergency Assist intervention
    if self.CP.flags & VolkswagenFlags.STOCK_KLR_PRESENT:
      # send capacitive steering wheel touched
      # probably EA is stock activated only for cars equipped with capacitive steering wheel
      # (also stock long does resume from stop as long as hands on is detected additionally to OP resume spam)
      klr_send_ready = CS.klr_stock_values["COUNTER"] != self.klr_counter_last
      if klr_send_ready:
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.cam, CC.latActive, CS.klr_stock_values))
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.pt, CC.latActive, CS.klr_stock_values))
      self.klr_counter_last = CS.klr_stock_values["COUNTER"]

    # **** Acceleration ***************************************************** #

    if self.CP.openpilotLongitudinalControl and self.frame % self.CCP.ACC_CONTROL_STEP == 0:
      # Hold stopping for up to 0.5s after longcontrol exits, until wheels actually stop, to keep EPB closed on inclines
      raw_stopping = actuators.longControlState == LongCtrlState.stopping
      if raw_stopping:
        self.long_stopping_counter = 25
      elif CS.out.vEgoRaw > 0.1:
        self.long_stopping_counter = 0
      else:
        self.long_stopping_counter = max(self.long_stopping_counter - 1, 0)
      stopping = raw_stopping or self.long_stopping_counter > 0
      starting = (actuators.longControlState == LongCtrlState.starting and
                  CS.out.vEgo <= self.CP.vEgoStarting and not stopping)
      accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.enabled else 0)
      if dm_brake is not None:
        accel = min(accel, dm_brake)
      ts = DT_CTRL * self.CCP.ACC_CONTROL_STEP
      jerk_max = DM_JERK_GRAD if dm_brake is not None else 2.0
      accel = float(np.clip(accel, self.accel_last - jerk_max * ts, self.accel_last + jerk_max * ts))
      self.accel_last = accel

      self.long_override_counter = min(self.long_override_counter + 1, 5) if override else 0
      override_begin = override and self.long_override_counter < 5

      self.long_disabled_counter = min(self.long_disabled_counter + 1, 5) if not CC.enabled else 0
      long_disabling = not CC.enabled and self.long_disabled_counter < 5

      acc_hold_type = mebcan.acc_hold_type(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, starting, stopping,
                                           CS.esp_hold_confirmation, override, override_begin, long_disabling)
      can_sends.extend(mebcan.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.enabled, accel,
                                                       acc_control, acc_hold_type, stopping, starting, CS.esp_hold_confirmation,
                                                       override, CS.travel_assist_available, dm_brake is not None))

    # **** HUD ************************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(mebcan.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                     CS.out.steeringPressed, hud_alert, hud_control))

    if self.CP.openpilotLongitudinalControl and self.frame % self.CCP.ACC_HUD_STEP == 0:
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(mebcan.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_control, set_speed))

    # **** Emergency-stop auto-horn ***************************************** #
    if self.frame % self.CCP.LDW_STEP == 0 and dm_phase3:
      can_sends.append(mebcan.create_emergency_horn(self.packer_pt, self.CAN.pt))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(mebcan.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                         cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
