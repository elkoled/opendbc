import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_std_curvature_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mebcan, mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# VW MEB EPS plant static-gain table — fit on 606 routes / 7 dongles of real
# VW ID4_MK1 device uploads (data-fallback.comma.life rlogs, 286k+ samples).
# K = QFK_01.Curvature / HCA_03.Curvature in steady state, at Power = at_cap
# (the dominant operating state with STEERING_POWER_MAX=50).
# Per-bin OLS slopes; R² in [0.86, 0.98]; 90% bootstrap CI width ~ 0.005-0.05.
# K(v) peaks at ~0.97 around 10–20 m/s and dips to ~0.88 at 25–30 m/s — the
# highway hot zone. K < 1 means rack under-delivers commanded curvature; we
# compensate by multiplying actuators.curvature by 1/K before sending it
# through the safety rate limiter. Boost is clipped to [1.00, 1.20] for
# safety. See tools/lateral_maneuvers/lateral_audit/EPS_MODEL_CARD.md.
EPS_MEB_K_TABLE = (
  # (v_low_mps, K)
  (0.0,  0.829), (5.0,  0.937), (10.0, 0.974), (15.0, 0.966),
  (20.0, 0.938), (25.0, 0.882), (30.0, 0.902), (36.0, 0.902),
)
_EPS_MEB_V = [v for v, _ in EPS_MEB_K_TABLE]
_EPS_MEB_K = [k for _, k in EPS_MEB_K_TABLE]
_EPS_BOOST_MIN = 1.00
_EPS_BOOST_MAX = 1.20
_EPS_V_BOOST_MIN = 5.0   # don't apply boost below this — low-speed already fine
_EPS_V_BOOST_MAX = 36.0  # don't apply boost above the fit range


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
    elif CP.flags & VolkswagenFlags.MEB:
      self.CCS = mebcan
    else:
      self.CCS = mqbcan

    self.apply_torque_last = 0
    self.apply_curvature_last = 0.
    self.steering_power_last = 0
    self.accel_last = 0.
    self.long_override_counter = 0
    self.long_disabled_counter = 0
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    self.gra_acc_counter_last = None
    self.klr_counter_last = None
    self.hca_mitigation = HCAMitigation(self.CCP)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # MEB-only Emergency-Assist pacification gate (capacitive wheel-touch).
    # VW EA fires a brake jolt after ~30 s of no driver hands-on detection.
    # We mask EA by continuously sending fake capacitive wheel-touch messages,
    # which means a distracted-driver alert from openpilot can never escalate
    # to the stock brake jolt. Two changes here:
    #   1. On openpilot's driver-distracted alert (VisualAlert.steerRequired),
    #      stop sending the wheel-touch messages entirely so the stock brake
    #      jolt actually fires and wakes the driver.
    #   2. Even during normal operation, only PULSE the wheel touch for ~1 s
    #      every 28 s instead of streaming it continuously. This lets VW EA's
    #      internal hands-off timer charge up to ~27 s between pulses; when
    #      openpilot's distracted alert hits, the brake jolt fires within a
    #      handful of seconds instead of waiting the full 30 s window.
    # openpilot stays engaged throughout — only the wheel-touch stream is
    # gated. MEB-only; MQB/PQ paths unchanged.
    ea_distracted = hud_control.visualAlert == VisualAlert.steerRequired
    _EA_PACIFY_PERIOD_FRAMES = int(28.0 / DT_CTRL)
    _EA_PACIFY_BURST_FRAMES = int(1.0 / DT_CTRL)
    ea_pacify_send_meb = bool(self.CP.flags & VolkswagenFlags.MEB) \
                         and not ea_distracted \
                         and (self.frame % _EA_PACIFY_PERIOD_FRAMES) < _EA_PACIFY_BURST_FRAMES

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      apply_torque = 0
      if self.CP.flags & VolkswagenFlags.MEB:
        # Logic to avoid HCA refused state:
        #   * steering power as counter and near zero before OP lane assist deactivation
        # MEB rack can be used continuously without time limits
        # maximum real steering angle change ~ 120-130 deg/s

        if CC.latActive:
          hca_enabled = True
          # EPS gain compensation (see EPS_MEB_K_TABLE above): boost the
          # commanded curvature by 1/K(v_ego) to counter the measured
          # under-delivery of the MEB rack at the current STEERING_POWER cap.
          if _EPS_V_BOOST_MIN <= CS.out.vEgo <= _EPS_V_BOOST_MAX:
            K = float(np.interp(CS.out.vEgo, _EPS_MEB_V, _EPS_MEB_K))
            K_boost = float(np.clip(1.0 / K, _EPS_BOOST_MIN, _EPS_BOOST_MAX))
          else:
            K_boost = 1.0
          apply_curvature = K_boost * actuators.curvature + (CS.curvature_meas - CC.currentCurvature)
          apply_curvature = apply_std_curvature_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw, CS.curvature_meas,
                                                       self.CCP.STEER_STEP, CC.latActive, self.CCP.CURVATURE_LIMITS)

          min_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MIN)
          max_power = min(self.steering_power_last + self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MAX)
          target_power_driver = int(np.interp(CS.out.steeringTorque, [self.CCP.STEER_DRIVER_ALLOWANCE, self.CCP.STEER_DRIVER_MAX],
                                                                     [self.CCP.STEERING_POWER_MAX, self.CCP.STEERING_POWER_MIN]))
          target_power = int(np.interp(CS.out.vEgo, [0., 0.5], [self.CCP.STEERING_POWER_MIN, target_power_driver]))
          steering_power = min(max(target_power, min_power), max_power)

        else:
          if self.steering_power_last > 0:  # keep HCA alive until steering power has reduced to zero
            hca_enabled = True
            apply_curvature = float(np.clip(CS.curvature_meas, -self.CCP.CURVATURE_LIMITS.CURVATURE_MAX, self.CCP.CURVATURE_LIMITS.CURVATURE_MAX))
            steering_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, 0)
          else:
            hca_enabled = False
            apply_curvature = 0.  # inactive curvature
            steering_power = 0

        can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_curvature, hca_enabled, steering_power))
        self.apply_curvature_last = apply_curvature
        self.steering_power_last = steering_power

      else:
        # Logic to avoid HCA state 4 "refused":
        #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
        #   * Don't steer at standstill
        #   * Don't send > 3.00 Newton-meters torque
        #   * Don't send the same torque for > 6 seconds
        #   * Don't send uninterrupted steering for > 360 seconds
        # MQB racks reset the uninterrupted steering timer after a single frame
        # of HCA disabled; this is done whenever output happens to be zero.
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

    # Emergency Assist intervention
    if self.CP.flags & VolkswagenFlags.MEB and self.CP.flags & VolkswagenFlags.STOCK_KLR_PRESENT:
      # send capacitive steering wheel touched
      # probably EA is stock activated only for cars equipped with capacitive steering wheel
      # (also stock long does resume from stop as long as hands on is detected additionally to OP resume spam)
      klr_send_ready = CS.klr_stock_values["COUNTER"] != self.klr_counter_last
      # Gate on the EA-pacification pulse window — see top of update().
      if klr_send_ready and ea_pacify_send_meb:
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.cam, CC.latActive, CS.klr_stock_values))
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.pt, CC.latActive, CS.klr_stock_values))
      self.klr_counter_last = CS.klr_stock_values["COUNTER"]

    # **** Acceleration Controls ******************************************** #

    if self.frame % self.CCP.ACC_CONTROL_STEP == 0 and self.CP.openpilotLongitudinalControl:
      stopping = actuators.longControlState == LongCtrlState.stopping

      if self.CP.flags & VolkswagenFlags.MEB:
        starting = actuators.longControlState == LongCtrlState.starting and CS.out.vEgo <= self.CP.vEgoStarting
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.enabled else 0)

        long_override = CC.cruiseControl.override or CS.out.gasPressed
        self.long_override_counter = min(self.long_override_counter + 1, 5) if long_override else 0
        long_override_begin = long_override and self.long_override_counter < 5

        self.long_disabled_counter = min(self.long_disabled_counter + 1, 5) if not CC.enabled else 0
        long_disabling = not CC.enabled and self.long_disabled_counter < 5

        acc_control = mebcan.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, long_override)
        acc_hold_type = mebcan.acc_hold_type(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled, starting, stopping,
                                             CS.esp_hold_confirmation, long_override, long_override_begin, long_disabling)
        can_sends.extend(mebcan.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.enabled,
                                                         4.0, 4.0, 0., 0.,
                                                         accel, acc_control, acc_hold_type, stopping, starting, CS.esp_hold_confirmation,
                                                         CS.out.vEgoRaw * CV.MS_TO_KPH, long_override, CS.travel_assist_available))
        self.accel_last = accel

      else:
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)

        if self.CP.flags & VolkswagenFlags.PQ:
          long_ccs = pqcan
        elif self.CP.flags & VolkswagenFlags.MLB:
          long_ccs = mlbcan
        else:
          long_ccs = mqbcan
        acc_control = long_ccs.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        can_sends.extend(long_ccs.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.longActive, accel,
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

    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      self.distance_bar_frame = self.frame

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      if self.CP.flags & VolkswagenFlags.MEB:
        fcw_alert = hud_control.visualAlert == VisualAlert.fcw
        show_distance_bars = self.frame - self.distance_bar_frame < 400
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:
          lead_distance = 8
        acc_hud_status = mebcan.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.enabled,
                                                     CC.cruiseControl.override or CS.out.gasPressed)
        can_sends.append(mebcan.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, hud_control.setSpeed * CV.MS_TO_KPH,
                                                       hud_control.leadVisible, hud_control.leadDistanceBars + 1, show_distance_bars,
                                                       CS.esp_hold_confirmation, lead_distance, 0, fcw_alert))

      else:
        lead_distance = 0
        if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
          lead_distance = 512 if CS.upscale_lead_car_signal else 8
        if self.CP.flags & VolkswagenFlags.PQ:
          long_ccs = pqcan
        elif self.CP.flags & VolkswagenFlags.MLB:
          long_ccs = mlbcan
        else:
          long_ccs = mqbcan
        acc_hud_status = long_ccs.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        # FIXME: PQ may need to use the on-the-wire mph/kmh toggle to fix rounding errors
        # FIXME: Detect clusters with vEgoCluster offsets and apply an identical vCruiseCluster offset
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        can_sends.append(long_ccs.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                         lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.curvature = float(self.apply_curvature_last)
    new_actuators.accel = self.accel_last

    self.lead_distance_bars_last = hud_control.leadDistanceBars
    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
