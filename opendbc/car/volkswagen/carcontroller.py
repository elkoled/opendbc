import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import EpsCurvatureRatio, apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mebcan, mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.power_test import MebPowerTest
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


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

    if CP.flags & VolkswagenFlags.MEB:
      self.meb_long_state = mebcan.MebLongStateMachine(self.CP, self.CCP)

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    else:
      self.CCS = mqbcan

    self.apply_torque_last = 0
    self.apply_curvature_last = 0.
    self.steering_power_last = 0
    self.curvature_error_i = 0.
    self.eps_curvature_ratio = EpsCurvatureRatio()
    self.power_test = MebPowerTest()
    self.accel_last = 0.
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    self.gra_acc_counter_last = None
    self.hca_mitigation = HCAMitigation(self.CCP)
    self.brake_pressed_last = False
    self.brake_exit = False
    self.desired_curvature_lp = 0.
    self.desired_curvature = 0.

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    # **** Steering Controls ************************************************ #

    if self.CP.flags & VolkswagenFlags.MEB:
      self.eps_curvature_ratio.update(CS.curvature_meas, CS.out.yawRate, CS.out.vEgoRaw)
      # the panda exits controls on the brake a frame before openpilot does, stop actuating on the same frame
      if CS.out.brakePressed and (not self.brake_pressed_last or not CS.out.standstill):
        self.brake_exit = True
      elif not CC.enabled:
        self.brake_exit = False
      self.brake_pressed_last = CS.out.brakePressed

    if self.frame % self.CCP.STEER_STEP == 0:
      apply_torque = 0
      if self.CP.flags & VolkswagenFlags.MEB:
        lat_active = CC.latActive and not self.brake_exit
        test = self.power_test.update(lat_active, CS.out.vEgo, CS.out.steeringTorque, getattr(CS, "hca_status", None), CS.out.steeringPressed,
                                      CS.curvature_meas, CS.out.steeringAngleDeg, CS.out.steeringRateDeg, CS.out.yawRate,
                                      self.apply_curvature_last, self.steering_power_last)
        # Logic to avoid HCA refused state:
        #   * steering power as counter and near zero before OP lane assist deactivation
        # MEB rack can be used continuously without time limits
        # maximum real steering angle change ~ 120-130 deg/s

        if lat_active:
          hca_enabled = True
          # the EPS lags and settles short of the command at low speed, so close the loop on the current curvature
          # it holds the car against light driver touches and fades out as the driver takes over
          # the EPS passes plan noise straight through, filter it in two stages for a steep cut at the same lag
          smooth_tau = float(np.interp(CS.out.vEgo, self.CCP.CURVATURE_SMOOTH_BP, self.CCP.CURVATURE_SMOOTH_V))
          alpha = 1. - np.exp(-self.CCP.STEER_STEP * DT_CTRL / smooth_tau)
          self.desired_curvature_lp += alpha * (actuators.curvature - self.desired_curvature_lp)
          self.desired_curvature += alpha * (self.desired_curvature_lp - self.desired_curvature)
          curvature_error = self.desired_curvature - CC.currentCurvature
          driver_torque = abs(CS.out.steeringTorque)
          if driver_torque >= self.CCP.STEER_DRIVER_MAX:
            self.curvature_error_i = 0.
          elif not CS.out.steeringPressed and abs(self.apply_curvature_last - CS.curvature_meas) < self.CCP.CURVATURE_I_MAX_LAG:
            self.curvature_error_i = float(np.clip(self.curvature_error_i + self.CCP.CURVATURE_KI * curvature_error * self.CCP.STEER_STEP * DT_CTRL,
                                                   -self.CCP.CURVATURE_I_MAX, self.CCP.CURVATURE_I_MAX))
          feedback_gain = float(np.interp(driver_torque, [self.CCP.STEER_DRIVER_ALLOWANCE, self.CCP.STEER_DRIVER_MAX], [1., 0.]))
          kp = float(np.interp(CS.out.vEgo, self.CCP.CURVATURE_KP_BP, self.CCP.CURVATURE_KP_V))
          feedback = feedback_gain * (kp * curvature_error + self.curvature_error_i)
          # compensate the gap between measured and current curvature
          apply_curvature = self.desired_curvature + (CS.curvature_meas - CC.currentCurvature) + feedback
          apply_curvature = self.CCP.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw, CS.curvature_meas,
                                                                   lat_active, self.CCP.STEER_STEP, self.eps_curvature_ratio.get())

          power_max = self.CCP.STEERING_POWER_MAX
          min_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, self.CCP.STEERING_POWER_MIN)
          # after the power test, come down from above the max in steps too
          max_power = max(min(self.steering_power_last + self.CCP.STEERING_POWER_STEP, power_max), min_power)
          target_power_driver = int(np.interp(abs(CS.out.steeringTorque), [self.CCP.STEER_DRIVER_POWER_START, self.CCP.STEER_DRIVER_MAX],
                                                                          [power_max, self.CCP.STEERING_POWER_MIN]))
          target_power = int(np.interp(CS.out.vEgo, [0., 0.5], [self.CCP.STEERING_POWER_MIN, target_power_driver]))
          steering_power = min(max(target_power, min_power), max_power)

          if test is not None:
            apply_curvature = self.CCP.CURVATURE_LIMITS.apply_limits(test[0], self.apply_curvature_last, CS.out.vEgoRaw, CS.curvature_meas,
                                                                     lat_active, self.CCP.STEER_STEP, self.eps_curvature_ratio.get())
            steering_power = test[1]
            self.curvature_error_i = 0.
            self.desired_curvature_lp = actuators.curvature
            self.desired_curvature = actuators.curvature

        else:
          self.curvature_error_i = 0.
          self.desired_curvature_lp = actuators.curvature
          self.desired_curvature = actuators.curvature
          if self.steering_power_last > 0:  # keep HCA alive until steering power has reduced to zero
            hca_enabled = True
            # move to the measured curvature within the jerk limit, the panda may still allow controls after an EPS fault
            apply_curvature = self.CCP.CURVATURE_LIMITS.apply_limits(CS.curvature_meas, self.apply_curvature_last, CS.out.vEgoRaw, CS.curvature_meas,
                                                                     True, self.CCP.STEER_STEP, self.eps_curvature_ratio.get())
            steering_power = max(self.steering_power_last - self.CCP.STEERING_POWER_STEP, 0)
          else:
            hca_enabled = False
            apply_curvature = 0.  # inactive curvature
            steering_power = 0

        can_sends.append(mebcan.create_steering_control(self.packer_pt, self.CAN.pt, apply_curvature, hca_enabled, steering_power))
        self.apply_curvature_last = apply_curvature
        self.steering_power_last = steering_power

      else:
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
      # send capacitive steering wheel hands-on message to keep ACC resume active and control Emergency Assist
      # MEB Emergency Assist brake jerks after 30s of continued hands-off time.
      # We send the stock wheeltouch message to start the stock DM timer when openpilot latches the critical driver monitoring alert
      if self.frame % self.CCP.KLR_01_STEP == 0:
        lat_active = CC.latActive and not CC.driverMonitoringEscalation
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.cam, lat_active, CS.klr_stock_values))
        can_sends.append(mebcan.create_capacitive_wheel_touch(self.packer_pt, self.CAN.pt, lat_active, CS.klr_stock_values))

    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl:
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        if self.CP.flags & VolkswagenFlags.MEB:
          accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX))
          accel, acc_status, acc_hold_type, braking_to_stop, leaving_standstill = self.meb_long_state.update(CS, CC, accel, self.brake_exit)
          can_sends.extend(mebcan.create_acc_accel_control(self.packer_pt, self.CAN.pt, self.CCP, CS.acc_type, CC.enabled and not self.brake_exit,
                                                           accel, acc_status, acc_hold_type, braking_to_stop, leaving_standstill,
                                                           CS.out.vEgoRaw * CV.MS_TO_KPH, CS.travel_assist_available))

        else:
          stopping = actuators.longControlState == LongCtrlState.stopping
          acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
          accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
          starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < 0.25)
          can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.longActive, accel,
                                                             acc_control, stopping, starting, CS.esp_hold_confirmation))

        self.accel_last = accel

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
        can_sends.append(mebcan.create_acc_hud_control(self.packer_pt, self.CAN.pt, self.meb_long_state.acc_status, hud_control.setSpeed * CV.MS_TO_KPH,
                                                       hud_control.leadVisible, hud_control.leadDistanceBars, show_distance_bars,
                                                       lead_distance, fcw_alert))

      else:
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
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel_last

    self.lead_distance_bars_last = hud_control.leadDistanceBars
    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends
