from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from opendbc.car.lateral import ISO_LATERAL_ACCEL, ISO_LATERAL_JERK

AVERAGE_ROAD_ROLL = 0.06  # 3.4 deg, AVERAGE_ROAD_ROLL in lateral.h


def apply_curvature_limits(curvature: float, curvature_last: float, v_ego: float,
                           max_curvature_abs: float, send_rate: float = 0.02) -> float:
  speed = max(v_ego - 1.0, 1.0)
  max_iso_curvature = (ISO_LATERAL_ACCEL + ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL) / (speed * speed)
  max_iso_curvature = min(max_iso_curvature, max_curvature_abs)
  max_curvature_delta = ISO_LATERAL_JERK / (speed * speed) * send_rate
  high = min(curvature_last + max_curvature_delta,  max_iso_curvature)
  low  = max(curvature_last - max_curvature_delta, -max_iso_curvature)
  return min(max(curvature, low), high)


def create_steering_control(packer, bus, apply_curvature, lkas_enabled, power):
  values = {
    "Curvature":     abs(apply_curvature),
    "Curvature_VZ":  1 if apply_curvature > 0 and lkas_enabled else 0,
    "Power":         power if lkas_enabled else 0,
    "RequestStatus": 4 if lkas_enabled else 2,  # 4: control active, 2: standby
    "HighSendRate":  lkas_enabled,
  }
  return packer.make_can_msg("HCA_03", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control):
  display_mode = 1 if lat_active else 0

  values = {}
  if len(ldw_stock_values):
    values = {s: ldw_stock_values[s] for s in [
      "LDW_SW_Warnung_links",
      "LDW_SW_Warnung_rechts",
      "LDW_Seite_DLCTLC",
      "LDW_DLC",
      "LDW_TLC",
    ]}

  values.update({
    "LDW_Status_LED_gelb":  1 if lat_active and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if lat_active and not steering_pressed else 0,
    "LDW_Lernmodus_links":  3 + display_mode if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible + display_mode,
    "LDW_Lernmodus_rechts": 3 + display_mode if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible + display_mode,
    "LDW_Texte":            hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


ACCEL_INACTIVE = 3.01  # one increment above the active range, stock idle marker
ACCEL_OVERRIDE = 0.0
ACC_CTRL_ERROR    = 6
ACC_CTRL_OVERRIDE = 4
ACC_CTRL_ACTIVE   = 3
ACC_CTRL_ENABLED  = 2
ACC_CTRL_DISABLED = 0
ACC_HMS_RAMP_RELEASE = 5   # smooths EPB transition at override start / long disable
ACC_HMS_RELEASE      = 4
ACC_HMS_HOLD         = 1
ACC_HMS_NO_REQUEST   = 0


def acc_hold_type(acc_faulted, long_active, starting, stopping, esp_hold, override, override_begin, long_disabling):
  # Emit RAMP_RELEASE for the first 5 frames after override or long control disables,
  # otherwise EPB faults during low-speed transitions
  if acc_faulted:
    return ACC_HMS_NO_REQUEST
  if not long_active:
    return ACC_HMS_RAMP_RELEASE if long_disabling else ACC_HMS_NO_REQUEST
  if override:
    return ACC_HMS_RAMP_RELEASE if override_begin else ACC_HMS_NO_REQUEST
  if starting:
    return ACC_HMS_RELEASE
  if stopping or esp_hold:
    return ACC_HMS_HOLD
  return ACC_HMS_NO_REQUEST


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, acc_hold,
                             stopping, starting, esp_hold, override, speed):
  # ACC_Sollbeschleunigung_02 must be ACCEL_OVERRIDE during gas press
  full_stop          = stopping and esp_hold
  full_stop_no_start = esp_hold and not starting
  actually_stopping  = stopping and not esp_hold
  active             = acc_control == ACC_CTRL_ACTIVE
  active_or_override = acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE)
  active_sending     = active_or_override and not full_stop_no_start

  if acc_enabled:
    if override:
      acceleration = ACCEL_OVERRIDE
    elif full_stop:
      acceleration = ACCEL_INACTIVE
    else:
      acceleration = accel
  else:
    acceleration = ACCEL_INACTIVE

  values = {
    "ACC_Typ":                    acc_type,
    "ACC_Status_ACC":             acc_control,
    "ACC_StartStopp_Info":        acc_enabled,
    "ACC_Sollbeschleunigung_02":  acceleration,
    "ACC_zul_Regelabw_unten":     0.2 if active_sending else 0,
    "ACC_zul_Regelabw_oben":      0.2 if active_sending else 0,
    "ACC_neg_Sollbeschl_Grad_02": 4.0 if active_sending else 0,
    "ACC_pos_Sollbeschl_Grad_02": 4.0 if active_sending else 0,
    "ACC_Anfahren":               starting,
    "ACC_Anhalten":               1 if actually_stopping else 0,
    "ACC_Anhalteweg":             0 if actually_stopping else 20.46,
    "ACC_Anforderung_HMS":        acc_hold,
    "ACC_AKTIV_regelt":           1 if active else 0,
    "Speed":                      speed,
    "SET_ME_0XFE":                0xFE,
    "SET_ME_0X1":                 0x1,
    "SET_ME_0X9":                 0x9,
  }
  return packer.make_can_msg("ACC_18", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active, override):
  # OVERRIDE keeps the controller alive while the driver presses gas
  if long_active:
    if acc_faulted:
      return ACC_CTRL_ERROR
    return ACC_CTRL_OVERRIDE if override else ACC_CTRL_ACTIVE
  return ACC_CTRL_ENABLED if main_switch_on else ACC_CTRL_DISABLED


def create_acc_hud_control(packer, bus, acc_control, set_speed):
  # TODO: remove
  values = {
    "ACC_Status_ACC":      acc_control,
    "ACC_Wunschgeschw_02": min(set_speed, 327.36),
    "ACC_Display_Prio":    1,
  }
  return packer.make_can_msg("MEB_ACC_01", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = {s: gra_stock_values[s] for s in [
    "GRA_Hauptschalter",
    "GRA_Typ_Hauptschalter",
    "GRA_Codierung",
    "GRA_Tip_Stufe_2",
    "GRA_ButtonTypeInfo",
  ]}
  values.update({
    "COUNTER":                (gra_stock_values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen":          cancel,
    "GRA_Tip_Wiederaufnahme": resume,
  })
  return packer.make_can_msg("GRA_ACC_01", bus, values)
