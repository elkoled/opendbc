# MEB CAN message packers for the minimal lateral-only port.
# Longitudinal (ACC_18 / ACC_02 / EA / KLR / radar replacement) packers are intentionally omitted
# and will be added in a follow-up commit when minimal OP-long is enabled.


def create_steering_control(packer, bus, apply_curvature, lkas_enabled, power):
  values = {
    "Curvature":     abs(apply_curvature),                          # in rad/m
    "Curvature_VZ":  1 if apply_curvature > 0 and lkas_enabled else 0,
    "Power":         power if lkas_enabled else 0,
    "RequestStatus": 4 if lkas_enabled else 2,                      # 4 = control active, 2 = standby
    "HighSendRate":  lkas_enabled,
  }
  return packer.make_can_msg("HCA_03", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control, sound_alert):
  display_mode = 1 if lat_active else 0  # travel-assist style: yellow lanes while OP is active

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
    "LDW_Gong":             sound_alert,
    "LDW_Status_LED_gelb":  1 if lat_active and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if lat_active and not steering_pressed else 0,
    "LDW_Lernmodus_links":  3 + display_mode if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible + display_mode,
    "LDW_Lernmodus_rechts": 3 + display_mode if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible + display_mode,
    "LDW_Texte":            hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


ACCEL_INACTIVE = 3.01      # one increment above the active range — stock idle marker
ACCEL_OVERRIDE = 0.0       # non-inactive accel that the car interprets as "driver overriding"
ACC_CTRL_ERROR    = 6
ACC_CTRL_OVERRIDE = 4
ACC_CTRL_ACTIVE   = 3
ACC_CTRL_ENABLED  = 2
ACC_CTRL_DISABLED = 0
ACC_HMS_RELEASE      = 4   # release stop-hold (drivetrain start moving from full stop)
ACC_HMS_HOLD         = 1   # request stop-hold (EPB / brake-by-wire holds the car)
ACC_HMS_NO_REQUEST   = 0


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control,
                             stopping, starting, esp_hold, override, speed):
  # The TSK is byte-sensitive: an active ACC_18 must follow the stock layout closely or it faults.
  # In particular: ACC_Sollbeschleunigung_02 must be ACCEL_OVERRIDE (0.0) — not ACCEL_INACTIVE — while
  # the driver is overriding (the stock radar keeps a "live" accel during gas-press), and must be
  # ACCEL_INACTIVE while at full stop on newer-gen cars (a non-neutral accel at standstill faults).
  full_stop          = stopping and esp_hold
  actually_stopping  = stopping and not esp_hold
  active             = acc_control == ACC_CTRL_ACTIVE
  active_or_override = acc_control in (ACC_CTRL_ACTIVE, ACC_CTRL_OVERRIDE)

  if acc_enabled:
    if override:
      acceleration = ACCEL_OVERRIDE
    elif full_stop:
      acceleration = ACCEL_INACTIVE
    else:
      acceleration = accel
  else:
    acceleration = ACCEL_INACTIVE

  # Hold mode signal: drives the car's stop-and-go state machine.
  if not acc_enabled or override:
    hold_mode = ACC_HMS_NO_REQUEST
  elif starting:
    hold_mode = ACC_HMS_RELEASE
  elif stopping or esp_hold:
    hold_mode = ACC_HMS_HOLD
  else:
    hold_mode = ACC_HMS_NO_REQUEST

  values = {
    "ACC_Typ":                    acc_type,
    "ACC_Status_ACC":             acc_control,
    "ACC_StartStopp_Info":        acc_enabled,
    "ACC_Sollbeschleunigung_02":  acceleration,
    "ACC_zul_Regelabw_unten":     0.2 if active_or_override else 0,
    "ACC_zul_Regelabw_oben":      0.2 if active_or_override else 0,
    "ACC_neg_Sollbeschl_Grad_02": 4.0 if active_or_override else 0,
    "ACC_pos_Sollbeschl_Grad_02": 4.0 if active_or_override else 0,
    "ACC_Anfahren":               starting,
    "ACC_Anhalten":               1 if actually_stopping else 0,
    "ACC_Anhalteweg":             0 if actually_stopping else 20.46,
    "ACC_Anforderung_HMS":        hold_mode,
    "ACC_AKTIV_regelt":           1 if active else 0,
    "Speed":                      speed,
    "SET_ME_0XFE":                0xFE,
    "SET_ME_0X1":                 0x1,
    "SET_ME_0X9":                 0x9,
  }
  return packer.make_can_msg("ACC_18", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active, override):
  # ACC_Status_ACC for ACC_18. Override path keeps the cruise controller "alive" so the car
  # doesn't reject our message stream while the driver is on the gas.
  if acc_faulted:
    return ACC_CTRL_ERROR
  if long_active:
    return ACC_CTRL_OVERRIDE if override else ACC_CTRL_ACTIVE
  return ACC_CTRL_ENABLED if main_switch_on else ACC_CTRL_DISABLED


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  # Pass-through of stock GRA_ACC_01 with cancel/resume injection (used to forward driver button presses).
  values = {s: gra_stock_values[s] for s in [
    "GRA_Hauptschalter",
    "GRA_Typ_Hauptschalter",
    "GRA_Codierung",
    "GRA_Tip_Stufe_2",
    "GRA_ButtonTypeInfo",
  ]}
  values.update({
    "COUNTER":               (gra_stock_values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen":         cancel,
    "GRA_Tip_Wiederaufnahme": resume,
  })
  return packer.make_can_msg("GRA_ACC_01", bus, values)
