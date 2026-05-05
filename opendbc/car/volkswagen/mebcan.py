ACCEL_INACTIVE = 3.01  # one increment above the active range, stock idle marker
ACC_CTRL_ERROR    = 6
ACC_CTRL_OVERRIDE = 4
ACC_CTRL_ACTIVE   = 3
ACC_CTRL_ENABLED  = 2
ACC_CTRL_DISABLED = 0
ACC_HMS_RAMP_RELEASE = 5
ACC_HMS_RELEASE      = 4
ACC_HMS_HOLD         = 1
ACC_HMS_NO_REQUEST   = 0


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
    values = {s: ldw_stock_values[s] for s in ("LDW_SW_Warnung_links", "LDW_SW_Warnung_rechts",
                                                "LDW_Seite_DLCTLC", "LDW_DLC", "LDW_TLC")}

  values.update({
    "LDW_Status_LED_gelb":  1 if lat_active and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if lat_active and not steering_pressed else 0,
    "LDW_Lernmodus_links":  3 + display_mode if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible + display_mode,
    "LDW_Lernmodus_rechts": 3 + display_mode if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible + display_mode,
    "LDW_Texte":            hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


def create_capacitive_wheel_touch(packer, bus, lat_active, klr_stock_values):
  values = {s: klr_stock_values[s] for s in [
    "COUNTER",
    "KLR_Touchintensitaet_1",
    "KLR_Touchintensitaet_2",
    "KLR_Touchintensitaet_3",
    "KLR_Touchauswertung",
  ]}

  if lat_active:
    values.update({
      "COUNTER": (klr_stock_values["COUNTER"] + 1) % 16,
      "KLR_Touchintensitaet_1": 80,
      "KLR_Touchintensitaet_2": 200,
      "KLR_Touchintensitaet_3": 10,
      "KLR_Touchauswertung": 10,
    })
  return packer.make_can_msg("KLR_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active, override):
  if long_active:
    if acc_faulted:
      return ACC_CTRL_ERROR
    return ACC_CTRL_OVERRIDE if override else ACC_CTRL_ACTIVE
  return ACC_CTRL_ENABLED if main_switch_on else ACC_CTRL_DISABLED


def acc_hold_type(main_switch_on, acc_faulted, long_active, starting, stopping, esp_hold, override, override_begin, long_disabling):
  # warning: car is reacting to hold mechanic even with long control off

  if acc_faulted:
    acc_hold_type = ACC_HMS_NO_REQUEST # no hold request
  elif not long_active:
    if long_disabling:
      acc_hold_type = ACC_HMS_RAMP_RELEASE # ramp release of requests right after disabling long control (prevents car error with EPB at low speed)
    else:
      acc_hold_type = ACC_HMS_NO_REQUEST # no hold request
  elif override:
    if override_begin:
      acc_hold_type = ACC_HMS_RAMP_RELEASE # ramp release of requests at the beginning of override (prevents car error with EPB at low speed)
    else:
      acc_hold_type = ACC_HMS_NO_REQUEST # overriding / no request
  elif starting:
    acc_hold_type = ACC_HMS_RELEASE # release request and startup
  elif stopping or esp_hold:
    acc_hold_type = ACC_HMS_HOLD # hold or hold request
  else:
    acc_hold_type = ACC_HMS_NO_REQUEST # no hold request

  return acc_hold_type


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, acc_hold_type, stopping, starting,
                             esp_hold, override, travel_assist_available):
  active = acc_control == ACC_CTRL_ACTIVE
  if not acc_enabled or (stopping and esp_hold and not starting):
    accel_out = ACCEL_INACTIVE
  elif override:
    accel_out = 0.0
  else:
    accel_out = accel

  commands = []

  values = {
    "ACC_Typ":                    acc_type,
    "ACC_Status_ACC":             acc_control,
    "ACC_StartStopp_Info":        acc_enabled,
    "ACC_Sollbeschleunigung_02":  accel_out,
    "ACC_zul_Regelabw_unten":     0.2 if active else 0,
    "ACC_zul_Regelabw_oben":      0.2 if active else 0,
    "ACC_neg_Sollbeschl_Grad_02": 0.5 if acc_control == ACC_CTRL_OVERRIDE else (4.0 if active else 0),
    "ACC_pos_Sollbeschl_Grad_02": 0.5 if acc_control == ACC_CTRL_OVERRIDE else (4.0 if active else 0),
    "ACC_Anfahren":               starting,
    "ACC_Anhalten":               stopping and not esp_hold,
    "ACC_Anhalteweg":             0 if (stopping and not esp_hold) else 20.46,
    "ACC_Anforderung_HMS":        acc_hold_type,
    "ACC_AKTIV_regelt":           1 if active else 0,
    "SET_ME_0XFE":                0xFE,
    "SET_ME_0X1":                 0x1,
    "SET_ME_0X9":                 0x9,
  }
  commands.append(packer.make_can_msg("ACC_18", bus, values))

  if travel_assist_available:
    values_ta = {
      "Travel_Assist_Status":    4 if acc_enabled else 2,
      "Travel_Assist_Request":   0,
      "Travel_Assist_Available": 1,
    }
    commands.append(packer.make_can_msg("TA_01", bus, values_ta))

  return commands


def create_acc_hud_control(packer, bus, acc_control, set_speed):
  values = {
    "ACC_Status_ACC":      acc_control,
    "ACC_Wunschgeschw_02": min(set_speed, 327.36),
    "ACC_Display_Prio":    1,
  }
  return packer.make_can_msg("MEB_ACC_01", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = {s: gra_stock_values[s] for s in ("GRA_Hauptschalter", "GRA_Typ_Hauptschalter", "GRA_Codierung",
                                              "GRA_Tip_Stufe_2", "GRA_ButtonTypeInfo")}
  values.update({
    "COUNTER":                (gra_stock_values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen":          cancel,
    "GRA_Tip_Wiederaufnahme": resume,
  })
  return packer.make_can_msg("GRA_ACC_01", bus, values)
