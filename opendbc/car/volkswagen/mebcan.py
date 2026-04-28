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


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  values = {
    "ACC_Typ":                    acc_type,
    "ACC_Status_ACC":             acc_control,
    "ACC_StartStopp_Info":        acc_enabled,
    "ACC_Sollbeschleunigung_02":  accel if acc_enabled else 3.01,
    "ACC_zul_Regelabw_unten":     0.2,
    "ACC_zul_Regelabw_oben":      0.2,
    "ACC_neg_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,
    "ACC_pos_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,
    "ACC_Anfahren":               starting,
    "ACC_Anhalten":               stopping,
    "SET_ME_0XFE":                0xFE,
    "SET_ME_0X1":                 0x1,
    "SET_ME_0X9":                 0x9,
  }
  return packer.make_can_msg("ACC_18", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  # Mirrors mqbcan.acc_control_value: maps OP long state to ACC_Status_ACC for ACC_18.
  if acc_faulted:
    return 6
  if long_active:
    return 3
  return 2 if main_switch_on else 0


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
