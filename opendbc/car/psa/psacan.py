from opendbc.car.can_definitions import CanData


def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x4f8: 0x4, 0x208: 0x5, 0x38D: 0x7, 0x2f6: 0x8, 0x2b6: 0xC, 0x42D: 0xC}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF


def create_lka_steering(packer, lat_active: bool, apply_angle: float, status: int):
  values = {
    'DRIVE': 1,
    'STATUS': status,
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)


def create_resume_acc(packer, counter, status, hs2_dat_mdd_cmd_452):
  hs2_dat_mdd_cmd_452['COUNTER'] = counter
  hs2_dat_mdd_cmd_452['COCKPIT_GO_ACC_REQUEST'] = status
  return packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, hs2_dat_mdd_cmd_452)


def create_drive_away_request(packer, hs2_dyn_mdd_etat_2f6):
  hs2_dyn_mdd_etat_2f6['DRIVE_AWAY_REQUEST'] = 0
  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, hs2_dyn_mdd_etat_2f6)


# Radar, 50 Hz
def create_HS2_DYN1_MDD_ETAT_2B6(packer, frame: int, accel: float, enabled: bool, gasPressed: bool, braking: int, brakePressed: int, standstill: bool, torque: int):
  # TODO: if gas pressed, ACC_STATUS is set to suspended and decel can be set negative (about -300 Nm / -0.6m/s²) with brake mode inactive
  # TODO: tune torque multiplier
  # TODO: check difference between GMP_POTENTIAL_WHEEL_TORQUE and GMP_WHEEL_TORQUE
  # TODO: transition from waiting to active enables torque control. For now, deactivate autohold or enable on brake pressed

  values = {
    'MDD_DESIRED_DECELERATION': (-10.65 if standstill else accel) if  braking and enabled else 2.05, # m/s²
    'POTENTIAL_WHEEL_TORQUE_REQUEST': (2 if braking else 1) if enabled else 0,
    'MIN_TIME_FOR_DESIRED_GEAR': 0.0 if braking or not enabled else 6.2,
    'GMP_POTENTIAL_WHEEL_TORQUE': torque if not braking and enabled else -4000,
    'ACC_STATUS': (5 if gasPressed else 2 if brakePressed and not standstill else 4) if enabled else (2 if brakePressed and not standstill else 3),
    'GMP_WHEEL_TORQUE': torque if not braking and enabled else -4000,
    'WHEEL_TORQUE_REQUEST': 1 if enabled and not braking else 0, # TODO: test 1: high torque range 2: low torque range
    'AUTO_BRAKING_STATUS': 3, # AEB # TODO: testing ALWAYS ENABLED to resolve DTC errors if enabled else 3, # maybe disabled on too high steering angle
    'MDD_DECEL_TYPE': int(braking),
    'MDD_DECEL_CONTROL_REQ': int(braking),
  }

  return packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', 1, values)


# Radar, 50 Hz
def create_HS2_DYN_MDD_ETAT_2F6(packer, braking, lead_detected):
  values = {
    'TARGET_DETECTED': lead_detected, # TODO: <target detected>
    # 'REQUEST_TAKEOVER': 0, # TODO potential signal for HUD message from OP
    # 'BLIND_SENSOR': 0,
    # 'REQ_VISUAL_COLL_ALERT_ARC': 0,
    # 'REQ_AUDIO_COLL_ALERT_ARC': 0,
    # 'REQ_HAPTIC_COLL_ALERT_ARC': 0,
    'INTER_VEHICLE_DISTANCE': 255.5, # TODO: <distance> if enabled else 255.5,
    'ARC_STATUS': 6,  # 12 after 50 frames (1 sec) after AUTO_BRAKING_STATUS else 6
    # 'AUTO_BRAKING_IN_PROGRESS': 0,
    # 'AEB_ENABLED': 0,
    # 'DRIVE_AWAY_REQUEST': 0, # TODO: potential RESUME request?
    'DISPLAY_INTERVEHICLE_TIME': 6.2, # TODO: <time to vehicle> if enabled else 6.2,
    'MDD_DECEL_CONTROL_REQ': int(braking),
    'AUTO_BRAKING_STATUS': 3, # AEB # TODO: testing ALWAYS ENABLED to resolve DTC errors if enabled else 3, # maybe disabled on too high steering angle
    'TARGET_POSITION': 4, # distance to lead car, far - 4, 3, 2, 1 - near
  }

  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, values)


# Radar, 10 Hz
def create_HS2_DAT_ARTIV_V2_4F6(packer, enabled: bool):
  values = {
    'TIME_GAP':  3.0 if enabled else 25.5, # TODO sync with 2F6
    'DISTANCE_GAP': 100 if enabled else 254, # TODO sync with 2F6
    'RELATIVE_SPEED': 0.0 if enabled else 93.8,
    'ARTIV_SENSOR_STATE': 2,
    'TARGET_DETECTED': 0, # 1 if enabled else 0,
    'ARTIV_TARGET_CHANGE_INFO': 0,
    'TRAFFIC_DIRECTION': 0, # Right hand traffic
  }
  return packer.make_can_msg('HS2_DAT_ARTIV_V2_4F6', 1, values)


# Radar, 1 Hz
def create_HS2_SUPV_ARTIV_796(packer):
  values = {
    'FAULT_CODE': 0,
    'STATUS_NO_CONFIG': 0,
    'STATUS_PARTIAL_WAKEUP_GMP': 0,
    'UCE_ELECTR_STATE': 0,
  }
  return packer.make_can_msg('HS2_SUPV_ARTIV_796', 1, values)


# Disable radar ECU by setting it to programming mode
def create_disable_radar():
  addr = 0x6B6
  bus = 1
  dat = [0x02, 0x10, 0x02, 0x80]
  dat.extend([0x0] * (8 - len(dat)))

  return CanData(addr, bytes(dat), bus)