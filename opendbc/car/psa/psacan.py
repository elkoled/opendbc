def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x4f8: 0x4, 0x208: 0x5, 0x38D: 0x7, 0x2f6: 0x8, 0x42D: 0xC}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF

# TODO: delete debug param LANE_DEPARTURE
def create_lka_steering(packer, lat_active: bool, apply_angle: float, status: int, debug: int):
  values = {
    'DRIVE': 1,
    'STATUS': status,
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
    'LANE_DEPARTURE': debug, # 0: off/pid, 1: starting
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)

def create_resume_acc(packer, counter, status, hs2_dat_mdd_cmd_452):
  hs2_dat_mdd_cmd_452['COUNTER'] = counter
  hs2_dat_mdd_cmd_452['COCKPIT_GO_ACC_REQUEST'] = status
  return packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, hs2_dat_mdd_cmd_452)

def create_drive_away_request(packer, hs2_dyn_mdd_etat_2f6):
  hs2_dyn_mdd_etat_2f6['DRIVE_AWAY_REQUEST'] = 0
  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, hs2_dyn_mdd_etat_2f6)