def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x42D: 0xC}.get(address, 0xB)
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

def create_resume_acc(packer, resume, hs2_dat_mdd_cmd_452):
  hs2_dat_mdd_cmd_452['COCKPIT_GO_ACC_REQUEST'] = resume
  return packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, hs2_dat_mdd_cmd_452)

def create_gas(packer, gas, driver):
  driver['GAS_PEDAL'] = gas
  return packer.make_can_msg('DRIVER', 0, driver)

def create_dyn_cmm(packer, gas, dyn_cmm):
  dyn_cmm['P002_Com_rAPP'] = gas
  return packer.make_can_msg('Dyn_CMM', 2, dyn_cmm)