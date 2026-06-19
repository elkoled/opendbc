import random

def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x42D: 0xC}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF


# def create_lka_steering(packer, apply_torque: int, torque_factor: int, status: int):
#   values = {
#     'TORQUE': apply_torque ,
#     # 'LANE_DEPARTURE':0 if not lat_active else 1 if torque>0 else 2,
#     # 'DRIVE': 1,
#     'STATUS': status,
#     # 'LXA_ACTIVATION': 1,
#     'TORQUE_FACTOR': torque_factor,
#     'SET_ANGLE': 0,
#   }

#   return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)


def create_lka_steering(packer, lat_active: bool, apply_torque: float, torque_factor: int, status: int):
  values = {
    'TORQUE': apply_torque,
    # 'LANE_DEPARTURE':0 if not lat_active else 1 if torque>0 else 2,
    # 'DRIVE': 1,
    'STATUS': status,
    # 'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': torque_factor, # * 100,
    'SET_ANGLE': 0,
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)


# def create_driver_torque(packer, steering):
#   # abs(driver_torque) > 10 to keep EPS engaged
#   torque = steering['DRIVER_TORQUE']

#   if abs(torque) < 10:
#     steering['DRIVER_TORQUE'] = 10 if torque > 0 else -10

#   return packer.make_can_msg('STEERING', 0, steering)

def create_driver_torque(packer, steering):
  t = int(steering.get('DRIVER_TORQUE', 0))
  if abs(t) < 10:
    t = random.randint(10, 12)
  t = max(0, min(20, t))
  steering['DRIVER_TORQUE'] = t
  return packer.make_can_msg('STEERING', 0, steering)



def create_steering_hold(packer, lat_active: bool, is_dat_dira):
  # set STEERWHL_HOLD_BY_DRV to keep EPS engaged when lat active
  if lat_active:
    is_dat_dira['STEERWHL_HOLD_BY_DRV'] = 1
  return packer.make_can_msg('IS_DAT_DIRA', 2, is_dat_dira)

def create_request_takeover(packer, HS2_DYN_MDD_ETAT_2F6, type):
  # HS2_DYN_MDD_ETAT_2F6
  #  1 = Non Critical Request
  #  2 = Critical request
  HS2_DYN_MDD_ETAT_2F6['REQUEST_TAKEOVER'] = type

  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, HS2_DYN_MDD_ETAT_2F6)

  # Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
  # Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
  # Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),

