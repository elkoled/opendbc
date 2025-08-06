def calculate_checksum(dat: bytearray, chk_ini: int) -> int:
  checksum = sum((b >> 4) + (b & 0xF) for b in dat)
  return (chk_ini - checksum) & 0xF


def create_lka_steering(packer, frame: int, lat_active: bool, apply_angle: float, eps_active: bool = True):
  if not hasattr(create_lka_steering, 's'):
    create_lka_steering.s = 2

  if not lat_active:
    create_lka_steering.s = 2
  elif not eps_active:
    create_lka_steering.s = 2 if create_lka_steering.s == 4 else create_lka_steering.s + 1
  else:
    create_lka_steering.s = 4

  values = {
    'DRIVE': 1,
    'COUNTER': frame % 0x10,
    'CHECKSUM': 0,
    # STATUS needs a sequence of 2->3->4 to engage steering. On steering override, cycle 2->3->4->2... until eps is active again
    # 0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
    'STATUS': create_lka_steering.s,
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
  }

  msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[1]
  values['CHECKSUM'] = calculate_checksum(msg, 0xB)

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
