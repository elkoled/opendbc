def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x42D: 0xC}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF


def create_lka_steering(packer, lat_active: bool, apply_torque: float, status: int):
  values = {
    'TORQUE': apply_torque,
    # 'LANE_DEPARTURE':0 if not lat_active else 1 if torque>0 else 2,
    # 'DRIVE': 1,
    'STATUS': status,
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': 0,
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)


def create_driver_torque_spoof(packer, steering, min_torque=8):
  """
  Spoof DRIVER_TORQUE to keep EPS engaged indefinitely.

  Per PSA EPS LXA spec (ST-LXA-EPS-27):
  - MIN_DRIVER_TORQUE_DETECTION = 0.7 Nm
  - If |DRIVER_TORQUE| >= 0.7 Nm, STEERWHL_HOLD_BY_DRV = "steering activity"
  - This prevents FLAG_HOLD from becoming FALSE after DELAY_LXA_DEACTIVATION

  Raw values have resolution 0.1 Nm, so:
  - 0.7 Nm threshold = 7 raw units
  - Using 8 raw units (0.8 Nm) for margin

  Args:
    packer: CAN packer
    steering: Original STEERING message values from carstate
    min_torque: Minimum torque in raw units (default 8 = 0.8 Nm, above 0.7 threshold)
  """
  # Copy original values
  values = dict(steering)

  # Increment counter (4-bit, wraps at 16)
  values['COUNTER'] = (steering['COUNTER'] + 1) % 16

  # Ensure DRIVER_TORQUE is above MIN_DRIVER_TORQUE_DETECTION (0.7 Nm)
  # Keep the sign of the original torque for natural feel
  torque = steering['DRIVER_TORQUE']
  if abs(torque) < min_torque:
    values['DRIVER_TORQUE'] = min_torque if torque >= 0 else -min_torque

  # Packer will compute checksum automatically via psa_checksum
  return packer.make_can_msg('STEERING', 0, values)


def create_steering_hold(packer, lat_active: bool, is_dat_dira):
  # set STEERWHL_HOLD_BY_DRV to keep EPS engaged when lat active
  if lat_active:
    is_dat_dira['STEERWHL_HOLD_BY_DRV'] = 1
  return packer.make_can_msg('IS_DAT_DIRA', 2, is_dat_dira)
