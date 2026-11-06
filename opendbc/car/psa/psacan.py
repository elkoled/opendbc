from enum import IntEnum

from opendbc.car.can_definitions import CanData


def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x42D: 0xC, 0x2B6: 0xC, 0x2F6: 0x8}.get(address, 0xB)
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


class AccMode(IntEnum):
  TORQUE = 0  # engine torque
  BRAKE = 1  # friction brakes
  HOLD = 2  # standstill hold
  LAUNCH = 3  # drive off from the hold


# what the radar sends when it requests nothing
INACTIVE_DECEL = 2.05  # m/s^2
INACTIVE_TORQUE = -4000  # Nm
# the ESP holds the car on the lowest decel, the stock radar always opens the launch with the highest one
HOLD_DECEL = -10.65  # m/s^2
LAUNCH_DECEL = 2.0  # m/s^2


def create_HS2_DYN1_MDD_ETAT_2B6(packer, active: bool, override: bool, inhibited: bool, mode: AccMode, accel: float, torque: float):
  # ACC_STATUS 2: inhibited, 3: standby, 4: active, 5: suspended by gas
  # the car shows an ACC fault when the driver turns ACC on and it stays in standby
  values = {
    'ACC_STATUS': 4 if active else 5 if override else 2 if inhibited else 3,
    'AUTO_BRAKING_STATUS': 3,
    'MDD_DESIRED_DECELERATION': INACTIVE_DECEL,
    'GMP_POTENTIAL_WHEEL_TORQUE': INACTIVE_TORQUE,
    'GMP_WHEEL_TORQUE': INACTIVE_TORQUE,
  }

  # all fields change in the same frame, a partial hand over faults the ESP
  if active:
    braking = mode in (AccMode.BRAKE, AccMode.HOLD)
    torque_req = mode in (AccMode.TORQUE, AccMode.LAUNCH)
    decel_req = mode != AccMode.TORQUE
    values.update({
      'POTENTIAL_WHEEL_TORQUE_REQUEST': 2 if braking else 1,
      'MIN_TIME_FOR_DESIRED_GEAR': 0.0 if braking else 6.2,
      'GMP_POTENTIAL_WHEEL_TORQUE': INACTIVE_TORQUE if braking else torque,
      'GMP_WHEEL_TORQUE': torque if torque_req else INACTIVE_TORQUE,
      'WHEEL_TORQUE_REQUEST': torque_req,
      'MDD_DECEL_TYPE': decel_req,
      'MDD_DECEL_CONTROL_REQ': decel_req,
    })
    if decel_req:
      values['MDD_DESIRED_DECELERATION'] = {AccMode.BRAKE: accel, AccMode.HOLD: HOLD_DECEL, AccMode.LAUNCH: LAUNCH_DECEL}[mode]

  return packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', 1, values)


def create_HS2_DYN_MDD_ETAT_2F6(packer, decel_req: bool, lead_visible: bool, lead_distance_bars: int):
  values = {
    'TARGET_DETECTED': lead_visible,
    'DISPLAY_INTERVEHICLE_TIME': 5.0,
    'MDD_DECEL_CONTROL_REQ': decel_req,
    # 1: near to 3: far, 4: no lead
    'TARGET_POSITION': lead_distance_bars if lead_visible else 4,
  }

  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, values)


# the radar stays silent in its programming session while it gets tester present
def create_disable_radar():
  return CanData(0x6B6, bytes([0x02, 0x10, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00]), 1)
