from opendbc.car.common.numpy_fast import clip
from opendbc.car import CanBusBase

class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)
    self.ramp_value = 0

  @property
  def main(self) -> int:
    return self.offset

  @property
  def adas(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset + 2

def calculate_checksum(dat: bytearray) -> int:
    checksum = 0
    for i, b in enumerate(dat):
        high_nibble = b >> 4
        low_nibble = b & 0xF
        checksum += high_nibble + low_nibble
    # find CHK so that (checksum + CHK) % 16 = 11 (0xB)
    needed = (11 - checksum) & 0xF
    return needed

def create_lka_msg(packer, CP, apply_steer: float, frame: int, lat_active: bool, max_torque: int, reverse: bool):
    # TODO: hud control for lane departure, status
    # TODO: unknowns could be recuperation mode/drive mode
    # TODO find better way to handle this
    max_torque = max_torque if lat_active else 0
    # ramp up/down
    CanBus(CP).ramp_value = max(min(CanBus(CP).ramp_value + (1 if lat_active else -1), 100), 0)

    values = {
        'unknown1': 0 if reverse else 1,
        'COUNTER': (frame//5) % 0x10,
        'CHECKSUM': 0,
        'unknown2': min(frame // 100, 0x0B), # TODO: check, currently ramps up 1/s up to 0x0B
        'TORQUE': max_torque if lat_active else 0,
        'LANE_DEPARTURE': 2 if max_torque < 0 else 1 if max_torque > 0 else 0, # TODO: check sign
        'LKA_DENY': 0 if lat_active else 1,
        'STATUS': 2 if lat_active else 1,
        'unknown3': 0, # TODO check
        'RAMP': CanBus(CP).ramp_value, # TODO check, currently ramps up 100/s
        'ANGLE': clip(apply_steer, -90, 90),
        'unknown4': 1,
    }

    msg = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
    dat = msg[1]
    if isinstance(dat, int):
        dat = dat.to_bytes(1, 'big')

    values['CHECKSUM'] = calculate_checksum(dat)

    return packer.make_can_msg('LANE_KEEP_ASSIST', CanBus(CP).main, values)
