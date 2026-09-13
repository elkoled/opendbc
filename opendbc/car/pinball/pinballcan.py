from opendbc.car.can_definitions import CanData


def crc8(data):
  crc = 0xff
  for value in data:
    crc ^= value
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1d) & 0xff if crc & 0x80 else (crc << 1) & 0xff
  return crc ^ 0xff


def create_solenoid_cmd(_packer, left, right, start, sequence=0):
  state = int(bool(left)) | (int(bool(right)) << 1) | (int(bool(start)) << 2)
  payload = bytes((0x50, 0x42, 1, sequence & 0xff, state, state ^ 0xff, 0))
  return CanData(0x200, payload + bytes((crc8(payload),)), 0)
