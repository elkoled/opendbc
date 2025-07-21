# ruff: noqa: E501
from opendbc.car.structs import CarParams
from opendbc.car.psa.values import CAR

Ecu = CarParams.Ecu

FINGERPRINTS = {
  CAR.PSA_PEUGEOT_208: [
  ],
}

FW_VERSIONS = {
  CAR.PSA_PEUGEOT_208: {
    # ARTIV - Radar
    (Ecu.fwdRadar, 0x6B6, None): [
        b'212053276', # e208 Allure ACC
        b'194504751', # e208 GT no ACC
        b'222256113', # NZ e208 GT ACC
        b'\xff\xff\x00\x00\x0f\xe8\x18\x05!@Y\xa4\x03\xff\xff\xff\x00\x02\x00\x00\x01\x94\x80\x97', # e208 Allure ACC
        b'\xff\xff\x00\x00\x0f\xe8\x07\x11\x19@Y\xa2\x03\xff\xff\xff\x00\x02\x00\x00\x01\x94 )', # e208 GT no ACC
    ],
    # DIRECTN - Electronic Power Steering
    (Ecu.eps, 0x6B5, None): [
        b'6077GC0817309',
        b'6077GC0165130',
        b'6077BD1235691',
        b'\xbfP\x00\x00\x13j\x07\x06\x15\xb5@\xf5\x03\xff\xff\xff\x00\x02\x00\x00\x01\x944g',
        b'p]\x00\x00\x13j\x1c\x0b\x13\xb5A\xf5\x02\x10\x07\x15\xfd\xd4S\xe2\x02\x944h',
    ],
    # HCU2 - Hybrid Control Unit
    (Ecu.hybrid, 0x6A6, None): [
        b'210306062100',
        b'191030025300',
        b'220609009900',
        b'\xff\xff\x00\x00\r\n\x06\x03!\x03\x01\x12\x01\xff\xff\xff\x00\x02\x00\x00\x02\x94\x86b',
        b'\xff\xff\x00\x00\r\n0\x10\x19\x02\x01!\x01\x05\x12$\xfd\xd4S\xe2\x05\x978\x13',
    ],
    # MSB - Electronic Brake Booster
    (Ecu.electricBrakeBooster, 0x6B4, None): [
        b'521021900860',
        b'419396900392',
        b'522276900847',
        b'\xff\xff\x00\x00t\x01\x11\x01!\x01\x040\x15\xff\xff\xff\x00\x02\x00\x00\xfe\x95\x08w',
        b'\xff\xff\x00\x00t\x01(\t\x19\x01\x01\x82\t\x05\x12$\xfd\xd4S\xe2\x05\x975\t',
    ],
    # VCU - Vehicle Control Unit
    (Ecu.engine, 0x6A2, None): [
        b'9210126909',
        b'9290984353',
        b'9221072645',
        b'\xf2i\x00\x00\r\x99\x11\x05\x15\x01!\xb2\x01!\x07#\xfd\xd4S\xe2\x02\x96E ',
        b'\xeb\xec\x00\x00\r\x99\x0f\x0b\x13\x01\x12\xb2\x01\x14\x12#\xfd\xd4S\xe2\x04\x96D\x93',
    ],
    # ABRASR - ABS/ESP
    (Ecu.abs, 0x6AD, None): [
        b'085095700857210527',
        b'085135705863191103',
        b'085095706198220818',
        b'\x00\x00\x00\x00\x03\x93!\x08 \x01\xc2\x12\x12\xff\xff\xff\x00\x02\x00\x00\x01\x94',
        b'\x00\x00\x00\x00\x03\x93\x15\x07\x19\x01\xc0\x0c!\x10\x07\x15\xfd\xd4S\xe2\x02\x94iF',
    ],
  }
}
