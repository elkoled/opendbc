from opendbc.car.structs import CarParams
from opendbc.car.psa.values import CAR

Ecu = CarParams.Ecu

FINGERPRINTS = {
  CAR.PSA_PEUGEOT_208: [
    # Peugeot 508
    {1150: 8, 1298: 4, 841: 8, 610: 1, 1370: 7, 1946: 8, 747: 5, 973: 8, 757: 7, 845: 8, 1303: 7, 941: 8, 749: 7, 773: 7, 114: 5, 520: 8, 602: 2, 1101: 8, 1338: 3, 820: 8, 1416: 8, 1018: 2, 1306: 6, 939: 4, 781: 8, 1037: 8, 877: 4, 1422: 8, 166: 5, 840: 8, 1274: 4, 1099: 8, 1272: 5, 1006: 4, 1074: 8, 1257: 4, 909: 8, 1035: 4, 1054: 8, 1067: 8, 1304: 4, 1387: 4, 851: 4, 1038: 5, 1186: 2, 1128: 8, 1010: 8, 1161: 5, 1390: 7, 1042: 8, 1278: 3, 1293: 8, 1086: 6, 1118: 6, 169: 5, 1554: 8, 850: 2, 1173: 6, 1394: 8, 1454: 5, 1522: 5, 1432: 4, 488: 8, 936: 6, 1528: 5, 1324: 1, 1928: 8, 1960: 4, 1430: 4, 1458: 8, 174: 5, 1943: 6, 1486: 3, 1835: 6, 974: 8, 1938: 8, 1507: 8, 168: 5, 1230: 8, 1326: 8, 1502: 8, 1358: 8, 1347: 6, 1922: 8, 1506: 8, 1442: 8, 1473: 8, 1379: 4, 1475: 8, 1362: 8, 1515: 8, 1102: 3, 1933: 8, 1929: 8, 1483: 8, 1012: 8, 1076: 8, 1108: 8, 1172: 8, 1443: 8, 1411: 7, 1941: 6}
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
