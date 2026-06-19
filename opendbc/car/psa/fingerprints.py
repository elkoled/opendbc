from opendbc.car.structs import CarParams
from opendbc.car.psa.values import CAR

Ecu = CarParams.Ecu

# Peugeot ECU list https://github.com/ludwig-v/arduino-psa-diag/blob/master/ECU_LIST.md

FW_VERSIONS = {
  CAR.PSA_PEUGEOT_208: {
    # ARTIV - Radar
    (Ecu.fwdRadar, 0x6B6, None): [
        b'212053276', # Peugeot e208 Allure Pack
        b'194504751', # Peugeot e208 GT CC-only
        b'222256113', # Peugeot e208 GT NZ
    ],
  },
  CAR.PSA_PEUGEOT_508: {
    # ARTIV - Radar
    (Ecu.fwdRadar, 0x6B6, None): [
        b'200603842', # Peugeot 508 Hybrid
    ],
  },

  CAR.PSA_PEUGEOT_3008: {
    # ARTIV - Front Radar ADAS
    (Ecu.fwdRadar, 0x6B6, None): [ ],

    # DIRECTN - Electronic power steering
    (Ecu.eps, 0x6B5, None): [ ],

    # HCU2 - Hybrid Control Unit
    (Ecu.hybrid, 0x6A6, None): [  ],

    # BOITEVIT - Automatic transmission  (EAT6/8)
    (Ecu.transmission, 0x6A9, None): [
        ## Peugeot 3008 II (Phase I, 2016) 1.6 PureTech (180 Hp) Automatic S&S /2018, 2019, 2020
        b'1614191B101502000000',
        b'\xff\xff\x00\x000`\x08\x01\x13\x01%\x06\x08\xff\xff\xff\x00\x02\x00\x00\x01\x934t',
        #
    ],

    # FREINEBB - Electronic Brake Booster
    (Ecu.electricBrakeBooster, 0x5D0, None): [],

    # INJ - Engine (VCU)
    (Ecu.engine, 0x6A8, None): [
        ## Peugeot 3008 II (Phase I, 2016) 1.6 PureTech (180 Hp) Automatic S&S /2018, 2019, 2020
        b'000D170047100',
        b'\xff\xff\x00\x00\x03&\t\x02\x13\x01C1\x04\xff\xff\xff\x00\x02\x00\x00\x01\x93YW',
        #
    ],

    # ABRASR - ABS/ESP
    (Ecu.abs, 0x6AD, None): [
        ## Peugeot 3008 II (Phase I, 2016) 1.6 PureTech (180 Hp) Automatic S&S /2018, 2019, 2020
        b'085065201906190129',
        b'\x00\x00\x00\x00\x03\x92\x01\x06\x11\x01\x06\x18\x02\xff\xff\xff\x00\x02\x00\x00\x01\x92\x83\x12',
        #
    ],
  }
}


