""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.psa.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.PSA_PEUGEOT_208: {
    (Ecu.abs, 0x6ad, None): [
      b'085095700857210527',
      b'085095706198220818',
      b'085095700473220908',
      b'085135705863191103',
      b'285160381930060025B',
      b'085095705963200902',
      b'085095709268220917',
      b'085135709971211107',
      b'085135702270200218',
    ],
  },
  CAR.PSA_PEUGEOT_508: {
    (Ecu.abs, 0x6ad, None): [
      b'085065315928191130',
      b'085095308910190312',
      b'085095701207240115',
      b'085135702688200218',
      b'085044414569231210',
      b'369042321125160806',
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
  },
}
