""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.structs import CarParams
from opendbc.car.psa.values import CAR

Ecu = CarParams.Ecu

FW_VERSIONS = {
  CAR.PSA_PEUGEOT_208: {
    # ARTIV - Radar
    (Ecu.fwdRadar, 0x6B6, None): [
        b'212053276', # Peugeot e208 Allure Pack 2021
        b'194504751', # Peugeot e208 GT 2020
        b'222256113', # Peugeot e208 GT NZ 2022
    ],
  },
}
