from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import AngleSteeringLimits, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

class CarControllerParams:
  STEER_STEP = 1  # spamming at 100 Hz works well, stock lkas is ~20 Hz

  # Angle rate limits are set to meet ISO 11270
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    390, # deg
    ([0., 5., 25.], [2.5, 1.5, 0.2]),
    ([0., 5., 25.], [5., 2.0, 0.3]),
  )
  STEER_DRIVER_ALLOWANCE = 10  # Driver intervention threshold, 1.0 Nm
  EPS_MAX_TORQUE = 4.5 # TODO: max EPS torque seen in Nm
  def __init__(self, CP):
    pass

@dataclass(frozen=True, kw_only=True)
class PSACarSpecs(CarSpecs):
  tireStiffnessFactor: float = 1.03

@dataclass
class PSACarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.psa_a]))

@dataclass
class PSAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'AEE2010_R3',
  })

class CAR(Platforms):
  PSA_OPEL_CORSA_F = PSAPlatformConfig(
    [PSACarDocs("Opel Corsa F")],
    PSACarSpecs(
      mass=1530,
      wheelbase=2.540,
      steerRatio=17.6,
      centerToFrontRatio=0.44,
    ),
  )

# Same as StdQueries.DEFAULT_DIAGNOSTIC_REQUEST
PSA_DIAGNOSTIC_REQUEST  = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, 0x01])
# The Diagnostic response includes the ECU rx/tx timings
PSA_DIAGNOSTIC_RESPONSE = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, 0x01])

PSA_SERIAL_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER,  0xF1, 0x8C])
PSA_SERIAL_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF1, 0x8C])

PSA_VERSION_REQUEST  = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER, 0xF0, 0xFE])
# TODO: Placeholder or info for uds module - The actual response is multi-frame TP
PSA_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF0, 0xFE])

PSA_RX_OFFSET = -0x20

# FW_QUERY_CONFIG = FwQueryConfig(
#   requests=[
#     # ──────────────────── ECU-serial number ────────────────────────
#     Request(
#       [StdQueries.DEFAULT_DIAGNOSTIC_REQUEST,   # 0x10 01
#        PSA_SERIAL_REQUEST],                     # 0x22 F1 8C
#       [StdQueries.DEFAULT_DIAGNOSTIC_RESPONSE,  # 0x50 01 …
#        PSA_SERIAL_RESPONSE],                    # 0x62 F1 8C …
#       rx_offset=PSA_RX_OFFSET,        # request 0x6B6  → reply 0x696
#       bus=1,                  # CAN-Comfort / ADAS harness line
#       logging=True,           # useful while bringing-up
#       obd_multiplexing=False, # don’t waste time toggling relays
#     ),

#     # ──────────────────── Software / calibration ID ───────────────
#     Request(
#       [StdQueries.DEFAULT_DIAGNOSTIC_REQUEST,   # 0x10 01
#        PSA_VERSION_REQUEST],                    # 0x22 F0 FE
#       [StdQueries.DEFAULT_DIAGNOSTIC_RESPONSE,  # 0x50 01 …
#        PSA_VERSION_RESPONSE],                   # 0x62 F0 FE …
#       rx_offset=PSA_RX_OFFSET,
#       bus=1,
#       logging=True,
#       obd_multiplexing=False,
#     ),
#   ],
# )

# TODO: multi-bus requests
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # iterate over bus 0/1/2 and both reply offsets (+8 and −0x20)
    request
    for bus in (0, 1, 2)
    for rx_off in (0x08, -0x20)
    for request in [

      # ── ECU serial number (DID 0xF1 8C) ────────────────────────────────
      Request(
        [StdQueries.DEFAULT_DIAGNOSTIC_REQUEST,
         PSA_SERIAL_REQUEST],
        [StdQueries.DEFAULT_DIAGNOSTIC_RESPONSE,
         PSA_SERIAL_RESPONSE],
        rx_offset=rx_off,
        bus=bus,
        logging=(bus == 1 and rx_off == -0x20),   # match your manual test
        obd_multiplexing=False,
      ),

      # ── ECU software / calibration version (DID 0xF0 FE) ───────────────
      Request(
        [StdQueries.DEFAULT_DIAGNOSTIC_REQUEST,
         PSA_VERSION_REQUEST],
        [StdQueries.DEFAULT_DIAGNOSTIC_RESPONSE,
         PSA_VERSION_RESPONSE],
        rx_offset=rx_off,
        bus=bus,
        logging=(bus == 1 and rx_off == -0x20),
        obd_multiplexing=False,
      ),

      # ── Manufacturer SW-number (standard DID 0xF1 88) ──────────────────
      Request(
        [StdQueries.DEFAULT_DIAGNOSTIC_REQUEST,
         StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
        [StdQueries.DEFAULT_DIAGNOSTIC_RESPONSE,
         StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
        rx_offset=rx_off,
        bus=bus,
        logging=False,
        obd_multiplexing=False,
      ),
    ]
  ],
)

DBC = CAR.create_dbc_map()
