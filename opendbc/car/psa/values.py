from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, uds

Ecu = CarParams.Ecu


class CarControllerParams:
  # TODO: tune these params
  STEER_MAX = 50  # TODO: find max torque
  # STEER_MAX_LOOKUP = [9, 17], [200, 100]
  STEER_STEP = 1
  STEER_DELTA_UP = 5  # TODO: torque increase per refresh
  STEER_DELTA_DOWN = 5  # TODO: torque decrease per refresh
  STEER_DRIVER_MULTIPLIER = 1  # TODO: weight driver torque
  STEER_DRIVER_FACTOR = 1
  STEER_DRIVER_ALLOWANCE = 5  # TODO: tune Driver intervention threshold

  def __init__(self, CP):
    pass


@dataclass
class PSACarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC) & Lane Assist"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.psa_a]))


@dataclass
class PSAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'psa_aee2010_r3',
  })


class CAR(Platforms):
  PSA_PEUGEOT_208 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 208 2019-25")],
    CarSpecs(mass=1530, wheelbase=2.73, steerRatio=14.0), # TODO: these are set to live learned Berlingo values
  )


# KWP2000
PSA_KWP_START_REQ_R2    = bytes([0x81])
PSA_KWP_START_RESP_R2   = bytes([0xC1])  # 0x81 + 0x40

PSA_KWP_SERIAL_REQ_R2   = bytes([0x21, 0x80])
PSA_KWP_SERIAL_RESP_R2  = bytes([0x61, 0x80])  # 0x21 + 0x40, ID 0x80

PSA_KWP_VERSION_REQ_R2  = bytes([0x21, 0x87, 0x00])
PSA_KWP_VERSION_RESP_R2 = bytes([0x61, 0x87, 0x00])  # 0x21 + 0x40, ID 0x8700

# AEE2010 R2
PSA_DIAG_REQ_R2  = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.EXTENDED_DIAGNOSTIC])
PSA_DIAG_RESP_R2 = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, 0x03])

PSA_SERIAL_REQ_R2 = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER,  0xF0, 0x80])
PSA_SERIAL_RESP_R2 = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF0, 0x80])

# AEE2010 R3
PSA_DIAG_REQ_R3  = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, uds.SESSION_TYPE.DEFAULT])
PSA_DIAG_RESP_R3 = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, 0x01])

PSA_SERIAL_REQ_R3 = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER,  0xF1, 0x8C])
PSA_SERIAL_RESP_R3 = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF1, 0x8C])

PSA_VERSION_REQ_R3  = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER, 0xF0, 0xFE])
PSA_VERSION_RESP_R3 = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF0, 0xFE])

PSA_RX_OFFSET = -0x20

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[request for bus in (0, 1, 2) for request in [
    Request(
      [PSA_DIAG_REQ_R3, PSA_SERIAL_REQ_R3],
      [PSA_DIAG_RESP_R3, PSA_SERIAL_RESP_R3],
      rx_offset=PSA_RX_OFFSET,
      bus=bus,
      obd_multiplexing=False,
    ),
    Request(
      [PSA_DIAG_REQ_R3, PSA_VERSION_REQ_R3],
      [PSA_DIAG_RESP_R3, PSA_VERSION_RESP_R3],
      rx_offset=PSA_RX_OFFSET,
      bus=bus,
      obd_multiplexing=False,
    ),
    Request(
      [PSA_DIAG_REQ_R2, PSA_SERIAL_REQ_R2],
      [PSA_DIAG_RESP_R2, PSA_SERIAL_RESP_R2],
      rx_offset=PSA_RX_OFFSET,
      bus=bus,
      obd_multiplexing=False,
    ),
    Request(
      [PSA_DIAG_REQ_R2, PSA_SERIAL_REQ_R3],
      [PSA_DIAG_RESP_R2, PSA_SERIAL_RESP_R3],
      rx_offset=PSA_RX_OFFSET,
      bus=bus,
      obd_multiplexing=False,
    ),
    Request(
      [PSA_KWP_START_REQ_R2, PSA_KWP_SERIAL_REQ_R2, PSA_KWP_VERSION_REQ_R2],
      [PSA_KWP_START_RESP_R2, PSA_KWP_SERIAL_RESP_R2, PSA_KWP_VERSION_RESP_R2],
      rx_offset=PSA_RX_OFFSET,
      bus=bus,
      obd_multiplexing=False,
    )
  ]]
)

DBC = CAR.create_dbc_map()
