from dataclasses import dataclass, field
from enum import IntEnum, IntFlag

from opendbc.car.structs import CarParams
from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimitsVM
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, uds

Ecu = CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 1
  STATUS_STEP = 5  # frames per step of the EPS activation sequence

  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    390,  # deg
    # the EPS drops out from ~3.1 m/s^2
    MAX_LATERAL_ACCEL=3.0,  # m/s^2
    # 75 deg/s, the EPS faults from ~90 deg/s at low speed
    MAX_ANGLE_RATE=0.75,  # deg/frame
  )
  STEER_DRIVER_ALLOWANCE = 0.5  # Driver intervention threshold, Nm


class EpsState(IntEnum):
  UNAUTHORIZED = 0
  AUTHORIZED = 1
  AVAILABLE = 2
  ACTIVE = 3
  DEFECT = 4


class PSASafetyFlags(IntFlag):
  EPS_CAM_BUS = 2


class PSAFlags(IntFlag):
  # the EPS is on the camera side of the harness
  EPS_CAM_BUS = 1


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
    CarSpecs(wheelbase=2.54, steerRatio=17.6),
  )
  PSA_PEUGEOT_508 = PSAPlatformConfig(
    [PSACarDocs("Peugeot 508 2019-23")],
    CarSpecs(wheelbase=2.79, steerRatio=17.6),
    flags=PSAFlags.EPS_CAM_BUS,
  )


# ABS serial number
PSA_DIAG_REQ = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, 0x01])
PSA_DIAG_RESP = bytes([uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40, 0x01])
PSA_SERIAL_REQ = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER, 0xF1, 0x8C])
PSA_SERIAL_RESP = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40, 0xF1, 0x8C])

FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"\d{18}[0-9A-Z]?",
  requests=[
    Request(
      [PSA_DIAG_REQ, PSA_SERIAL_REQ],
      [PSA_DIAG_RESP, PSA_SERIAL_RESP],
      rx_offset=-0x20,
      bus=0,
      obd_multiplexing=False,
    ),
  ],
)

DBC = CAR.create_dbc_map()
