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

  ACC_STEP = 2  # radar messages are sent at 50 Hz
  ACCEL_MAX = 2.0  # m/s^2
  ACCEL_MIN = -3.5  # m/s^2

  # engine braking runs out around -0.3 m/s^2, the stock radar uses the friction brakes from there
  BRAKE_ON_ACCEL = -0.25  # m/s^2
  BRAKE_OFF_ACCEL = -0.1  # m/s^2

  # wheel torque for the pitch compensated accel, from 208 and Corsa F drives
  TORQUE_LOOKUP_BP = [-1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 3.0]  # m/s^2
  TORQUE_LOOKUP_V = [-400, -140, 82, 375, 615, 865, 1100, 1600]  # Nm
  # from e-208 drives, close to what its stock radar sends
  TORQUE_LOOKUP_V_EV = [-480, -248, -15, 218, 450, 682, 915, 1380]  # Nm
  TORQUE_MAX = 1500  # Nm, the stock radar asks up to ~1480 on steep hills
  TORQUE_MIN = -400  # Nm
  EV_LIFT_OFF_TORQUE_MIN = 0  # Nm
  # the motor answers within a frame, steps in the request are felt as jerk
  EV_TORQUE_RATE = 20  # Nm per radar frame
  # an EV regens to a negative torque request, on the brakes the motor regens fully, so brake only past the regen
  EV_BRAKE_ON_ACCEL = -0.45  # m/s^2
  EV_BRAKE_OFF_ACCEL = -0.25  # m/s^2


class EpsState(IntEnum):
  UNAUTHORIZED = 0
  AUTHORIZED = 1
  AVAILABLE = 2
  ACTIVE = 3
  DEFECT = 4


class PSASafetyFlags(IntFlag):
  LONG_CONTROL = 1
  EPS_CAM_BUS = 2


class PSAFlags(IntFlag):
  # the EPS is on the camera side of the harness
  EPS_CAM_BUS = 1
  # the motor holds the speed with no torque and needs less of it than an engine
  EV = 2


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
