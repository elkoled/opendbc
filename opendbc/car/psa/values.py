from dataclasses import dataclass, field

from opendbc.car.structs import CarParams
from opendbc.car import AngleRateLimit, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu

class CarControllerParams:
  STEER_STEP = 1  # spamming at 100 Hz works well, stock lkas is ~20 Hz

  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[20., 3.2, .6]) # TODO: refine limits
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[20., 14., 1.6]) # TODO: refine limits
  STEER_DRIVER_ALLOWANCE = 10  # Driver intervention threshold, 1 Nm
  EPS_MAX_TORQUE = 4 # TODO: tune   max torque of EPS in Nm

  # LKAS CC
  STEER_MAX = 100  # TODO: LKAS CC
  STEER_DRIVER_FACTOR = 1 # TODO: LKAS CC
  STEER_DRIVER_MULTIPLIER = 1 # TODO: LKAS CC
  STEER_DELTA_UP = 10 # TODO: LKAS CC
  STEER_DELTA_DOWN = 10 # TODO: LKAS CC

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
    Bus.cam: 'AEE2010_R3',
    Bus.adas: 'AEE2010_R3',
    Bus.main: 'AEE2010_R3',
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

PSA_RX_OFFSET = -20

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
      logging=True,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      rx_offset=PSA_RX_OFFSET,
      bus=0,
      logging=True,
    ),
  ],
)

DBC = CAR.create_dbc_map()
