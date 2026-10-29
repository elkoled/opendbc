from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'psa'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long

    # measured command to aEgo lag is 0.3 s driving and 0.4-0.7 s braking
    ret.longitudinalActuatorDelay = 0.4
    # integral covers drag the speed independent torque map misses at highway speed
    ret.longitudinalTuning.kiBP = [4., 13., 15., 20.]
    ret.longitudinalTuning.kiV = [0., 0.3, 0.5, 0.8]

    return ret