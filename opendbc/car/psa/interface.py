from opendbc.car import structs, get_safety_config
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.values import CAR, LKAS_LIMITS


TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'psa'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    #
    ret.dashcamOnly = False

    if candidate in (CAR.PSA_PEUGEOT_3008,):
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.steerControlType = structs.CarParams.SteerControlType.torque
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS
      ret.steerActuatorDelay = 0.376803
      ret.steerLimitTimer = 1
      ret.steerAtStandstill = False
    else:
      ret.steerAtStandstill = True

    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = False

    return ret