from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.values import CAR, PSAFlags, PSASafetyFlags

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'psa'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]
    if ret.flags & PSAFlags.EPS_CAM_BUS:
      ret.safetyConfigs[0].safetyParam |= PSASafetyFlags.EPS_CAM_BUS.value

    ret.dashcamOnly = True

    # lagd learns 0.25-0.3 s on the 508 and 0.35-0.45 s on the 208
    ret.steerActuatorDelay = 0.1 if candidate == CAR.PSA_PEUGEOT_508 else 0.3
    ret.steerLimitTimer = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = False

    return ret
