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

    # only the electric cars send 0x202 on the ADAS bus
    if 0x202 in fingerprint[1]:
      ret.flags |= PSAFlags.EV.value

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

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= PSASafetyFlags.LONG_CONTROL.value

    # 0.3 s on torque, 0.4-0.7 s on the friction brakes
    ret.longitudinalActuatorDelay = 0.4
    # the integrator picks up drag and the launch, the torque map is speed independent
    ret.longitudinalTuning.kiBP = [0., 4., 13., 20.]
    # an EV lags its torque, more integrator overshoots every swing of the plan
    ret.longitudinalTuning.kiV = [0.18, 0.15, 0.1, 0.1] if ret.flags & PSAFlags.EV else [0.3, 0.4, 0.5, 0.8]

    return ret
