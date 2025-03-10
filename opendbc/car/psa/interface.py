from opendbc.car import structs, uds
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.values import CAR
# from opendbc.car.disable_ecu import disable_ecu
from opendbc.car.psa.disable_radar import implement_best_disable_strategy

TransmissionType = structs.CarParams.TransmissionType

class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret: structs.CarParams, candidate: CAR, fingerprint, car_fw, experimental_long, docs):
    ret.brand = 'psa'
    ret.dashcamOnly = False

    ret.radarUnavailable = True
    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 1.0

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    if not docs:
      ret.transmissionType = TransmissionType.automatic
      ret.minEnableSpeed = 0
    ret.minSteerSpeed = 0.

    ret.autoResumeSng = ret.minEnableSpeed == -1
    ret.centerToFront = ret.wheelbase * 0.44
    ret.wheelSpeedFactor = 1.04

    return ret

  # TODO: find radar ECU address to disable it, check sub_addr with panda script
  @staticmethod
  def init(CP, can_recv, can_send):
    # ARTIV	ARTIV, RADAR_AV_4, LIDAR, ARTIV_UDS	>6B6:696
    implement_best_disable_strategy(can_recv, can_send)