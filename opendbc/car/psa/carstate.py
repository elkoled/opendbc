from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.values import DBC, CarControllerParams
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.cam]
    cp_adas = can_parsers[Bus.adas]
    cp_main = can_parsers[Bus.main]
    ret = structs.CarState()

    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'],
      cp.vl['Dyn4_FRE']['P264_VehV_VPsvValWhlFrtR'],
      cp.vl['Dyn4_FRE']['P265_VehV_VPsvValWhlBckL'],
      cp.vl['Dyn4_FRE']['P266_VehV_VPsvValWhlBckR'],
    )
    ret.vEgoRaw = cp_adas.vl['HS2_DYN_ABR_38D']['VITESSE_VEHICULE_ROUES'] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.yawRate = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD
    ret.standstill = ret.vEgo < 0.1

    # gas
    ret.gasPressed = (cp.vl['Dyn_CMM']['P002_Com_rAPP'] / 100.0) > 0

    # brake
    ret.brake = cp.vl['Dyn2_FRE']['BRAKE_PRESSURE'] / 1500.
    ret.brakePressed = bool(cp_main.vl['Dat_BSI']['P013_MainBrake'])
    ret.parkingBrake = cp.vl['Dyn_EasyMove']['P337_Com_stPrkBrk'] == 1 # 0: disengaged, 1: engaged, 3: brake actuator moving

    # steering wheel
    ret.steeringAngleDeg = cp.vl['STEERING_ALT']['ANGLE'] # EPS
    ret.steeringRateDeg = cp.vl['STEERING_ALT']['RATE'] * (2 * cp.vl['STEERING_ALT']['RATE_SIGN'] - 1) # convert [0,1] to [-1,1] EPS: rot. speed * rot. sign
    ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
    ret.steeringTorqueEps = cp.vl['IS_DAT_DIRA']['EPS_TORQUE']
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    ret.steerFaultTemporary = False # cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA'] == 4
    ret.steerFaultPermanent = False
    ret.espDisabled = False

    # cruise
    ret.cruiseState.speed = cp_adas.vl['HS2_DAT_MDD_CMD_452']['SPEED_SETPOINT'] * CV.KPH_TO_MS # set to 255 when ACC is off, -2 kph offset from dash speed
    ret.cruiseState.enabled = cp_adas.vl['HS2_DAT_MDD_CMD_452']['RVV_ACC_ACTIVATION_REQ'] == 1
    ret.cruiseState.available = True # not available for CC-only
    ret.cruiseState.nonAdaptive = False # not available for CC-only
    ret.cruiseState.standstill = False # not available for CC-only
    ret.accFaulted = False # not available for CC-only

    # gear
    if self.CP.transmissionType == TransmissionType.manual:
      ret.clutchPressed = bool(cp.vl['Dyn2_CMM']['P091_ConvCD_stDebVal'])
    if bool(cp_main.vl['Dat_BSI']['P103_Com_bRevGear']):
      ret.gearShifter = GearShifter.reverse
    else:
      ret.gearShifter = GearShifter.drive

    # blinkers
    blinker = cp_main.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC']
    ret.leftBlinker = blinker == 1
    ret.rightBlinker = blinker == 2

    # lock info
    ret.doorOpen = any([cp_main.vl['Dat_BSI']['DRIVER_DOOR'], cp_main.vl['Dat_BSI']['PASSENGER_DOOR']])
    ret.seatbeltUnlatched = cp_main.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2
    return ret

  # TODO: reset frequencies
  @staticmethod
  def get_can_parsers(CP):
    cam_messages = [
      ('Dyn4_FRE', 1),
      ('STEERING_ALT', 1),
      ('STEERING', 1),
      ('Dyn2_FRE', 1),
      ('Dyn2_CMM', 1),
      ('Dyn_CMM', 1),
      ('Dyn_EasyMove', 1),
      ('IS_DAT_DIRA', 1),
    ]
    adas_messages = [
      ('HS2_DYN_ABR_38D', 1),
      ('HS2_DYN_UCF_MDD_32D', 1),
      ('HS2_DAT_MDD_CMD_452', 1),
    ]
    main_messages = [
      ('Dat_BSI', 1),
      ('RESTRAINTS', 1),
      ('HS2_DAT7_BSI_612', 1),
    ]
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], main_messages, 2),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], adas_messages, 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 0),
    }
