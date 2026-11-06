from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.psa.values import DBC, CarControllerParams, EpsState, PSAFlags
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter

# the radar sends at 50 Hz with up to 35 ms of jitter
RADAR_TIMEOUT_FRAMES = 6
# the CMM briefly drops the ACC torque when switching to the brakes
ACC_TORQUE_LOST_FRAMES = 100

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.radar_alive = False
    self.radar_ts_last = 0
    self.radar_stale_frames = 0
    self.acc_torque_lost_frames = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.main]
    cp_adas = can_parsers[Bus.adas]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # car speed
    self.parse_wheel_speeds(ret,
      cp.vl['Dyn4_FRE']['P263_VehV_VPsvValWhlFrtL'],
      cp.vl['Dyn4_FRE']['P264_VehV_VPsvValWhlFrtR'],
      cp.vl['Dyn4_FRE']['P265_VehV_VPsvValWhlBckL'],
      cp.vl['Dyn4_FRE']['P266_VehV_VPsvValWhlBckR'],
    )
    ret.yawRate = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VITESSE_LACET_BRUTE'] * CV.DEG_TO_RAD
    ret.standstill = bool(cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VEHICLE_STANDSTILL'])

    # gas, the pedal in Dyn_CMM also shows the ACC torque request
    ret.gasPressed = cp_cam.vl['DRIVER']['GAS_PEDAL'] > 0

    # brake
    ret.brakePressed = bool(cp_cam.vl['Dat_BSI']['P013_MainBrake'])
    ret.parkingBrake = cp.vl['Dyn_EasyMove']['P337_Com_stPrkBrk'] == 1 # 0: disengaged, 1: engaged, 3: brake actuator moving

    # steering wheel
    cp_eps = cp_cam if self.CP.flags & PSAFlags.EPS_CAM_BUS else cp
    ret.steeringAngleDeg = cp_eps.vl['STEERING_ALT']['ANGLE']  # EPS
    ret.steeringRateDeg = cp_eps.vl['STEERING_ALT']['RATE'] * (1 - 2 * cp_eps.vl['STEERING_ALT']['RATE_SIGN']) # RATE_SIGN [0,1] to [1,-1]
    ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    self.eps_state = cp.vl['IS_DAT_DIRA']['EPS_STATE_LKA']
    self.eps_active = self.eps_state == EpsState.ACTIVE
    ret.steerFaultTemporary = self.eps_state == EpsState.DEFECT

    # cruise
    ret.cruiseState.speed = cp_adas.vl['HS2_DAT_MDD_CMD_452']['SPEED_SETPOINT'] * CV.KPH_TO_MS # set to 255 when ACC is off, -2 kph offset from dash speed
    ret.cruiseState.enabled = cp_adas.vl['HS2_DAT_MDD_CMD_452']['RVV_ACC_ACTIVATION_REQ'] == 1
    ret.cruiseState.nonAdaptive = cp_adas.vl['HS2_DAT_MDD_CMD_452']['LONGITUDINAL_REGULATION_TYPE'] != 3 # 0: None, 1: CC, 2: Limiter, 3: ACC
    if self.CP.openpilotLongitudinalControl:
      # the radar is silent with openpilot long, so its ACC state is not available
      ret.cruiseState.available = True
    else:
      ret.cruiseState.available = cp_adas.vl['HS2_DYN1_MDD_ETAT_2B6']['ACC_STATUS'] > 2
      ret.cruiseState.standstill = bool(cp_adas.vl['HS2_DYN_UCF_MDD_32D']['VEHICLE_STANDSTILL'])
    ret.accFaulted = cp_adas.vl['HS2_DYN_UCF_MDD_32D']['ACC_ETAT_DECEL_OR_ESP_STATUS'] == 3 # 0: Inhibited, 1: Waiting, 2: Active, 3: Fault

    # our radar messages only come back as TX, a new one is from the stock radar
    radar_ts = cp_adas.ts_nanos['HS2_DYN1_MDD_ETAT_2B6']['COUNTER']
    self.radar_stale_frames = 0 if radar_ts != self.radar_ts_last else self.radar_stale_frames + 1
    self.radar_ts_last = radar_ts
    self.radar_alive = radar_ts != 0 and self.radar_stale_frames < RADAR_TIMEOUT_FRAMES

    # the CMM can stop taking our torque for the rest of the drive, it drops this before the driver's gas press shows up
    gas_cmm = cp.vl['Dyn_CMM']['P002_Com_rAPP'] > 0
    acc_torque_lost = (self.CP.openpilotLongitudinalControl and not self.radar_alive and ret.cruiseState.enabled and not (ret.gasPressed or gas_cmm) and
                       cp_adas.vl['NEW_MSG_318']['ACC_GO_RELATED'] == 0)
    self.acc_torque_lost_frames = self.acc_torque_lost_frames + 1 if acc_torque_lost else 0
    ret.accFaulted = ret.accFaulted or self.acc_torque_lost_frames > ACC_TORQUE_LOST_FRAMES

    # gear
    if bool(cp_cam.vl['Dat_BSI']['P103_Com_bRevGear']):
      ret.gearShifter = GearShifter.reverse
    else:
      ret.gearShifter = GearShifter.drive

    # blinkers
    blinker = cp_cam.vl['HS2_DAT7_BSI_612']['CDE_CLG_ET_HDC']
    ret.leftBlinker = blinker == 1
    ret.rightBlinker = blinker == 2

    # lock info
    ret.doorOpen = any((cp_cam.vl['Dat_BSI']['DRIVER_DOOR'], cp_cam.vl['Dat_BSI']['PASSENGER_DOOR']))
    ret.seatbeltUnlatched = cp_cam.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2
    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      # the radar is silent with openpilot long
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [('HS2_DYN1_MDD_ETAT_2B6', float('nan')), ('NEW_MSG_318', float('nan'))], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
