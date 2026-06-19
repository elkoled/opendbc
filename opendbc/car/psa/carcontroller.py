from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering, create_driver_torque, create_steering_hold, create_request_takeover
from opendbc.car.psa.values import CarControllerParams, CAR
import random
import math

SteerControlType = structs.CarParams.SteerControlType

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0
    self.apply_torque_factor = 0
    self.apply_torque = 0
    self.status = 2
    self.takeover_req_sent = False
    # this is the frame when the latactive is being pressed
    self.lat_activation_frame  = 0
    self.car_fingerprint = CP.carFingerprint
    self.params = CarControllerParams(CP)
    self.steering_hold_counter = 0
    self.next_steering_hold = random.randint(8, 12)  # ~10Hz con jitter ±20%
    self.driver_torque_counter = 0
    self.next_driver_torque = random.randint(500, 800)  # 5–8 s @100 Hz

  def _reset_lat_state(self):
    self.status = 2
    self.apply_torque_factor = 0
    self.takeover_req_sent = False
    self.lat_activation_frame = 0

  def _activate_eps(self, eps_active):
    # Save the frame number when the LKA (steering assist) button is first pressed on the car
    if self.lat_activation_frame == 0:
      # first frame the EPS activate or re activate is sent
      self.lat_activation_frame = self.frame
      self.takeover_req_sent = False


    if not eps_active: # and not CS.out.steeringPressed:
      #######
      # Alarm - Takeover request!
      # EPS works from 50km/h - Takeover Request if speed is slower than 50
      ######
      if not self.takeover_req_sent and self.frame % 2 == 0: # 50 Hz
        if (self.frame - self.lat_activation_frame) > 10:
        # can_sends.append(create_request_takeover(self.packer, CS.HS2_DYN_MDD_ETAT_2F6,1))
          self.takeover_req_sent = True

      ######
      # EPS activation sequence 2->3->4 to re-engage
      # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
      ######
      self.status = 2 if self.status == 4 else self.status + 1
      # EPS likes a progressive activation of the Torque Factor
      self.apply_torque_factor += 10
      self.apply_torque_factor = min( self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR)

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    self.apply_new_torque = 0
    apply_new_torque = 0

    # lateral control
    if self.CP.steerControlType == SteerControlType.torque:
      if self.frame % self.params.STEER_STEP == 0:
        if not CC.latActive:
          self._reset_lat_state()
        else:
          if not CS.eps_active: # and not CS.out.steeringPressed:
            self._activate_eps( CS.eps_active)

          else:
            ##########
            ### START EPS ACTIVE
            ######
            # EPS is active, proceed with lateral control
            self.lat_activation_frame = 0
            self.status = 4 # 4: EPS ACTIVE

            ######
            # TORQUE CALCULATION
            temp_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))
            apply_new_torque = apply_driver_steer_torque_limits(temp_torque, self.apply_torque_last,
                                                            CS.out.steeringTorque, self.params, self.params.STEER_MAX)

            # Linearly increase torque factor
            ratio = min(1.0, (abs(apply_new_torque) / float(self.params.STEER_MAX)) * 1.0)

            self.apply_torque_factor = int(self.params.MIN_TORQUE_FACTOR + ratio * (self.params.MAX_TORQUE_FACTOR - self.params.MIN_TORQUE_FACTOR))
            self.apply_torque_factor = max(self.params.MIN_TORQUE_FACTOR, min(self.apply_torque_factor, self.params.MAX_TORQUE_FACTOR))


        #
        #####
        # CAN MESSAGE needs to be sent every 5 frames
        #  - psa.h  check_relay is set for PSA_LANE_KEEP_ASSIST
        ####
        can_sends.append(create_lka_steering(self.packer, CC.latActive, apply_new_torque, self.apply_torque_factor, self.status))
        # last sent value to the EPS
        self.apply_torque_last = apply_new_torque
        ### END EPS ACTIVE
        ##########

    # if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,):
    #   if self.frame % 10 == 0:
    #     # send steering wheel hold message
    #     can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))



    if self.car_fingerprint in (CAR.PSA_PEUGEOT_3008,):
      if not CC.latActive:
        self.driver_torque_counter = 0
        self.next_driver_torque = random.randint(500, 800)
      else:
        # --- HOLD HANDS (~10 Hz con jitter 8–12 frame) ---
        self.steering_hold_counter += 1
        if self.steering_hold_counter >= self.next_steering_hold:
          can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))
          self.steering_hold_counter = 0
          self.next_steering_hold = random.randint(8, 12)
        # --- DRIVER TORQUE (ogni 5–8 s) ---
        self.driver_torque_counter += 1
        if self.driver_torque_counter >= self.next_driver_torque:
          can_sends.append(create_driver_torque(self.packer, CS.steering))
          self.driver_torque_counter = 0
          self.next_driver_torque = random.randint(500, 800)

    # Actuators output
    new_actuators = actuators.as_builder()
    if self.CP.steerControlType == SteerControlType.torque:
      # Keep last applied torque between 20 Hz LKA updates.
      # The EPS maintains assist longer than 50 ms, preventing gaps in actuator output.
      new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends