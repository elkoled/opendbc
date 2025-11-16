from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering, create_steering_hold, create_driver_torque
from opendbc.car.psa.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0
    self.apply_torque = 0
    self.status = 2

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators


    # lateral control
    apply_torque = 0
    if self.frame % 5 == 0:
      if CC.latActive:
        new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                        CS.out.steeringTorque, CarControllerParams)
        # TODO: test 2
        # emulate driver torque message at 1 Hz
        # if self.frame % 100 == 0:
        #   can_sends.append(create_driver_torque(self.packer, CS.steering))

      # EPS disengages on steering override, activation sequence 2->3->4 to re-engage
      # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
      if not CC.latActive:
        self.status = 2
      elif not CS.eps_active and not CS.out.steeringPressed:
        self.status = 2 if self.status == 4 else self.status + 1
      else:
        self.status = 4

      # TODO: test 1
      if self.frame % 900 == 0:
        self.status = 0  # deactivate LKA every 9 seconds to avoid steering hold warning

      can_sends.append(create_lka_steering(self.packer, CC.latActive, apply_torque, self.status))

      # TODO: test 3
      # if self.frame % 500 == 0:
      #   # send steering wheel hold message at 10 Hz to keep EPS engaged
      #   can_sends.append(create_steering_hold(self.packer, CC.latActive, CS.is_dat_dira))

      self.apply_torque_last = apply_torque

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
