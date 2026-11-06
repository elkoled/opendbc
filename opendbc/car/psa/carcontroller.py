from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_steer_angle_limits_vm, get_max_angle_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.psa.psacan import create_lka_steering
from opendbc.car.psa.values import CarControllerParams
from opendbc.car.vehicle_model import VehicleModel


def get_safety_CP():
  # the 208 has the shortest wheelbase, so the lowest angle limits for every platform
  from opendbc.car.psa.interface import CarInterface
  return CarInterface.get_non_essential_params("PSA_PEUGEOT_208")


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.VM = VehicleModel(get_safety_CP())
    self.apply_angle_last = 0
    self.lat_active = False
    self.status = 2

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators

    # lateral control
    if self.frame % CarControllerParams.STEER_STEP == 0:
      # don't take over a curve steered beyond the lateral accel limit, safety would block the unwind
      max_angle = get_max_angle_vm(max(CS.out.vEgoRaw, 1), self.VM, CarControllerParams)
      self.lat_active = CC.latActive and (self.lat_active or abs(CS.out.steeringAngleDeg) < max_angle)

      apply_angle = apply_steer_angle_limits_vm(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
                                                self.lat_active, CarControllerParams, self.VM)

      # EPS disengages on steering override, activation sequence 2->3->4 to re-engage
      # STATUS  -  0: UNAVAILABLE, 1: UNSELECTED, 2: READY, 3: AUTHORIZED, 4: ACTIVE
      if not self.lat_active:
        self.status = 2
      elif not CS.eps_active and not CS.out.steeringPressed:
        self.status = 2 if self.status == 4 else self.status + 1
      else:
        self.status = 4

      can_sends.append(create_lka_steering(self.packer, self.lat_active, apply_angle, self.status))

      self.apply_angle_last = apply_angle

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last
    self.frame += 1
    return new_actuators, can_sends
