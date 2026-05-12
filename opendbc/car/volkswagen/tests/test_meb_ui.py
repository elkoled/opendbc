import unittest

import numpy as np

from opendbc.car import structs
from opendbc.car.interfaces import CarInterfaceBase, get_torque_params
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.values import VolkswagenFlags


def _torque_bar_value(curvature, desired_curvature, v_ego, roll, max_lateral_accel, lat_active):
  # Mirror of selfdrive/ui/mici/onroad/torque_bar.py:_update_state angle branch
  actual_lateral_accel = curvature * v_ego ** 2
  desired_lateral_accel = desired_curvature * v_ego ** 2
  accel_diff = desired_lateral_accel - actual_lateral_accel
  ACCELERATION_DUE_TO_GRAVITY = 9.81
  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY * np.interp(v_ego, [5, 15], [0.0, 1.0])
  lateral_acceleration = actual_lateral_accel - roll_compensation
  if not lat_active:
    return 0.0
  return float(np.clip((lateral_acceleration + accel_diff) / max_lateral_accel, -1, 1))


def _make_cp():
  ret = CarInterfaceBase.get_std_params('VOLKSWAGEN_ID4_MK1')
  ret.brand = 'volkswagen'
  ret.flags = int(VolkswagenFlags.MEB)
  ret.steerLimitTimer = 0.4
  ret.steerActuatorDelay = 0.3
  ret.steerControlType = structs.CarParams.SteerControlType.curvatureDEPRECATED
  ret.transmissionType = structs.CarParams.TransmissionType.direct
  ret.steerRatio = 15.6
  ret.wheelbase = 2.77
  ret.mass = 2224.
  return ret


class _StubCarState:
  def __init__(self):
    self.out = structs.CarState()
    self.out.vEgoRaw = 20.0
    self.out.vEgo = 20.0
    self.out.steeringPressed = False
    self.out.steeringTorque = 0.0
    self.measured_curvature = 0.0
    self.gra_stock_values = {"COUNTER": 0}


def _make_cc(lat_active=False, desired_curvature=0.0, current_curvature=0.0):
  cc = structs.CarControl()
  cc.actuators.curvature = desired_curvature
  cc.latActive = lat_active
  cc.currentCurvature = current_curvature
  return cc.as_reader()


class TestMebTorqueBar(unittest.TestCase):
  def test_max_lateral_accel_set(self):
    """maxLateralAccel must be > 0 — torque_bar.py divides by it."""
    params = get_torque_params()
    assert 'VOLKSWAGEN_ID4_MK1' in params, "ID4_MK1 missing from torque_data"
    cp = CarInterfaceBase.get_std_params('VOLKSWAGEN_ID4_MK1')
    assert cp.maxLateralAccel > 0.1, f"maxLateralAccel too small: {cp.maxLateralAccel}"

  def test_torque_bar_straight_line(self):
    val = _torque_bar_value(0.0, 0.0, 20.0, 0.0, 1.0, lat_active=True)
    assert abs(val) < 0.01

  def test_torque_bar_disengaged_is_zero(self):
    val = _torque_bar_value(0.1, 0.1, 20.0, 0.0, 1.0, lat_active=False)
    assert val == 0.0

  def test_torque_bar_turn_in_range(self):
    # 0.5 m/s^2 lateral accel at 20 m/s → curvature = 0.00125 1/m
    val = _torque_bar_value(0.00125, 0.00125, 20.0, 0.0, 1.0, lat_active=True)
    assert 0.45 < val < 0.55

  def test_torque_bar_clipped_to_unit(self):
    assert _torque_bar_value(0.01, 0.02, 30.0, 0.0, 1.0, lat_active=True) == 1.0
    assert _torque_bar_value(-0.01, -0.02, 30.0, 0.0, 1.0, lat_active=True) == -1.0

  def test_torque_bar_roll_compensation_reduces_value(self):
    flat = _torque_bar_value(0.001, 0.001, 20.0, 0.0, 1.0, lat_active=True)
    banked = _torque_bar_value(0.001, 0.001, 20.0, 0.05, 1.0, lat_active=True)
    assert banked < flat


class TestMebCarController(unittest.TestCase):
  def _build(self):
    return CarController({'pt': 'vw_meb'}, _make_cp())

  def test_inactive_emits_zero_curvature(self):
    cc_mod = self._build()
    new_actuators, _ = cc_mod.update(_make_cc(lat_active=False), _StubCarState(), 0)
    assert new_actuators.curvature == 0.0

  def test_active_emits_clipped_curvature(self):
    cc_mod = self._build()
    cc = _make_cc(lat_active=True, desired_curvature=0.05, current_curvature=0.0)
    new_actuators, _ = cc_mod.update(cc, _StubCarState(), 0)
    assert 0.0 < new_actuators.curvature <= cc_mod.CCP.CURVATURE_MAX

  def test_actuator_never_exceeds_max(self):
    cc_mod = self._build()
    cc = _make_cc(lat_active=True, desired_curvature=1.0, current_curvature=0.0)
    for _ in range(50):
      new_actuators, _ = cc_mod.update(cc, _StubCarState(), 0)
      assert abs(new_actuators.curvature) <= cc_mod.CCP.CURVATURE_MAX + 1e-6

  def test_safety_curvature_bounds_match(self):
    """Python CURVATURE_MAX must match volkswagen_meb.h max_curvature (29105 × 6.7e-6)."""
    cc_mod = self._build()
    safety_max_rad = 29105 * 6.7e-6  # 29105 raw × 6.7e-6 rad/m per LSB
    assert abs(cc_mod.CCP.CURVATURE_MAX - safety_max_rad) < 1e-3


if __name__ == "__main__":
  unittest.main()
