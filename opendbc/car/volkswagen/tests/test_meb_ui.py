import numpy as np

from opendbc.car import DT_CTRL, structs
from opendbc.car.interfaces import CarInterfaceBase, get_torque_params
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.values import VolkswagenFlags


# Replicas of selfdrive/controls/lib/latcontrol_angle.py and selfdrive/ui/mici/onroad/torque_bar.py
# kept here so opendbc-side tests can verify the UI/saturation contract for MEB cars without
# pulling openpilot/cereal into the opendbc test runner.

STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # deg, latcontrol_angle.py
LATCONTROL_ANGLE_MIN_SAT_SPEED = 5.0    # m/s, latcontrol_angle.py


def _check_saturation(sat_time, saturated, v_ego, steering_pressed, curvature_limited, sat_limit, dt,
                      sat_check_min_speed=LATCONTROL_ANGLE_MIN_SAT_SPEED):
  if (saturated or curvature_limited) and v_ego > sat_check_min_speed and not steering_pressed:
    sat_time += dt
  else:
    sat_time -= dt
  sat_time = float(np.clip(sat_time, 0.0, sat_limit))
  return sat_time, sat_time > (sat_limit - 1e-3)


def _latcontrol_angle_saturated(angle_steers_des, steering_angle_deg, v_ego, steering_pressed,
                                curvature_limited, sat_limit, dt, prev_sat_time=0.0):
  angle_control_saturated = abs(angle_steers_des - steering_angle_deg) > STEER_ANGLE_SATURATION_THRESHOLD
  return _check_saturation(prev_sat_time, angle_control_saturated, v_ego, steering_pressed,
                           curvature_limited, sat_limit, dt)


def _torque_bar_value(curvature, desired_curvature, v_ego, roll, max_lateral_accel, lat_active):
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
  ret.steerControlType = structs.CarParams.SteerControlType.angle
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


def _make_cc(lat_active=False, desired_curvature=0.0, desired_angle_deg=0.0, current_curvature=0.0):
  cc = structs.CarControl()
  cc.actuators.curvature = desired_curvature
  cc.actuators.steeringAngleDeg = desired_angle_deg
  cc.latActive = lat_active
  cc.currentCurvature = current_curvature
  return cc.as_reader()


def _build_carcontroller():
  return CarController({'pt': 'vw_meb'}, _make_cp())


class TestMebTorqueBar:
  def test_max_lateral_accel_set(self):
    """torque_bar.py divides by maxLateralAccel; ensure ID4_MK1 has a usable value."""
    params = get_torque_params()
    assert 'VOLKSWAGEN_ID4_MK1' in params, "ID4_MK1 missing from torque_data"
    cp = CarInterfaceBase.get_std_params('VOLKSWAGEN_ID4_MK1')
    assert cp.maxLateralAccel > 0.1

  def test_torque_bar_straight_line(self):
    assert abs(_torque_bar_value(0.0, 0.0, 20.0, 0.0, 1.0, True)) < 0.01

  def test_torque_bar_disengaged_is_zero(self):
    assert _torque_bar_value(0.1, 0.1, 20.0, 0.0, 1.0, False) == 0.0

  def test_torque_bar_turn_in_range(self):
    assert 0.45 < _torque_bar_value(0.00125, 0.00125, 20.0, 0.0, 1.0, True) < 0.55

  def test_torque_bar_clipped_to_unit(self):
    assert _torque_bar_value(0.01, 0.02, 30.0, 0.0, 1.0, True) == 1.0
    assert _torque_bar_value(-0.01, -0.02, 30.0, 0.0, 1.0, True) == -1.0

  def test_torque_bar_roll_compensation_reduces_value(self):
    flat = _torque_bar_value(0.001, 0.001, 20.0, 0.0, 1.0, True)
    banked = _torque_bar_value(0.001, 0.001, 20.0, 0.05, 1.0, True)
    assert banked < flat


class TestMebLatControlAngleSaturation:
  """For SteerControlType.angle, LatControlAngle drives the steerSaturated alert."""

  sat_limit = _make_cp().steerLimitTimer

  def test_no_saturation_when_tracking(self):
    sat_time = 0.0
    saturated = False
    for _ in range(int(self.sat_limit / DT_CTRL) + 50):
      sat_time, saturated = _latcontrol_angle_saturated(10.0, 10.0, 20.0, False, False, self.sat_limit, DT_CTRL, sat_time)
    assert not saturated

  def test_saturation_triggers_after_timer(self):
    sat_time = 0.0
    triggered_at = None
    for i in range(int(self.sat_limit / DT_CTRL) + 50):
      sat_time, saturated = _latcontrol_angle_saturated(15.0, 10.0, 20.0, False, False, self.sat_limit, DT_CTRL, sat_time)
      if saturated and triggered_at is None:
        triggered_at = i * DT_CTRL
    assert triggered_at is not None
    assert self.sat_limit - 0.02 <= triggered_at <= self.sat_limit + 0.02

  def test_curvature_limited_triggers(self):
    sat_time = 0.0
    saturated = False
    for _ in range(int(self.sat_limit / DT_CTRL) + 50):
      sat_time, saturated = _latcontrol_angle_saturated(0.0, 0.0, 20.0, False, True, self.sat_limit, DT_CTRL, sat_time)
    assert saturated

  def test_steering_pressed_inhibits_saturation(self):
    sat_time = 0.0
    saturated = False
    for _ in range(int(self.sat_limit / DT_CTRL) + 50):
      sat_time, saturated = _latcontrol_angle_saturated(15.0, 10.0, 20.0, True, False, self.sat_limit, DT_CTRL, sat_time)
    assert not saturated

  def test_low_speed_inhibits_saturation(self):
    sat_time = 0.0
    saturated = False
    for _ in range(int(self.sat_limit / DT_CTRL) + 50):
      sat_time, saturated = _latcontrol_angle_saturated(15.0, 10.0, 2.0, False, False, self.sat_limit, DT_CTRL, sat_time)
    assert not saturated


class TestMebCarController:
  def test_inactive_emits_zero_curvature(self):
    new_actuators, _ = _build_carcontroller().update(_make_cc(lat_active=False), _StubCarState(), 0)
    assert new_actuators.curvature == 0.0

  def test_active_emits_curvature(self):
    cc_mod = _build_carcontroller()
    cc = _make_cc(lat_active=True, desired_curvature=0.05, current_curvature=0.0)
    last = None
    for _ in range(50):
      new_actuators, _ = cc_mod.update(cc, _StubCarState(), 0)
      last = new_actuators
    assert last.curvature > 0.0
    assert last.curvature <= cc_mod.CCP.CURVATURE_MAX

  def test_actuator_never_exceeds_max(self):
    cc_mod = _build_carcontroller()
    cc = _make_cc(lat_active=True, desired_curvature=1.0, current_curvature=0.0)
    for _ in range(2000):
      new_actuators, _ = cc_mod.update(cc, _StubCarState(), 0)
      assert abs(new_actuators.curvature) <= cc_mod.CCP.CURVATURE_MAX + 1e-6

  def test_safety_curvature_bounds_match(self):
    """openpilot CURVATURE_MAX must match volkswagen_meb.h max_curvature (29105 × 6.7e-6)."""
    cc_mod = _build_carcontroller()
    safety_max_rad = 29105 * 6.7e-6
    assert abs(cc_mod.CCP.CURVATURE_MAX - safety_max_rad) < 1e-3
    assert abs(cc_mod.CCP.CURVATURE_LIMITS.CURVATURE_MAX - safety_max_rad) < 1e-3
