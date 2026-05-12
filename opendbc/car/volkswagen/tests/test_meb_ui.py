import unittest

import numpy as np

from opendbc.car import DT_CTRL, structs
from opendbc.car.interfaces import CarInterfaceBase, get_torque_params
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.values import VolkswagenFlags


# Replicas of openpilot selfdrive/controls/lib/latcontrol_angle.py and torque_bar.py
# kept in sync so opendbc-side tests can verify the UI contract for MEB cars.

STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # deg, latcontrol_angle.py:7
TORQUE_BAR_MIN_SAT_SPEED = 5.0          # m/s, latcontrol_angle.py:13


def _check_saturation(sat_time, saturated, v_ego, steering_pressed, curvature_limited, sat_limit, dt,
                      sat_check_min_speed=TORQUE_BAR_MIN_SAT_SPEED):
  # Mirror of selfdrive/controls/lib/latcontrol.py:_check_saturation
  if (saturated or curvature_limited) and v_ego > sat_check_min_speed and not steering_pressed:
    sat_time += dt
  else:
    sat_time -= dt
  sat_time = float(np.clip(sat_time, 0.0, sat_limit))
  return sat_time, sat_time > (sat_limit - 1e-3)


def _latcontrol_angle_saturated(angle_steers_des, steering_angle_deg, v_ego, steering_pressed,
                                curvature_limited, sat_limit, dt, prev_sat_time=0.0):
  # Mirror of selfdrive/controls/lib/latcontrol_angle.py update path for non-tesla/hyundai brands
  angle_control_saturated = abs(angle_steers_des - steering_angle_deg) > STEER_ANGLE_SATURATION_THRESHOLD
  return _check_saturation(prev_sat_time, angle_control_saturated, v_ego, steering_pressed,
                           curvature_limited, sat_limit, dt)


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
  ret.steerActuatorDelay = 0.1
  ret.steerControlType = structs.CarParams.SteerControlType.angle
  ret.transmissionType = structs.CarParams.TransmissionType.direct
  ret.steerRatio = 15.6
  ret.wheelbase = 2.77
  ret.mass = 2224.
  ret.tireStiffnessFront = 192150.
  ret.tireStiffnessRear = 202500.
  ret.centerToFront = ret.wheelbase * 0.45
  return ret


class TestMebTorqueBar(unittest.TestCase):
  def test_max_lateral_accel_set(self):
    """maxLateralAccel must be > 0 — torque_bar.py divides by it."""
    params = get_torque_params()
    assert 'VOLKSWAGEN_ID4_MK1' in params, "ID4_MK1 missing from torque_data"
    cp = CarInterfaceBase.get_std_params('VOLKSWAGEN_ID4_MK1')
    assert cp.maxLateralAccel > 0.1, f"maxLateralAccel too small: {cp.maxLateralAccel}"

  def test_torque_bar_straight_line(self):
    """Driving straight with no curvature error: bar should be near zero."""
    val = _torque_bar_value(curvature=0.0, desired_curvature=0.0, v_ego=20.0, roll=0.0,
                            max_lateral_accel=1.0, lat_active=True)
    assert abs(val) < 0.01, f"Expected near 0, got {val}"

  def test_torque_bar_disengaged_is_zero(self):
    """When latActive=False the bar always reads zero."""
    val = _torque_bar_value(curvature=0.1, desired_curvature=0.1, v_ego=20.0, roll=0.0,
                            max_lateral_accel=1.0, lat_active=False)
    assert val == 0.0

  def test_torque_bar_turn_in_range(self):
    """Tracking a 0.5 m/s^2 turn perfectly: bar should reflect that lateral accel / max ratio."""
    # 0.5 m/s^2 lateral accel at 20 m/s → curvature = 0.00125 1/m
    val = _torque_bar_value(curvature=0.00125, desired_curvature=0.00125, v_ego=20.0, roll=0.0,
                            max_lateral_accel=1.0, lat_active=True)
    assert 0.45 < val < 0.55, f"Expected ~0.5, got {val}"

  def test_torque_bar_clipped_to_unit(self):
    """Bar saturates at ±1.0 even when actual lateral acceleration exceeds the configured max."""
    val = _torque_bar_value(curvature=0.01, desired_curvature=0.02, v_ego=30.0, roll=0.0,
                            max_lateral_accel=1.0, lat_active=True)
    assert val == 1.0

    val_neg = _torque_bar_value(curvature=-0.01, desired_curvature=-0.02, v_ego=30.0, roll=0.0,
                                max_lateral_accel=1.0, lat_active=True)
    assert val_neg == -1.0

  def test_torque_bar_roll_compensation_reduces_value(self):
    """Banked road in the steering direction lowers the bar (roll compensation)."""
    flat = _torque_bar_value(curvature=0.001, desired_curvature=0.001, v_ego=20.0, roll=0.0,
                             max_lateral_accel=1.0, lat_active=True)
    banked = _torque_bar_value(curvature=0.001, desired_curvature=0.001, v_ego=20.0, roll=0.05,
                               max_lateral_accel=1.0, lat_active=True)
    assert banked < flat, f"Banked ({banked}) should be less than flat ({flat}) with positive roll"


class TestMebSteerSaturation(unittest.TestCase):
  def _sat_limit(self):
    return _make_cp().steerLimitTimer

  def test_no_saturation_when_tracking(self):
    """EPS matches the request: saturated flag stays False."""
    sat_limit = self._sat_limit()
    sat_time = 0.0
    for _ in range(int(sat_limit / DT_CTRL) + 50):
      sat_time, saturated = _latcontrol_angle_saturated(
        angle_steers_des=10.0, steering_angle_deg=10.0, v_ego=20.0,
        steering_pressed=False, curvature_limited=False, sat_limit=sat_limit, dt=DT_CTRL,
        prev_sat_time=sat_time)
    assert not saturated, "Should not be saturated when des == measured"

  def test_saturation_triggers_after_timer(self):
    """Sustained angle error → saturated after steerLimitTimer seconds."""
    sat_limit = self._sat_limit()
    sat_time = 0.0
    triggered_at = None
    for i in range(int(sat_limit / DT_CTRL) + 50):
      sat_time, saturated = _latcontrol_angle_saturated(
        angle_steers_des=15.0, steering_angle_deg=10.0, v_ego=20.0,
        steering_pressed=False, curvature_limited=False, sat_limit=sat_limit, dt=DT_CTRL,
        prev_sat_time=sat_time)
      if saturated and triggered_at is None:
        triggered_at = i * DT_CTRL
    assert triggered_at is not None, "Saturation never triggered"
    assert sat_limit - 0.02 <= triggered_at <= sat_limit + 0.02, \
      f"Saturated at {triggered_at}s, expected ~{sat_limit}s"

  def test_curvature_limited_triggers(self):
    """curvature_limited flag alone triggers saturation."""
    sat_limit = self._sat_limit()
    sat_time = 0.0
    saturated = False
    for _ in range(int(sat_limit / DT_CTRL) + 50):
      sat_time, saturated = _latcontrol_angle_saturated(
        angle_steers_des=0.0, steering_angle_deg=0.0, v_ego=20.0,
        steering_pressed=False, curvature_limited=True, sat_limit=sat_limit, dt=DT_CTRL,
        prev_sat_time=sat_time)
    assert saturated, "curvature_limited should trigger saturation"

  def test_steering_pressed_inhibits_saturation(self):
    """Driver torque on the wheel suppresses the saturated alert."""
    sat_limit = self._sat_limit()
    sat_time = 0.0
    saturated = False
    for _ in range(int(sat_limit / DT_CTRL) + 50):
      sat_time, saturated = _latcontrol_angle_saturated(
        angle_steers_des=15.0, steering_angle_deg=10.0, v_ego=20.0,
        steering_pressed=True, curvature_limited=False, sat_limit=sat_limit, dt=DT_CTRL,
        prev_sat_time=sat_time)
    assert not saturated, "steeringPressed should suppress saturation"

  def test_low_speed_inhibits_saturation(self):
    """Below sat_check_min_speed the timer cannot grow."""
    sat_limit = self._sat_limit()
    sat_time = 0.0
    saturated = False
    for _ in range(int(sat_limit / DT_CTRL) + 50):
      sat_time, saturated = _latcontrol_angle_saturated(
        angle_steers_des=15.0, steering_angle_deg=10.0, v_ego=2.0,
        steering_pressed=False, curvature_limited=False, sat_limit=sat_limit, dt=DT_CTRL,
        prev_sat_time=sat_time)
    assert not saturated, "Low speed should suppress saturation"


class _StubCarState:
  """Minimal CarState stand-in matching the attributes the MEB carcontroller reads."""

  def __init__(self):
    self.out = structs.CarState()
    self.out.vEgoRaw = 20.0
    self.out.vEgo = 20.0
    self.out.steeringPressed = False
    self.steering_curvature_measured = 0.0
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


class TestMebCarControllerActuators(unittest.TestCase):
  def test_inactive_emits_zero_curvature(self):
    cc_mod = _build_carcontroller()
    new_actuators, _ = cc_mod.update(_make_cc(lat_active=False), _StubCarState(), 0)
    assert new_actuators.curvature == 0.0
    # steeringAngleDeg inherited from CC.actuators (controlsd writes from latcontrol_angle)
    assert new_actuators.steeringAngleDeg == 0.0

  def test_active_emits_curvature_and_preserves_angle(self):
    """latActive=True with a curvature request: carcontroller ramps and preserves angle for UI."""
    cc_mod = _build_carcontroller()
    cs = _StubCarState()
    # Ramp under jerk limit a few frames to get past zero
    cc = _make_cc(lat_active=True, desired_curvature=0.1, desired_angle_deg=25.0,
                  current_curvature=0.0)
    last = None
    for _ in range(20):
      new_actuators, _ = cc_mod.update(cc, cs, 0)
      last = new_actuators
    assert last.curvature > 0.0
    assert last.curvature <= 0.195
    # actuators_output preserves steeringAngleDeg so selfdrived's safety-limit comparison
    # (CC.actuators.steeringAngleDeg vs CO.actuatorsOutput.steeringAngleDeg) does not false-trigger.
    assert last.steeringAngleDeg == 25.0

  def test_actuator_never_exceeds_max(self):
    """Whatever the request, output curvature never exceeds CURVATURE_LIMITS.CURVATURE_MAX."""
    cc_mod = _build_carcontroller()
    cs = _StubCarState()
    cc = _make_cc(lat_active=True, desired_curvature=1.0, desired_angle_deg=999.0,
                  current_curvature=0.0)
    for _ in range(2000):
      new_actuators, _ = cc_mod.update(cc, cs, 0)
      assert abs(new_actuators.curvature) <= cc_mod.CCP.CURVATURE_LIMITS.CURVATURE_MAX + 1e-6

  def test_steering_power_ramps_down_after_disengage(self):
    """When latActive flips False, steering_power decays to zero without spiking apply_curvature."""
    cc_mod = _build_carcontroller()
    cs = _StubCarState()
    # Ramp up first
    for _ in range(20):
      cc_mod.update(_make_cc(lat_active=True, desired_curvature=0.05,
                             desired_angle_deg=10.0, current_curvature=0.0), cs, 0)
    # Now disengage
    last_power = cc_mod.steering_power_last
    assert last_power > 0
    for _ in range(50):
      cc_mod.update(_make_cc(lat_active=False), cs, 0)
    # Power should ramp down to 0 within 50 frames at STEERING_POWER_STEP=2 per STEER_STEP
    assert cc_mod.steering_power_last == 0

  def test_safety_curvature_bounds_match(self):
    """Python CURVATURE_LIMITS.CURVATURE_MAX must match volkswagen_meb.h max_curvature."""
    cc_mod = _build_carcontroller()
    py_max = cc_mod.CCP.CURVATURE_LIMITS.CURVATURE_MAX
    # Safety panda max_curvature = 29105 raw units, scale 6.7e-6 rad/m → ~0.195
    safety_max_rad = 29105 * 6.7e-6
    assert abs(py_max - safety_max_rad) < 1e-3, \
      f"Python ({py_max}) and safety C ({safety_max_rad}) disagree on max curvature"


if __name__ == "__main__":
  unittest.main()
