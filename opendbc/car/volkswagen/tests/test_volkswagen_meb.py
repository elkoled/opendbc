import pytest

from opendbc.car import Bus, structs
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.carstate import CarState
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR


HCA_03 = 0x303
LDW_02 = 0x397

# HCA_03 Power signal: bit 16, 8-bit, scale 0.4. So raw = packed_byte_2, value = raw * 0.4
POWER_SCALE = 0.4


def _decode_hca_power_raw(data):
  return data[2]


def _decode_hca_request_status(data):
  return (data[1] >> 4) & 0x0F


def _decode_ldw_leds(data):
  # LDW_Status_LED_gelb @ bit 61 (byte 7 bit 5), LDW_Status_LED_gruen @ bit 62 (byte 7 bit 6)
  gelb = (data[7] >> 5) & 0x01
  gruen = (data[7] >> 6) & 0x01
  return gelb, gruen


def _make_CC(*, lat_active, curvature=0.0):
  cc = structs.CarControl()
  cc.enabled = lat_active
  cc.latActive = lat_active
  cc.actuators.curvature = curvature
  cc.currentCurvature = curvature
  return cc.as_reader()


def _make_cs(CP, *, steering_torque=0.0, v_ego=20.0):
  cs = CarState(CP)
  cs.gra_stock_values = {"COUNTER": 0}
  cs.measured_curvature = 0.0
  cs.out.steeringTorque = steering_torque
  cs.out.steeringPressed = abs(steering_torque) > 60  # STEER_DRIVER_ALLOWANCE
  cs.out.vEgo = v_ego
  cs.out.vEgoRaw = v_ego
  return cs


def _run(cc, cs, CC, n=200):
  last_hca, last_ldw = None, None
  for _ in range(n):
    _, sends = cc.update(CC, cs, 0)
    for addr, data, _bus in sends:
      if addr == HCA_03:
        last_hca = data
      elif addr == LDW_02:
        last_ldw = data
  return last_hca, last_ldw


@pytest.fixture
def cc_cp():
  CP = CarInterface.get_non_essential_params(CAR.VOLKSWAGEN_ID4_MK1.value)
  dbc = {b: 'vw_meb' for b in (Bus.pt, Bus.alt, Bus.cam, Bus.radar)}
  return CarController(dbc, CP), CP


# --- Torque bar (HCA_03 Power) ---------------------------------------------

# Math: target_power_driver = interp(CS.out.steeringTorque, [60, 300], [50, 4])
# Signed torque: only positive torque past the allowance reduces power.
# (Carstate uses abs() to detect steering_pressed; carcontroller uses signed torque per sunnypilot.)
# At vEgo=20 (>= 0.5), target_power = target_power_driver. Power raw byte = int(power / 0.4)
@pytest.mark.parametrize("driver_torque, expected_power", [
  (0,    50),  # no override → full
  (30,   50),  # below allowance → full
  (60,   50),  # at threshold
  (120,  38),  # interp midway low
  (180,  27),  # interp middle
  (240,  15),  # interp middle high
  (300,   4),  # at max → min
  (400,   4),  # past max → clamped
  # Negative torque is below the [60, 300] interp range, so np.interp clamps to y[0]=50
  (-120, 50),
  (-180, 50),
  (-300, 50),
  (-500, 50),
])
def test_torque_bar_curve(cc_cp, driver_torque, expected_power):
  cc, CP = cc_cp
  cs = _make_cs(CP, steering_torque=driver_torque)
  CC = _make_CC(lat_active=True)
  last_hca, _ = _run(cc, cs, CC)
  assert last_hca is not None, "HCA_03 was never sent"
  power_raw = _decode_hca_power_raw(last_hca)
  expected_raw = round(expected_power / POWER_SCALE)
  # ±2 raw units tolerance for packer rounding and rate-step quantization
  assert abs(power_raw - expected_raw) <= 2, \
    f"torque={driver_torque}Nm/100 expected raw~{expected_raw} ({expected_power}%) got raw={power_raw} ({power_raw*POWER_SCALE:.1f}%)"


def test_torque_bar_rate_limit(cc_cp):
  """STEERING_POWER_STEP=2 caps per-step delta. Raw delta = 2/0.4 = 5."""
  cc, CP = cc_cp
  cs = _make_cs(CP, steering_torque=0)  # target = POWER_MAX = 50
  CC = _make_CC(lat_active=True)
  powers = []
  for _ in range(60):
    _, sends = cc.update(CC, cs, 0)
    for addr, data, _bus in sends:
      if addr == HCA_03:
        powers.append(data[2])
  assert len(powers) > 5
  for i in range(1, len(powers)):
    delta = abs(powers[i] - powers[i - 1])
    assert delta <= 6, f"power_raw jump at idx {i}: {powers[i-1]} -> {powers[i]}"


def test_torque_bar_speed_gate(cc_cp):
  """At very low speed (vEgo < 0.5 m/s), target = STEERING_POWER_MIN even with no override."""
  cc, CP = cc_cp
  cs = _make_cs(CP, steering_torque=0, v_ego=0.0)
  CC = _make_CC(lat_active=True)
  last_hca, _ = _run(cc, cs, CC)
  power_raw = _decode_hca_power_raw(last_hca)
  # STEERING_POWER_MIN = 4 → raw = 10
  assert abs(power_raw - 10) <= 1, f"low-speed power_raw should be ~10 (4%) got {power_raw}"


def test_hca_request_status_active_inactive(cc_cp):
  cc, CP = cc_cp
  # active
  cs = _make_cs(CP, steering_torque=0)
  CC = _make_CC(lat_active=True)
  last_hca, _ = _run(cc, cs, CC, n=10)
  assert _decode_hca_request_status(last_hca) == 4, "RequestStatus must be 4 when lat_active"
  # inactive (fresh controller, latActive=False)
  cc_off = CarController({b: 'vw_meb' for b in (Bus.pt, Bus.alt, Bus.cam, Bus.radar)}, CP)
  cs_off = _make_cs(CP, steering_torque=0)
  CC_off = _make_CC(lat_active=False)
  last_hca_off, _ = _run(cc_off, cs_off, CC_off, n=10)
  assert _decode_hca_request_status(last_hca_off) == 2, "RequestStatus must be 2 when not active"
  # When inactive, power and curvature must both be 0 so safety's inactive_angle_is_zero passes
  assert last_hca_off[2] == 0, "Power must be 0 when not lat_active"
  curvature_raw = (last_hca_off[3] | (last_hca_off[4] << 8)) & 0x7FFF
  assert curvature_raw == 0, "Curvature must be 0 when not lat_active"


# --- Steering limit warning (LDW_02 LEDs) ----------------------------------

def test_warning_yellow_led_when_steering_pressed(cc_cp):
  cc, CP = cc_cp
  cs = _make_cs(CP, steering_torque=100)  # > 60 allowance
  CC = _make_CC(lat_active=True)
  _, last_ldw = _run(cc, cs, CC, n=30)
  assert last_ldw is not None, "LDW_02 was never sent"
  gelb, gruen = _decode_ldw_leds(last_ldw)
  assert gelb == 1, "yellow LED must be on when steering is pressed"
  assert gruen == 0, "green LED must be off while overriding"


def test_warning_green_led_when_active_no_override(cc_cp):
  cc, CP = cc_cp
  cs = _make_cs(CP, steering_torque=0)
  CC = _make_CC(lat_active=True)
  _, last_ldw = _run(cc, cs, CC, n=30)
  assert last_ldw is not None
  gelb, gruen = _decode_ldw_leds(last_ldw)
  assert gelb == 0
  assert gruen == 1, "green LED must be on when active without override"


def test_warning_off_when_lat_inactive(cc_cp):
  cc, CP = cc_cp
  cs = _make_cs(CP, steering_torque=100)
  CC = _make_CC(lat_active=False)
  _, last_ldw = _run(cc, cs, CC, n=30)
  assert last_ldw is not None
  gelb, gruen = _decode_ldw_leds(last_ldw)
  assert gelb == 0, "yellow LED must be off when openpilot not active"
  assert gruen == 0, "green LED must be off when openpilot not active"


# --- 1:1 alignment between Python (carcontroller) and C (panda safety) limits ----

def test_angle_limits_match_safety_header():
  """CCP.ANGLE_LIMITS must match VOLKSWAGEN_MEB_STEERING_LIMITS in volkswagen_meb.h.
  Both sides clip curvature through these same numbers; any drift breaks the
  steer_angle_cmd_checks contract and the safety hook can start blocking valid commands."""
  import re
  from pathlib import Path
  CP = CarInterface.get_non_essential_params(CAR.VOLKSWAGEN_ID4_MK1.value)
  ccp = type(cc_cp)  # not used; access CCP via CarController instance below
  from opendbc.car.volkswagen.values import CarControllerParams
  ccp = CarControllerParams(CP)
  py_limits = ccp.ANGLE_LIMITS

  header = Path(__file__).resolve().parents[3] / "safety/modes/volkswagen_meb.h"
  src = header.read_text()

  # Pull the VOLKSWAGEN_MEB_STEERING_LIMITS block up to the matching closing brace
  start = src.index("VOLKSWAGEN_MEB_STEERING_LIMITS")
  open_brace = src.index("{", start)
  depth = 0
  end = open_brace
  for i, ch in enumerate(src[open_brace:], start=open_brace):
    if ch == "{":
      depth += 1
    elif ch == "}":
      depth -= 1
      if depth == 0:
        end = i
        break
  block = src[open_brace + 1:end]

  def _grab_float(field):
    mm = re.search(rf"\.{field}\s*=\s*([-+]?\d+(?:\.\d*)?(?:e[-+]?\d+)?)", block)
    assert mm is not None, f"safety header missing field {field}"
    return float(mm.group(1))

  def _grab_lookup(field):
    mm = re.search(rf"\.{field}\s*=\s*\{{\s*\{{([^}}]+)\}}\s*,\s*\{{([^}}]+)\}}\s*\}}", block, re.DOTALL)
    assert mm is not None, f"safety header missing lookup {field}"
    xs = [float(s.strip().rstrip(".")) for s in mm.group(1).split(",") if s.strip()]
    ys = [float(s.strip().rstrip(".")) for s in mm.group(2).split(",") if s.strip()]
    return xs, ys

  c_max_angle = _grab_float("max_angle")
  c_a2c = _grab_float("angle_deg_to_can")
  c_up_x, c_up_y = _grab_lookup("angle_rate_up_lookup")
  c_dn_x, c_dn_y = _grab_lookup("angle_rate_down_lookup")

  # Python's STEER_ANGLE_MAX is in rad/m; C's max_angle is in CAN-raw units (= rad/m * a2c).
  expected_max_can = py_limits.STEER_ANGLE_MAX * c_a2c
  assert abs(c_max_angle - expected_max_can) <= 1, \
    f"max_angle drift: Python {py_limits.STEER_ANGLE_MAX} rad/m * {c_a2c} = {expected_max_can}, C={c_max_angle}"

  py_up_x, py_up_y = py_limits.ANGLE_RATE_LIMIT_UP
  py_dn_x, py_dn_y = py_limits.ANGLE_RATE_LIMIT_DOWN
  assert py_up_x == c_up_x and py_up_y == c_up_y, "angle_rate_up_lookup mismatch"
  assert py_dn_x == c_dn_x and py_dn_y == c_dn_y, "angle_rate_down_lookup mismatch"
