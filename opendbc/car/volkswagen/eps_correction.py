"""
EPS plant inverse for VW MEB (initial coverage: VOLKSWAGEN_ID4_MK1).

The MEB EPS systematically under-actuates commanded curvature at highway
speed: when openpilot commands curvature c, the rack delivers roughly
0.86 - 0.97 * c depending on speed. This module applies the inverse
correction in front of the existing rack-loop error feed-forward.

Fitted offline against 1662 engaged route-segments from 7 ID4_MK1
dongles (4-5 cars contributing to each speed anchor; one outlier dongle
excluded with documented reason). Static per-speed gain G(v); bias was
≤ 5e-5 rad/m across cars so it's not deployed (kept here for reference).
Leave-one-route-out CV passed, and held-out empirical RMSE reduction was
+8 to +22% at highway speeds on the majority dongle.

Outside the anchor range the correction smoothsteps to identity, so the
controller sees no step discontinuity at the deployment boundary. For
MEB fingerprints not in the fit (ID3 / Q4 / Born / Enyaq), the correction
passes through unchanged.

Derivation: openpilot tools/vw_id4_lateral/, RESULT.md.
"""
from __future__ import annotations


# (speed_mps, G, bias_rad_per_m) — anchors fitted on the fleet-median across
# 5 non-outlier dongles. bias is kept here for traceability but kept below
# the deploy threshold so we apply gain only.
_TABLE_ID4_MK1 = (
  (60.0 / 3.6, 0.971, -3.1e-5),   #  60 km/h
  (100.0 / 3.6, 0.906, -2.8e-5),  # 100 km/h
  (120.0 / 3.6, 0.862,  5.3e-6),  # 120 km/h
  (140.0 / 3.6, 0.855, -5.5e-6),  # 140 km/h
)

# Smoothstep fade-in/out band (m/s) outside the anchor range. Keeps the
# correction continuous; outside the fade band it is identity.
_FADE_MPS = 5.0 / 3.6

# Safety bound on the resulting effective gain. If a future table update or
# interpolation produces a value outside this range, the correction is
# silently bypassed for that frame. Same envelope the planner uses.
_G_LO = 0.5
_G_HI = 1.5

# Supported fingerprints. Anything else is identity passthrough.
_SUPPORTED_FINGERPRINTS = ("VOLKSWAGEN_ID4_MK1",)


def _smoothstep(x: float) -> float:
  if x <= 0.0:
    return 0.0
  if x >= 1.0:
    return 1.0
  return x * x * (3.0 - 2.0 * x)


def _table_for(fingerprint: str):
  if fingerprint in _SUPPORTED_FINGERPRINTS:
    return _TABLE_ID4_MK1
  return None


def correct_meb_curvature(commanded: float, v_ego: float, fingerprint: str) -> float:
  """Apply the speed-dependent plant inverse to a commanded curvature.

  Returns the commanded value unchanged for unsupported fingerprints, for
  speeds outside the supported anchor range plus a smoothstep band, and
  if the interpolated gain falls outside the safety envelope.
  """
  table = _table_for(fingerprint)
  if table is None:
    return float(commanded)

  v = float(v_ego)
  v_lo = table[0][0]
  v_hi = table[-1][0]
  if v <= v_lo - _FADE_MPS or v >= v_hi + _FADE_MPS:
    return float(commanded)

  if v <= v_lo:
    alpha = _smoothstep((v - (v_lo - _FADE_MPS)) / _FADE_MPS)
    G = alpha * table[0][1] + (1.0 - alpha) * 1.0
    bias = alpha * table[0][2]
  elif v >= v_hi:
    alpha = _smoothstep(1.0 - (v - v_hi) / _FADE_MPS)
    G = alpha * table[-1][1] + (1.0 - alpha) * 1.0
    bias = alpha * table[-1][2]
  else:
    G = 1.0
    bias = 0.0
    for i in range(len(table) - 1):
      lo, G_lo, b_lo = table[i]
      hi, G_hi, b_hi = table[i + 1]
      if lo <= v <= hi:
        t = (v - lo) / (hi - lo)
        G = (1.0 - t) * G_lo + t * G_hi
        bias = (1.0 - t) * b_lo + t * b_hi
        break

  if not (_G_LO < G < _G_HI):
    return float(commanded)
  return float((float(commanded) - bias) / G)
