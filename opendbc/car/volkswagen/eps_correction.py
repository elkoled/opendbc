"""
Static EPS correction table for VW MEB curvature control.

Fit on 1,081 openpilot-engaged VOLKSWAGEN_ID4_MK1 segments from 9 dongles
(2026-05-22). Per (speed_anchor, |kappa|_bucket, sign-of-kappa)
cell, the table stores the mean projected residual
`direction * (kappa_desired - kappa_actual)` observed during engaged
driving. The value, when added to `carControl.actuators.curvature`,
shifts the commanded curvature toward what would have eliminated the
historical residual on the training fleet.

Limitations (from leave-one-dongle-out validation; see id4_lateral/1/FINDINGS.md):

- 1 of 7 held-out dongles got measurably WORSE with this table (-11% RMSE).
- The 80%-dominant (bad-tracking) dongle gained only +0.9%.
- Median RMSE reduction across held-out dongles: +4.8%.
- This is fit on VOLKSWAGEN_ID4_MK1 only; no MEB-platform-generalization
  evidence exists.

Safety: applied correction is capped at 50% of |kappa_desired| (matches
the openpilot10 `virtual/dynamic_steering` learner). Outside the supported
speed/kappa bucket range, returns 0.
"""
from __future__ import annotations

import math
from typing import List, Tuple


# Source: tools/lateral_fleet/eps_model.py @ /home/batman/lateral_fleet_out/model
# Generated from eps_correction_table.npz, fit layer=gated, min_samples=4.

# Speed anchors in m/s (== 20..140 km/h)
SPEED_ANCHORS_MPS: Tuple[float, ...] = (5.555555556, 11.111111111, 16.666666667, 22.222222222, 27.777777778, 33.333333333, 38.888888889)

# Log-spaced |kappa| bucket edges (rad/m), len = 13 -> 12 buckets
KAPPA_BUCKET_EDGES: Tuple[float, ...] = (1e-06, 2e-06, 4e-06, 8e-06, 1.6e-05, 3.2e-05, 6.4e-05, 0.000128, 0.000256, 0.000512, 0.001024, 0.002048, 0.004096)

# Table: SPEED x KAPPA x SIGN (sign 0 = positive kappa, 1 = negative kappa).
# Value at [s, k, sign] is the correction (rad/m) to ADD to commanded curvature
# at that operating point, in the desired direction.
TABLE: Tuple[Tuple[Tuple[float, float], ...], ...] = (
  (  # speed = 5.56 m/s
    (+1.427145e-07, -3.396447e-08),  # |k|=1.41e-06
    (+5.431343e-07, +1.042930e-08),  # |k|=2.83e-06
    (+7.489618e-07, +1.354621e-07),  # |k|=5.66e-06
    (+1.452758e-07, -3.778447e-07),  # |k|=1.13e-05
    (+2.885921e-07, -1.111624e-06),  # |k|=2.26e-05
    (+8.965188e-07, -2.004674e-07),  # |k|=4.53e-05
    (+4.101351e-06, -4.051475e-06),  # |k|=9.05e-05
    (+9.569035e-06, +6.952754e-06),  # |k|=1.81e-04
    (+2.719215e-05, +1.879194e-05),  # |k|=3.62e-04
    (+6.540348e-05, +5.125211e-05),  # |k|=7.24e-04
    (+9.715652e-05, +1.096853e-04),  # |k|=1.45e-03
    (+1.420559e-04, +1.219158e-05),  # |k|=2.90e-03
  ),
  (  # speed = 11.11 m/s
    (-1.762302e-09, +1.380873e-08),  # |k|=1.41e-06
    (+2.697306e-07, +1.259445e-07),  # |k|=2.83e-06
    (+2.218724e-07, -4.704494e-07),  # |k|=5.66e-06
    (+5.635340e-07, -7.995648e-08),  # |k|=1.13e-05
    (+1.071568e-06, -1.040560e-06),  # |k|=2.26e-05
    (+2.779170e-06, +3.065263e-07),  # |k|=4.53e-05
    (+7.289737e-06, -1.070434e-06),  # |k|=9.05e-05
    (+2.204574e-05, +5.631246e-06),  # |k|=1.81e-04
    (+4.858202e-05, +1.775555e-05),  # |k|=3.62e-04
    (+6.191315e-05, +1.576207e-05),  # |k|=7.24e-04
    (+6.072069e-05, +3.050890e-05),  # |k|=1.45e-03
    (+1.202678e-04, +3.100339e-05),  # |k|=2.90e-03
  ),
  (  # speed = 16.67 m/s
    (+1.756248e-08, -7.024984e-08),  # |k|=1.41e-06
    (+2.564414e-07, -1.964505e-07),  # |k|=2.83e-06
    (+3.056730e-07, -3.792345e-07),  # |k|=5.66e-06
    (+1.050670e-06, -9.174510e-07),  # |k|=1.13e-05
    (+1.930868e-06, -1.818639e-06),  # |k|=2.26e-05
    (+3.388845e-06, -2.023167e-06),  # |k|=4.53e-05
    (+8.272187e-06, -2.190581e-06),  # |k|=9.05e-05
    (+2.057696e-05, +3.971028e-06),  # |k|=1.81e-04
    (+4.099177e-05, +1.126012e-05),  # |k|=3.62e-04
    (+7.330204e-05, -5.198938e-06),  # |k|=7.24e-04
    (+8.046357e-05, +3.087738e-05),  # |k|=1.45e-03
    (+9.926449e-05, +4.009699e-05),  # |k|=2.90e-03
  ),
  (  # speed = 22.22 m/s
    (-6.222881e-08, -1.704959e-07),  # |k|=1.41e-06
    (+2.766983e-07, -2.354642e-07),  # |k|=2.83e-06
    (-1.575463e-08, -3.047067e-07),  # |k|=5.66e-06
    (+8.500766e-07, -4.655498e-07),  # |k|=1.13e-05
    (+2.205002e-06, -1.166728e-06),  # |k|=2.26e-05
    (+5.991450e-06, -7.761344e-08),  # |k|=4.53e-05
    (+1.637071e-05, +3.303160e-06),  # |k|=9.05e-05
    (+3.084618e-05, +1.186579e-05),  # |k|=1.81e-04
    (+5.815189e-05, +1.868452e-05),  # |k|=3.62e-04
    (+9.437995e-05, -1.219768e-06),  # |k|=7.24e-04
    (+9.506512e-05, +1.356242e-05),  # |k|=1.45e-03
    (+8.105617e-05, +1.263076e-05),  # |k|=2.90e-03
  ),
  (  # speed = 27.78 m/s
    (+9.661124e-08, -6.461406e-08),  # |k|=1.41e-06
    (+1.966445e-07, -1.087878e-07),  # |k|=2.83e-06
    (+5.762460e-07, -5.220558e-07),  # |k|=5.66e-06
    (+1.405264e-06, -7.721850e-07),  # |k|=1.13e-05
    (+3.412912e-06, -1.569406e-06),  # |k|=2.26e-05
    (+6.503637e-06, -1.188384e-06),  # |k|=4.53e-05
    (+1.581328e-05, +7.635552e-07),  # |k|=9.05e-05
    (+2.960176e-05, +6.514659e-06),  # |k|=1.81e-04
    (+4.924094e-05, +2.740677e-05),  # |k|=3.62e-04
    (+7.259327e-05, +5.904604e-05),  # |k|=7.24e-04
    (+1.328367e-04, +1.146048e-04),  # |k|=1.45e-03
    (+1.323321e-04, +6.341720e-05),  # |k|=2.90e-03
  ),
  (  # speed = 33.33 m/s
    (-5.159219e-08, -2.388877e-07),  # |k|=1.41e-06
    (+4.035936e-07, -3.465995e-07),  # |k|=2.83e-06
    (+9.233670e-07, -3.421156e-07),  # |k|=5.66e-06
    (+1.747271e-06, -9.649550e-07),  # |k|=1.13e-05
    (+3.406564e-06, -2.008435e-06),  # |k|=2.26e-05
    (+6.613473e-06, -3.906845e-06),  # |k|=4.53e-05
    (+1.247683e-05, -1.921811e-06),  # |k|=9.05e-05
    (+2.570613e-05, +4.675995e-06),  # |k|=1.81e-04
    (+6.229984e-05, +3.177673e-05),  # |k|=3.62e-04
    (+7.234882e-05, +5.891937e-05),  # |k|=7.24e-04
    (+1.859747e-04, +1.211024e-04),  # |k|=1.45e-03
    (+2.027611e-04, +4.901278e-04),  # |k|=2.90e-03
  ),
  (  # speed = 38.89 m/s
    (+6.848421e-08, -1.839270e-07),  # |k|=1.41e-06
    (+4.320401e-07, -3.727857e-07),  # |k|=2.83e-06
    (+7.800520e-07, -2.186693e-07),  # |k|=5.66e-06
    (+1.913601e-06, -8.642821e-07),  # |k|=1.13e-05
    (+3.742750e-06, -2.899729e-07),  # |k|=2.26e-05
    (+7.291372e-06, -9.031781e-07),  # |k|=4.53e-05
    (+1.806693e-05, +2.815382e-06),  # |k|=9.05e-05
    (+2.790954e-05, +9.359596e-06),  # |k|=1.81e-04
    (+6.104263e-05, +2.382664e-05),  # |k|=3.62e-04
    (+1.385769e-04, +7.244780e-05),  # |k|=7.24e-04
    (+1.698003e-04, +4.444760e-05),  # |k|=1.45e-03
    (+6.754986e-04, +6.754986e-04),  # |k|=2.90e-03
  ),
)


# 50%-of-|kappa| safety envelope (matches openpilot10 virtual/dynamic_steering)
RELATIVE_CAP_FULL_RATIO = 0.50

MIN_SPEED_MPS = SPEED_ANCHORS_MPS[0] * 0.5   # ~2.78 m/s = 10 km/h
KAPPA_BUCKET_MIN = KAPPA_BUCKET_EDGES[0]
KAPPA_BUCKET_MAX = KAPPA_BUCKET_EDGES[-1]


def _speed_interp(v_ego: float) -> Tuple[int, int, float]:
  """Return (low_idx, high_idx, alpha) for bilinear interpolation."""
  v = float(v_ego)
  if v <= SPEED_ANCHORS_MPS[0]:
    return 0, 0, 0.0
  if v >= SPEED_ANCHORS_MPS[-1]:
    last = len(SPEED_ANCHORS_MPS) - 1
    return last, last, 0.0
  # linear search is fine — only 7 anchors
  for high in range(1, len(SPEED_ANCHORS_MPS)):
    if SPEED_ANCHORS_MPS[high] >= v:
      low = high - 1
      span = SPEED_ANCHORS_MPS[high] - SPEED_ANCHORS_MPS[low]
      alpha = (v - SPEED_ANCHORS_MPS[low]) / max(span, 1e-6)
      if alpha < 0.0: alpha = 0.0
      if alpha > 1.0: alpha = 1.0
      return low, high, alpha
  return 0, 0, 0.0


def _kappa_bucket(abs_kappa: float) -> int:
  """Return bucket index in [0, 11], or -1 if out of range."""
  if abs_kappa < KAPPA_BUCKET_MIN or abs_kappa > KAPPA_BUCKET_MAX:
    return -1
  # bucket k contains [edges[k], edges[k+1])
  for k in range(len(KAPPA_BUCKET_EDGES) - 1):
    if KAPPA_BUCKET_EDGES[k] <= abs_kappa < KAPPA_BUCKET_EDGES[k + 1]:
      return k
  return len(KAPPA_BUCKET_EDGES) - 2  # last bucket if equal to max edge


def lookup(commanded_curvature: float, v_ego: float) -> float:
  """Return correction (rad/m) to ADD to commanded_curvature.

  Returns 0 outside the supported (speed, |kappa|) envelope. Result is
  bounded by 50% of |commanded_curvature|.
  """
  v = float(v_ego)
  k_cmd = float(commanded_curvature)
  abs_k = abs(k_cmd)
  if v < MIN_SPEED_MPS or abs_k < KAPPA_BUCKET_MIN:
    return 0.0

  bucket = _kappa_bucket(abs_k)
  if bucket < 0:
    return 0.0
  sign_idx = 0 if k_cmd >= 0.0 else 1

  low_s, high_s, alpha = _speed_interp(v)
  c_low = TABLE[low_s][bucket][sign_idx]
  c_high = TABLE[high_s][bucket][sign_idx]
  correction = (1.0 - alpha) * c_low + alpha * c_high

  cap = RELATIVE_CAP_FULL_RATIO * abs_k
  if correction > cap:
    correction = cap
  elif correction < -cap:
    correction = -cap
  return correction
