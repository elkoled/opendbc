"""
Static EPS correction table for VOLKSWAGEN_ID4_MK1 (MEB lateral control).

Fitted from 1,081 openpilot-engaged segments across 7 distinct ID4_MK1 dongles
(`~/eps_seglist.csv`, 2026-05-22). The table stores the per-cell mean
*projected residual* `direction * (kappa_desired - kappa_actual_pose)` observed
over a 7-speed x 12-curvature-magnitude x 2-sign grid (transcribed from
~/openpilot10 `virtual/dynamic_steering` learner).

USE NOTES — read these before assuming this corrects fleet tracking:

  * Validated by leave-one-dongle-out CV:
      - Median LOO RMSE reduction across 6 measurable dongles: +4.8%
      - Worst-case LOO outcome:           -11.0% (one dongle: tracking WORSE)
      - Highway-only (>=80 km/h) LOO:      one dongle degraded -67.9% RMSE
      - 5 of 7 dongles helped; 1 of 7 measurably worsened
  * The "per-VIN residual is real, but does NOT generalize across cars" result
    is documented in /home/batman/lateral_fleet_out/model/FINDINGS.md.
  * The honest fix is a per-VIN online learner (port of openpilot10
    `virtual/dynamic_steering`). This table is the static stop-gap.
  * GATED TO ID4_MK1 ONLY. Other MEB platforms (ID3/Atlas/Q4/Born/Enyaq) have
    no training data and would be guessing; lookup returns 0 there.

Safety:
  * 50% relative cap on the applied correction matches the openpilot10
    learner: |correction| <= 0.5 * |kappa_desired|.
  * Returns 0 below MIN_SPEED (lat-accel gate undefined at standstill).
  * Returns 0 for |kappa| outside [1e-6, 4.096e-3] (table grid edges).
"""
from __future__ import annotations

# Grid (matches openpilot10/selfdrive/locationd/curvatured.py)
SPEED_ANCHORS_MPS = (
  5.555555820465088,  # 20 km/h
  11.111111640930176,  # 40
  16.666667938232422,  # 60
  22.22222328186035,   # 80
  27.77777862548828,   # 100
  33.333335876464844,  # 120
  38.88888931274414,   # 140
)

CURVATURE_BUCKET_EDGES = (
  1.0e-6, 2.0e-6, 4.0e-6, 8.0e-6, 1.6e-5, 3.2e-5, 6.4e-5,
  1.28e-4, 2.56e-4, 5.12e-4, 1.024e-3, 2.048e-3, 4.096e-3,
)

CURVATURE_BUCKET_MIN = CURVATURE_BUCKET_EDGES[0]
CURVATURE_BUCKET_MAX = CURVATURE_BUCKET_EDGES[-1]
MIN_SPEED_MPS = SPEED_ANCHORS_MPS[0] * 0.5  # 2.78 m/s
RELATIVE_CAP = 0.5                          # 50% of |kappa_desired|, mirrors learner

# Table shape: 7 speeds x 12 |kappa| buckets x 2 signs (idx 0 = kappa>=0, idx 1 = kappa<0).
# Value = mean(direction * (kappa_desired - kappa_actual_pose)) on training fleet,
# clamped to +/- RELATIVE_CAP * kappa_bucket_center per cell at fit time.
TABLE = (
  (  # speed_idx=0  v=5.56 m/s
    (+1.4271451e-07, -3.3964468e-08), (+5.4313428e-07, +1.0429296e-08),
    (+7.4896177e-07, +1.3546209e-07), (+1.4527584e-07, -3.7784472e-07),
    (+2.8859210e-07, -1.1116243e-06), (+8.9651882e-07, -2.0046740e-07),
    (+4.1013511e-06, -4.0514754e-06), (+9.5690347e-06, +6.9527535e-06),
    (+2.7192147e-05, +1.8791937e-05), (+6.5403476e-05, +5.1252108e-05),
    (+9.7156525e-05, +1.0968532e-04), (+1.4205586e-04, +1.2191582e-05),
  ),
  (  # speed_idx=1  v=11.11 m/s
    (-1.7623020e-09, +1.3808727e-08), (+2.6973065e-07, +1.2594452e-07),
    (+2.2187236e-07, -4.7044943e-07), (+5.6353401e-07, -7.9956476e-08),
    (+1.0715680e-06, -1.0405604e-06), (+2.7791697e-06, +3.0652625e-07),
    (+7.2897373e-06, -1.0704337e-06), (+2.2045739e-05, +5.6312462e-06),
    (+4.8582018e-05, +1.7755552e-05), (+6.1913149e-05, +1.5762072e-05),
    (+6.0720687e-05, +3.0508897e-05), (+1.2026778e-04, +3.1003391e-05),
  ),
  (  # speed_idx=2  v=16.67 m/s
    (+1.7562478e-08, -7.0249839e-08), (+2.5644141e-07, -1.9645049e-07),
    (+3.0567304e-07, -3.7923450e-07), (+1.0506700e-06, -9.1745099e-07),
    (+1.9308676e-06, -1.8186393e-06), (+3.3888453e-06, -2.0231668e-06),
    (+8.2721867e-06, -2.1905812e-06), (+2.0576964e-05, +3.9710277e-06),
    (+4.0991771e-05, +1.1260121e-05), (+7.3302042e-05, -5.1989381e-06),
    (+8.0463571e-05, +3.0877376e-05), (+9.9264485e-05, +4.0096991e-05),
  ),
  (  # speed_idx=3  v=22.22 m/s
    (-6.2228806e-08, -1.7049594e-07), (+2.7669829e-07, -2.3546420e-07),
    (-1.5754630e-08, -3.0470670e-07), (+8.5007664e-07, -4.6554979e-07),
    (+2.2050016e-06, -1.1667285e-06), (+5.9914503e-06, -7.7613437e-08),
    (+1.6370706e-05, +3.3031601e-06), (+3.0846180e-05, +1.1865785e-05),
    (+5.8151890e-05, +1.8684516e-05), (+9.4379946e-05, -1.2197682e-06),
    (+9.5065116e-05, +1.3562418e-05), (+8.1056172e-05, +1.2630763e-05),
  ),
  (  # speed_idx=4  v=27.78 m/s
    (+9.6611236e-08, -6.4614057e-08), (+1.9664450e-07, -1.0878780e-07),
    (+5.7624604e-07, -5.2205575e-07), (+1.4052637e-06, -7.7218501e-07),
    (+3.4129117e-06, -1.5694062e-06), (+6.5036369e-06, -1.1883837e-06),
    (+1.5813284e-05, +7.6355518e-07), (+2.9601762e-05, +6.5146585e-06),
    (+4.9240944e-05, +2.7406773e-05), (+7.2593269e-05, +5.9046037e-05),
    (+1.3283666e-04, +1.1460478e-04), (+1.3233206e-04, +6.3417201e-05),
  ),
  (  # speed_idx=5  v=33.33 m/s
    (-5.1592186e-08, -2.3888769e-07), (+4.0359360e-07, -3.4659955e-07),
    (+9.2336697e-07, -3.4211559e-07), (+1.7472711e-06, -9.6495497e-07),
    (+3.4065643e-06, -2.0084348e-06), (+6.6134728e-06, -3.9068445e-06),
    (+1.2476830e-05, -1.9218107e-06), (+2.5706132e-05, +4.6759947e-06),
    (+6.2299839e-05, +3.1776732e-05), (+7.2348815e-05, +5.8919370e-05),
    (+1.8597467e-04, +1.2110240e-04), (+2.0276112e-04, +4.9012777e-04),
  ),
  (  # speed_idx=6  v=38.89 m/s
    (+6.8484210e-08, -1.8392704e-07), (+4.3204013e-07, -3.7278571e-07),
    (+7.8005197e-07, -2.1866935e-07), (+1.9136010e-06, -8.6428207e-07),
    (+3.7427499e-06, -2.8997291e-07), (+7.2913724e-06, -9.0317809e-07),
    (+1.8066931e-05, +2.8153824e-06), (+2.7909536e-05, +9.3595963e-06),
    (+6.1042632e-05, +2.3826644e-05), (+1.3857688e-04, +7.2447799e-05),
    (+1.6980028e-04, +4.4447596e-05), (+6.7549858e-04, +6.7549858e-04),
  ),
)

N_SPEED = len(SPEED_ANCHORS_MPS)
N_KAPPA = len(CURVATURE_BUCKET_EDGES) - 1
SUPPORTED_FINGERPRINTS = ("VOLKSWAGEN_ID4_MK1",)


def _speed_index(v_ego: float) -> int:
  """Nearest speed anchor (caller has already checked MIN_SPEED_MPS)."""
  best_i = 0
  best_d = abs(SPEED_ANCHORS_MPS[0] - v_ego)
  for i in range(1, N_SPEED):
    d = abs(SPEED_ANCHORS_MPS[i] - v_ego)
    if d < best_d:
      best_d = d
      best_i = i
  return best_i


def _curvature_index(abs_kappa: float) -> int | None:
  if abs_kappa < CURVATURE_BUCKET_MIN or abs_kappa > CURVATURE_BUCKET_MAX:
    return None
  # find rightmost edge <= abs_kappa, returning bucket index in [0, N_KAPPA-1]
  for i in range(N_KAPPA, 0, -1):
    if abs_kappa >= CURVATURE_BUCKET_EDGES[i - 1]:
      return i - 1 if abs_kappa < CURVATURE_BUCKET_EDGES[i] else min(i, N_KAPPA - 1)
  return 0


class EPSCorrection:
  """Per-fingerprint static correction lookup, with 50% relative-cap safety.

  Returns 0 for any platform not in SUPPORTED_FINGERPRINTS.  Returns 0 below
  MIN_SPEED_MPS, and 0 for |kappa| outside the table's [1e-6, 4.096e-3] grid.
  """
  def __init__(self, car_fingerprint: str):
    self.enabled = car_fingerprint in SUPPORTED_FINGERPRINTS

  def lookup(self, desired_curvature: float, v_ego: float) -> float:
    if not self.enabled:
      return 0.0
    if v_ego < MIN_SPEED_MPS:
      return 0.0
    abs_k = abs(desired_curvature)
    k_idx = _curvature_index(abs_k)
    if k_idx is None:
      return 0.0
    s_idx = _speed_index(v_ego)
    sign_idx = 0 if desired_curvature >= 0.0 else 1
    direction = 1.0 if desired_curvature >= 0.0 else -1.0
    cell = TABLE[s_idx][k_idx][sign_idx]
    correction = direction * cell
    # 50% relative cap mirrors openpilot10 learner safety envelope.
    cap = RELATIVE_CAP * abs_k
    if correction > cap:
      correction = cap
    elif correction < -cap:
      correction = -cap
    return correction
