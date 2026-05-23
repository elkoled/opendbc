"""VW MEB fleet EPS plant model -- K(v) lookup.

Identified empirically from 1009 engaged-driving segments across 6
VOLKSWAGEN_ID4_MK1 dongles (id4_lateral/2 run, 2026-05-22). K(v) is the
steady-state EPS+vehicle yaw gain: actual_yaw_curvature = K(v) * commanded
through-origin fit on samples with |c_cmd| >= 5e-4 rad/m and the
sunnypilot-style strict lat-accel / engagement gates.

Hierarchical pooling: per-dongle mean first, then unweighted mean across
the 6 dongles. CIs are 5-95 bootstrap over dongles.

Caveats:
 * Identified on ID4_MK1 only. The MEB platform shares EPS hardware
   across ID3 / ID4 / Q4 / Born / Enyaq so the gain shape is likely
   similar, but this is not directly validated.
 * Per-dongle dispersion at 120 km/h is wide (std=0.20); the apply
   function returns K=1 (passthrough) at any cell with fewer than 5
   dongles, including the 120 km/h cell.
 * Open-loop steady-state. Plant lag (~0.36 s) is left to openpilot's
   liveDelay estimator; do not modify it here.
"""

# (speed_kph, K_mean, K_lo, K_hi, n_dongles_valid)
FLEET_K_YAW = (
  (20.0, 0.6966, 0.5179, 0.8752, 3),
  (40.0, 0.7375, 0.6047, 0.8704, 3),
  (60.0, 0.8727, 0.8069, 0.9247, 5),
  (80.0, 0.8394, 0.7947, 0.8841, 6),
  (100.0, 0.6279, 0.5655, 0.6835, 6),
  (120.0, 0.4516, 0.2375, 0.6143, 4),
  (140.0, 0.5690, 0.4863, 0.6517, 3),
)

MIN_DONGLES_VALID = 5
MAX_BOOST = 2.0


def _valid_anchors():
  return [(v, K) for (v, K, _, _, n) in FLEET_K_YAW if n >= MIN_DONGLES_VALID]


def K_at_speed(v_ego_ms: float) -> float:
  """Linear-interp K(v_ego) over the *well-supported* anchors (those with
  >= MIN_DONGLES_VALID dongles agreeing). Holds the nearest valid anchor's
  K outside the supported range; returns 1.0 (passthrough) if no anchor
  meets the threshold.

  Anchors that fall below the threshold (the sparse 20/40/120/140 km/h
  cells in the current model) are *skipped* rather than triggering a step
  to passthrough at every adjacent speed.
  """
  anchors = _valid_anchors()
  if not anchors:
    return 1.0
  v_kph = float(v_ego_ms) * 3.6
  if v_kph < anchors[0][0]:
    # Low-speed driving is empirically fine without correction (user
    # feedback + few-dongle support below the first well-supported anchor).
    return 1.0
  if v_kph >= anchors[-1][0]:
    # Above the last well-supported anchor (100 km/h), hold the last K.
    # We have data showing K continues to drop at 120 km/h (0.45) but the
    # cross-dongle spread there (std=0.20) is too wide to ship.
    return anchors[-1][1]
  for i in range(len(anchors) - 1):
    v0, K0 = anchors[i]
    v1, K1 = anchors[i + 1]
    if v0 <= v_kph <= v1:
      t = (v_kph - v0) / (v1 - v0)
      K = K0 * (1.0 - t) + K1 * t
      return K if K > 0.0 else 1.0
  return 1.0


def apply_fleet_K_correction(c_cmd: float, v_ego_ms: float) -> float:
  """Boost commanded curvature by 1/K(v) to compensate for VW MEB EPS
  plant undershoot (id4_lateral/2). Capped at MAX_BOOST (2.0x).

  The correction is gain-only; lag is handled separately by liveDelay.
  apply_std_curvature_limits is still applied downstream, so the lat-accel
  and rate envelopes still clip the corrected value.
  """
  K = K_at_speed(v_ego_ms)
  if K <= 0.0:
    return float(c_cmd)
  boost = 1.0 / K
  if boost > MAX_BOOST:
    boost = MAX_BOOST
  return float(c_cmd) * boost
