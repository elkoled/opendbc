#!/usr/bin/env python3
# Fits an EPS model to the power test logs, per speed band and steering power:
#   meas[k+1] = meas[k] + clip(DT / tau * (gain * cmd[k - delay] - meas[k]), +-rate * DT)
# usage: power_test_fit.py <log dir or csv files>
import argparse
import csv
import glob
import os
from collections import defaultdict

import numpy as np

from opendbc.car.volkswagen.power_test import DT, LOG_DIR

DEG_PER_CURVATURE = 2600.  # about, ID.4 steering wheel deg per 1/m at low speed
MAX_DELAY = 8  # steering messages


def load(paths: list[str]) -> list[dict]:
  files = []
  for p in paths:
    files += sorted(glob.glob(os.path.join(p, "*.csv"))) if os.path.isdir(p) else [p]
  runs = []
  for fn in files:
    with open(fn) as f:
      rows = list(csv.DictReader(f))
    # split into contiguous runs of one step
    run: list[dict] = []
    for r in rows:
      if run and (r["step"] != run[-1]["step"] or r["band"] != run[-1]["band"] or float(r["t"]) - float(run[-1]["t"]) > 3 * DT):
        runs.append(run)
        run = []
      run.append(r)
    if run:
      runs.append(run)
  return runs


TAUS = np.geomspace(0.04, 0.8, 40)
GAINS = np.linspace(0.7, 1.2, 51)


def simulate(cmd: np.ndarray, meas0: float, delay: int, tau: np.ndarray, gain: np.ndarray, rate: float) -> np.ndarray:
  # every (tau, gain) at once
  y = np.empty((len(cmd),) + tau.shape)
  y[0] = meas0
  for k in range(len(cmd) - 1):
    u = cmd[max(k - delay, 0)]
    y[k + 1] = y[k] + np.clip(DT / tau * (gain * u - y[k]), -rate * DT, rate * DT)
  return y


def fit(segments: list[tuple[np.ndarray, np.ndarray]], rate: float | None = None) -> dict | None:
  if not segments or sum(len(c) for c, _ in segments) < 30:
    return None
  if rate is None:
    # the rate limit shows as the top of the measured rate
    dy = np.concatenate([np.diff(m) / DT for _, m in segments])
    rate = float(np.percentile(np.abs(dy), 99.5)) * 1.02
  tau, gain = np.meshgrid(TAUS, GAINS, indexing="ij")
  tot = sum(float(np.sum((m - m.mean()) ** 2)) for _, m in segments)
  best = None
  for delay in range(MAX_DELAY + 1):
    err = sum(np.sum((simulate(c, m[0], delay, tau, gain, rate) - m[:, None, None]) ** 2, axis=0) for c, m in segments)
    i, j = np.unravel_index(np.argmin(err), err.shape)
    if best is None or err[i, j] < best["err"]:
      # the rate is only the EPS limit if the model asked for more for a while
      t, g = TAUS[i], GAINS[j]
      sat = sum(int(np.sum(np.abs(g * (np.r_[np.full(delay, c[0]), c[:len(c) - delay]] if delay else c) - m) / t > rate)) for c, m in segments)
      best = {"delay_ms": delay * DT * 1000, "tau": float(t), "gain": float(g), "rate_deg_s": rate * DEG_PER_CURVATURE, "saturated": sat >= 10,
              "err": float(err[i, j]), "r2": 1. - float(err[i, j]) / max(tot, 1e-12), "n": int(sum(len(c) for c, _ in segments))}
  return best


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("paths", nargs="*", default=[LOG_DIR])
  args = parser.parse_args()

  groups: dict[tuple[float, int], list] = defaultdict(list)
  faults = []
  for run in load(args.paths):
    if any(r["hca_status"] in ("FAULT", "REJECTED", "PREEMPTED", "DISABLED") for r in run):
      r = run[-1]
      faults.append(f"band {r['band']} m/s {r['phase']} {r['maneuver']} power {r['power_sent']}: {r['hca_status']}")
    # only the maneuver itself at its target power, recentering and ramps excluded
    rows = [r for r in run if r["maneuver"] in ("small", "big", "turn") and r["power_sent"] == r["target_power"]]
    if len(rows) < 25:
      continue
    cmd = np.array([float(r["curvature_sent"]) for r in rows])
    meas = np.array([float(r["curvature_meas"]) for r in rows])
    groups[(float(rows[0]["band"]), int(rows[0]["target_power"]))].append((cmd, meas))

  print(f"{'band m/s':>8} {'power':>5} {'delay ms':>8} {'tau s':>6} {'gain':>5} {'rate deg/s':>10} {'r2':>5} {'samples':>7}")
  for (band, power), segs in sorted(groups.items()):
    f = fit(segs)
    if f is None:
      continue
    rate = ("" if f["saturated"] else ">=") + f"{f['rate_deg_s']:.0f}"
    print(f"{band:8.1f} {power:5d} {f['delay_ms']:8.0f} {f['tau']:6.3f} {f['gain']:5.2f} {rate:>10} {f['r2']:5.2f} {f['n']:7d}")
  for line in faults:
    print("fault:", line)


if __name__ == "__main__":
  main()
