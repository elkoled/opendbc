#!/usr/bin/env python3
import argparse
import json
import re
import shutil
import subprocess
import sys
import tempfile
from collections import defaultdict
from pathlib import Path


def run_git(cmd, cwd):
  r = subprocess.run(["git"] + cmd, cwd=cwd, capture_output=True, text=True)
  if r.returncode != 0:
    raise RuntimeError(f"git {' '.join(cmd)}: {r.stderr}")
  return r.stdout.strip()


def get_changed_platforms(cwd, database):
  changed = run_git(["diff", "--name-only", "origin/master...HEAD"], cwd=cwd)
  brands = set()
  for line in changed.splitlines():
    if m := re.search(r"opendbc/car/(\w+)/", line):
      brands.add(m.group(1))
    if m := re.search(r"opendbc/dbc/(\w+?)_", line):
      brands.add(m.group(1).lower())
  return [p for p in database if any(b.upper() in p for b in brands)]


def get_database():
  import requests
  return requests.get("https://huggingface.co/datasets/commaai/commaCarSegments/raw/main/database.json").json()


def run_worker(platforms, segments, ref_path, update, cwd, worker_path, workers=8):
  cmd = [sys.executable, worker_path,
         "--platforms", json.dumps(platforms),
         "--segments", json.dumps(segments),
         "--ref-path", ref_path,
         "--workers", str(workers)]
  if update:
    cmd.append("--update")

  r = subprocess.run(cmd, capture_output=True, text=True, cwd=cwd)
  print(r.stdout)
  if r.stderr:
    print(r.stderr, file=sys.stderr)

  if "RESULTS:" in r.stdout:
    return json.loads(r.stdout.split("RESULTS:")[1].strip())
  return []


def format_diff(diffs, total_frames=None):
  if not diffs:
    return []
  if not all(isinstance(d[2], bool) and isinstance(d[3], bool) for d in diffs):
    return [f"    frame {d[1]}: {d[2]} -> {d[3]}" for d in diffs[:10]]

  lines = []
  ranges, cur = [], [diffs[0]]
  for d in diffs[1:]:
    if d[1] <= cur[-1][1] + 15:
      cur.append(d)
    else:
      ranges.append(cur)
      cur = [d]
  ranges.append(cur)

  for rdiffs in ranges:
    t0, t1 = max(0, rdiffs[0][1] - 5), rdiffs[-1][1] + 6
    diff_map = {d[1]: d for d in rdiffs}

    b_vals, m_vals, ts_map = [], [], {}
    b_st, m_st = False, False
    for f in range(t0, t1):
      if f in diff_map:
        b_st, m_st = diff_map[f][2], diff_map[f][3]
        if len(diff_map[f]) > 4:
          ts_map[f] = diff_map[f][4]
      else:
        prev = [d for d in rdiffs if d[1] < f]
        if prev and prev[-1][2] != prev[-1][3]:
          b_st = m_st = prev[-1][2] or prev[-1][3]
      b_vals.append(b_st)
      m_vals.append(m_st)

    # Get timestamps from diff data (nanoseconds -> seconds)
    ts_start = ts_map.get(t0, rdiffs[0][4] if len(rdiffs[0]) > 4 else 0)
    ts_end = ts_map.get(t1 - 1, rdiffs[-1][4] if len(rdiffs[-1]) > 4 else 0)
    t0_sec = ts_start / 1e9
    t1_sec = ts_end / 1e9

    # Calculate ms per frame from actual timestamps
    if len(ts_map) >= 2:
      ts_vals = sorted(ts_map.items())
      frame_ms = (ts_vals[-1][1] - ts_vals[0][1]) / 1e6 / (ts_vals[-1][0] - ts_vals[0][0])
    else:
      frame_ms = 10  # fallback

    lines.append(f"\n  frames {t0}-{t1-1} (t={t0_sec:.2f}s - {t1_sec:.2f}s)")
    pad = 12
    for label, vals in [("master", b_vals), ("PR", m_vals)]:
      top, bot = " " * pad, f"  {label}:".ljust(pad)
      for i, v in enumerate(vals):
        pv = vals[i - 1] if i > 0 else False
        if v and not pv:
          top += "┌"
          bot += "┘"
        elif not v and pv:
          top += "┐"
          bot += "└"
        elif v:
          top += "─"
          bot += " "
        else:
          top += " "
          bot += "─"
      lines.extend([top, bot])

    b_rises = [i for i, v in enumerate(b_vals) if v and (i == 0 or not b_vals[i - 1])]
    m_rises = [i for i, v in enumerate(m_vals) if v and (i == 0 or not m_vals[i - 1])]
    b_falls = [i for i, v in enumerate(b_vals) if not v and i > 0 and b_vals[i - 1]]
    m_falls = [i for i, v in enumerate(m_vals) if not v and i > 0 and m_vals[i - 1]]

    ann_lines = []
    if b_rises and m_rises:
      delta = m_rises[0] - b_rises[0]
      if delta:
        ms = int(abs(delta) * frame_ms)
        direction = "lags" if delta > 0 else "leads"
        pos = min(b_rises[0], m_rises[0])
        arrows = "↑" * (abs(delta) + 1)
        ann_lines.append((" " * (pad + pos) + arrows, f"rise: PR {direction} by {abs(delta)} frames ({ms}ms)"))
    if b_falls and m_falls:
      delta = m_falls[0] - b_falls[0]
      if delta:
        ms = int(abs(delta) * frame_ms)
        direction = "lags" if delta > 0 else "leads"
        pos = min(b_falls[0], m_falls[0])
        arrows = "↑" * (abs(delta) + 1)
        ann_lines.append((" " * (pad + pos) + arrows, f"fall: PR {direction} by {abs(delta)} frames ({ms}ms)"))

    for arrow, desc in ann_lines:
      lines.append(arrow)
      lines.append(" " * pad + "^ " + desc)

  return lines


def main(platform=None, segments_per_platform=10):
  cwd = Path(__file__).resolve().parents[4]
  ref_path = tempfile.mkdtemp(prefix="car_ref_")

  worker_src = Path(__file__).parent / "worker.py"
  worker_tmp = Path(ref_path) / "worker.py"
  shutil.copy(worker_src, worker_tmp)

  print(f"{'=' * 60}\nComparing HEAD vs origin/master\n{'=' * 60}\n")

  database = get_database()
  platforms = [platform] if platform else get_changed_platforms(cwd, database)[:10]
  if not platforms:
    print("No platforms detected from changes")
    return 0

  print(f"Platforms: {', '.join(platforms)}\n")
  segments = {p: database.get(p, [])[:segments_per_platform] for p in platforms}
  print(f"Testing {sum(len(s) for s in segments.values())} segments...\n")

  head = run_git(["rev-parse", "HEAD"], cwd=cwd)

  try:
    print("Generating baseline on origin/master...")
    run_git(["checkout", "origin/master"], cwd=cwd)
    run_worker(platforms, segments, ref_path, True, cwd, str(worker_tmp))

    print("\nTesting HEAD...")
    run_git(["checkout", head], cwd=cwd)
    results = run_worker(platforms, segments, ref_path, False, cwd, str(worker_tmp))

    passed = [(p, s) for p, s, d, e, n in results if not d and not e]
    with_diffs = [(p, s, d, n) for p, s, d, e, n in results if d]
    errors = [(p, s, e) for p, s, d, e, n in results if e]

    print(f"\n{'=' * 60}")
    print(f"Results: {len(passed)} passed, {len(with_diffs)} with diffs, {len(errors)} errors")

    for plat, seg, diffs, total_frames in with_diffs:
      print(f"\n{plat} - {seg}")
      by_field = defaultdict(list)
      for d in diffs:
        by_field[d[0]].append(d)
      for field, fd in sorted(by_field.items()):
        print(f"  {field}: {len(fd)} diffs")
        for line in format_diff(fd, total_frames):
          print(line)

    return 0
  finally:
    run_git(["checkout", head], cwd=cwd)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--platform")
  parser.add_argument("--segments-per-platform", type=int, default=10)
  args = parser.parse_args()
  sys.exit(main(args.platform, args.segments_per_platform))
