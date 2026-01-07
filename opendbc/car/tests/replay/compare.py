#!/usr/bin/env python3
"""
Compare CarState outputs between HEAD and origin/master.
Auto-detects affected platforms from changed files and runs replay on segments.
"""
import argparse
import json
import re
import subprocess
import sys
import tempfile
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass
class Diff:
  field: str
  frame: int
  old_value: Any
  new_value: Any


def format_waveform(diffs: list[Diff], context: int = 5) -> list[str]:
  """Format boolean diffs as ASCII waveform visualization"""
  if not diffs or not all(isinstance(d.old_value, bool) and isinstance(d.new_value, bool) for d in diffs):
    return [f"    frame {d.frame}: {d.old_value} -> {d.new_value}" for d in diffs[:10]]

  # Group nearby diffs into ranges
  ranges, cur = [], [diffs[0]]
  for d in diffs[1:]:
    if d.frame <= cur[-1].frame + 15:
      cur.append(d)
    else:
      ranges.append(cur)
      cur = [d]
  ranges.append(cur)

  lines = []
  for rdiffs in ranges:
    t0 = max(0, rdiffs[0].frame - context)
    t1 = rdiffs[-1].frame + context + 1
    diff_map = {d.frame: d for d in rdiffs}

    # Reconstruct signal values
    m_vals, p_vals = [], []
    m_st, p_st = False, False
    for f in range(t0, t1):
      if f in diff_map:
        m_st, p_st = diff_map[f].old_value, diff_map[f].new_value
      else:
        prev = [d for d in rdiffs if d.frame < f]
        if prev and prev[-1].old_value != prev[-1].new_value:
          m_st = p_st = prev[-1].old_value or prev[-1].new_value
      m_vals.append(m_st)
      p_vals.append(p_st)

    lines.append(f"\n    frames {t0}-{t1-1}:")

    # Draw waveforms
    for label, vals in [("master", m_vals), ("PR", p_vals)]:
      top = " " * 12
      bot = f"    {label}:".ljust(12)
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

    # Count edges and annotate differences
    m_rises = [i for i, v in enumerate(m_vals) if v and (i == 0 or not m_vals[i - 1])]
    m_falls = [i for i, v in enumerate(m_vals) if not v and i > 0 and m_vals[i - 1]]
    p_rises = [i for i, v in enumerate(p_vals) if v and (i == 0 or not p_vals[i - 1])]
    p_falls = [i for i, v in enumerate(p_vals) if not v and i > 0 and p_vals[i - 1]]

    ann = []
    if m_rises and p_rises:
      d = p_rises[0] - m_rises[0]
      if d:
        ann.append(f"{'+'if d>0 else ''}{d} frames")
    if len(m_rises) != len(p_rises) or len(m_falls) != len(p_falls):
      m_edges = len(m_rises) + len(m_falls)
      p_edges = len(p_rises) + len(p_falls)
      if m_edges > p_edges:
        ann.append(f"master: {m_edges - p_edges} extra edge(s)")
      elif p_edges > m_edges:
        ann.append(f"PR: {p_edges - m_edges} extra edge(s)")

    if ann:
      pos = min(m_rises[0] if m_rises else 0, p_rises[0] if p_rises else 0)
      lines.append(" " * 12 + " " * pos + "↑")
      lines.append(" " * 12 + " " * pos + ", ".join(ann))

  return lines


def run_git(cmd: list[str], cwd: Path) -> str:
  r = subprocess.run(["git"] + cmd, cwd=cwd, capture_output=True, text=True)
  if r.returncode != 0:
    raise RuntimeError(f"git {' '.join(cmd)}: {r.stderr}")
  return r.stdout.strip()


def get_changed_platforms(cwd: Path, database: dict) -> list[str]:
  """Detect platforms affected by changes between HEAD and origin/master"""
  changed = run_git(["diff", "--name-only", "origin/master...HEAD"], cwd=cwd)
  brands = set()
  for line in changed.splitlines():
    if m := re.search(r"opendbc/car/(\w+)/", line):
      brands.add(m.group(1))
    if m := re.search(r"opendbc/dbc/(\w+?)_", line):
      brands.add(m.group(1).lower())
  return [p for p in database if any(b.upper() in p for b in brands)]


def get_database() -> dict:
  import requests
  return requests.get("https://huggingface.co/datasets/commaai/commaCarSegments/raw/main/database.json").json()


def run_worker(platforms: list, segments: dict, ref_path: str, update: bool, cwd: Path, worker_path: str, workers: int = 8) -> list:
  """Run worker.py in subprocess to process segments with fresh imports"""
  cmd = [
    sys.executable, worker_path,
    "--platforms", json.dumps(platforms),
    "--segments", json.dumps(segments),
    "--ref-path", ref_path,
    "--workers", str(workers),
  ]
  if update:
    cmd.append("--update")

  r = subprocess.run(cmd, capture_output=True, text=True, cwd=cwd)
  print(r.stdout)
  if r.stderr:
    print(r.stderr, file=sys.stderr)

  if "RESULTS:" in r.stdout:
    data = json.loads(r.stdout.split("RESULTS:")[1].strip())
    return [(p, s, [Diff(d[0], d[1], d[2], d[3]) for d in diffs], e) for p, s, diffs, e in data]
  return []


def test_replay(platform: str | None = None, segments_per_platform: int = 10) -> int:
  cwd = Path(__file__).resolve().parents[2]  # opendbc/car
  ref_path = tempfile.mkdtemp(prefix="car_ref_")

  # Copy worker.py to temp location before switching to master
  import shutil
  worker_src = Path(__file__).parent / "worker.py"
  worker_tmp = Path(ref_path) / "worker.py"
  shutil.copy(worker_src, worker_tmp)
  worker_path = str(worker_tmp)

  print(f"{'='*60}\nComparing HEAD vs origin/master\n{'='*60}\n")

  database = get_database()
  if platform:
    platforms = [platform]
  else:
    platforms = get_changed_platforms(cwd, database)[:10]
    if not platforms:
      print("No platforms detected from changes")
      return 1
  print(f"Platforms: {', '.join(platforms)}\n")

  segments = {p: database.get(p, [])[:segments_per_platform] for p in platforms}
  total = sum(len(s) for s in segments.values())
  print(f"Testing {total} segments...\n")

  head = run_git(["rev-parse", "HEAD"], cwd=cwd)

  try:
    print("Generating baseline on origin/master...")
    run_git(["checkout", "origin/master"], cwd=cwd)
    run_worker(platforms, segments, ref_path, update=True, cwd=cwd, worker_path=worker_path)

    print("\nTesting HEAD...")
    run_git(["checkout", head], cwd=cwd)
    results = run_worker(platforms, segments, ref_path, update=False, cwd=cwd, worker_path=worker_path)

    with_diffs = [(p, s, d) for p, s, d, e in results if d]
    errors = [(p, s, e) for p, s, d, e in results if e]

    print(f"\n{'='*60}")
    print(f"Results: {len(results)-len(with_diffs)-len(errors)} passed, {len(with_diffs)} with diffs, {len(errors)} errors")

    if with_diffs:
      print("\nDifferences:")
      for plat, seg, diffs in with_diffs:
        print(f"\n{plat} - {seg}")
        by_field = defaultdict(list)
        for d in diffs:
          by_field[d.field].append(d)
        for field, fd in sorted(by_field.items()):
          print(f"  {field}: {len(fd)} diffs")
          for line in format_waveform(fd):
            print(line)

    return 1 if with_diffs else 0
  finally:
    run_git(["checkout", head], cwd=cwd)


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Compare CarState outputs between HEAD and origin/master")
  parser.add_argument("--platform", help="Test specific platform instead of auto-detecting")
  parser.add_argument("--segments-per-platform", type=int, default=10)
  args = parser.parse_args()
  sys.exit(test_replay(args.platform, args.segments_per_platform))
