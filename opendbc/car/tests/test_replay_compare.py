#!/usr/bin/env python3
"""
opendbc car behavior replay - Compare CarState outputs before/after PR changes.
Compares HEAD vs origin/master to detect behavior differences.
"""
import argparse, json, re, subprocess, sys, tempfile
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

CARSTATE_FIELDS = [
  "vEgo", "aEgo", "standstill", "steeringAngleDeg", "steeringTorque", "steeringPressed",
  "gas", "gasPressed", "brake", "brakePressed", "gearShifter", "leftBlinker", "rightBlinker",
  "cruiseState.enabled", "cruiseState.speed", "cruiseState.available", "doorOpen", "seatbeltUnlatched",
]

@dataclass
class Diff:
  field: str
  frame: int
  old_value: Any
  new_value: Any

def format_waveform(diffs: list[Diff], context: int = 5) -> list[str]:
  if not diffs or not all(isinstance(d.old_value, bool) and isinstance(d.new_value, bool) for d in diffs):
    return [f"    frame {d.frame}: {d.old_value} -> {d.new_value}" for d in diffs[:10]]
  lines, ranges, cur = [], [], [diffs[0]]
  for d in diffs[1:]:
    if d.frame <= cur[-1].frame + 15: cur.append(d)
    else: ranges.append(cur); cur = [d]
  ranges.append(cur)
  for rdiffs in ranges:
    t0, t1 = max(0, rdiffs[0].frame - context), rdiffs[-1].frame + context + 1
    diff_map = {d.frame: d for d in rdiffs}
    m_vals, p_vals, m_st, p_st = [], [], False, False
    for f in range(t0, t1):
      if f in diff_map: m_st, p_st = diff_map[f].old_value, diff_map[f].new_value
      else:
        prev = [d for d in rdiffs if d.frame < f]
        if prev and prev[-1].old_value != prev[-1].new_value:
          m_st = p_st = prev[-1].old_value or prev[-1].new_value
      m_vals.append(m_st); p_vals.append(p_st)
    lines.append(f"\n    frames {t0}-{t1-1}:")
    for label, vals in [("master", m_vals), ("PR", p_vals)]:
      top, bot = " " * 12, f"    {label}:".ljust(12)
      for i, v in enumerate(vals):
        pv = vals[i-1] if i > 0 else False
        if v and not pv: top += "┌"; bot += "┘"
        elif not v and pv: top += "┐"; bot += "└"
        elif v: top += "─"; bot += " "
        else: top += " "; bot += "─"
      lines.extend([top, bot])
    # count all edges
    m_rises = [i for i, v in enumerate(m_vals) if v and (i == 0 or not m_vals[i-1])]
    m_falls = [i for i, v in enumerate(m_vals) if not v and i > 0 and m_vals[i-1]]
    p_rises = [i for i, v in enumerate(p_vals) if v and (i == 0 or not p_vals[i-1])]
    p_falls = [i for i, v in enumerate(p_vals) if not v and i > 0 and p_vals[i-1]]
    # annotations
    ann = []
    if m_rises and p_rises:
      d = p_rises[0] - m_rises[0]
      if d: ann.append(f"{'+'if d>0 else ''}{d} frames")
    if len(m_rises) != len(p_rises) or len(m_falls) != len(p_falls):
      m_edges, p_edges = len(m_rises) + len(m_falls), len(p_rises) + len(p_falls)
      if m_edges > p_edges: ann.append(f"master: {m_edges - p_edges} extra edge(s)")
      elif p_edges > m_edges: ann.append(f"PR: {p_edges - m_edges} extra edge(s)")
    if ann:
      pos = min(m_rises[0] if m_rises else 0, p_rises[0] if p_rises else 0)
      lines.append(" " * 12 + " " * pos + "↑")
      lines.append(" " * 12 + " " * pos + ", ".join(ann))
  return lines

def run_git(cmd, cwd):
  r = subprocess.run(["git"] + cmd, cwd=cwd, capture_output=True, text=True)
  if r.returncode != 0: raise RuntimeError(f"git {' '.join(cmd)}: {r.stderr}")
  return r.stdout.strip()

def get_changed_platforms(cwd: Path, database: dict) -> list[str]:
  changed = run_git(["diff", "--name-only", "origin/master...HEAD"], cwd=cwd)
  brands = set()
  for line in changed.splitlines():
    if m := re.search(r"opendbc/car/(\w+)/", line): brands.add(m.group(1))
    if m := re.search(r"opendbc/dbc/(\w+?)_", line): brands.add(m.group(1).lower())
  return [p for p in database if any(b.upper() in p for b in brands)]

def get_database():
  import requests
  return requests.get("https://huggingface.co/datasets/commaai/commaCarSegments/raw/main/database.json").json()

def run_replay(platforms, segments, ref_path, update, cwd, workers=8):
  script = f'''
import sys; sys.path.insert(0, "."); import json, os; os.chdir("{cwd}")
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed

FIELDS = {CARSTATE_FIELDS!r}
platforms, segments, ref_path, update = {platforms!r}, {json.dumps(segments)}, {ref_path!r}, {update}

def get_attr(obj, path):
  for p in path.split("."): obj = getattr(obj, p, None)
  return obj

def differs(v1, v2):
  if v1 is None and v2 is None: return False
  if v1 is None or v2 is None: return True
  if isinstance(v1, float) and isinstance(v2, float): return abs(v1-v2) > 1e-3
  return v1 != v2

def process_segment(args):
  plat, seg = args
  try:
    from opendbc.car import gen_empty_fingerprint, structs
    from opendbc.car.can_definitions import CanData
    from opendbc.car.car_helpers import FRAME_FINGERPRINT, interfaces
    from openpilot.tools.lib.logreader import _LogFileReader
    from openpilot.selfdrive.pandad import can_capnp_to_list
    from openpilot.tools.lib.comma_car_segments import get_url
    import requests, tempfile

    seg = seg.rstrip("/s")
    parts = seg.split("/")
    url = get_url(f"{{parts[0]}}/{{parts[1]}}", parts[2])
    resp = requests.get(url)
    resp.raise_for_status()
    tmp = tempfile.NamedTemporaryFile(suffix=".zst", delete=False)
    tmp.write(resp.content)
    tmp.close()
    path = tmp.name

    can_msgs = []
    for m in _LogFileReader(path):
      if m.which() == "can":
        c = can_capnp_to_list((m.as_builder().to_bytes(),))[0]
        can_msgs.append((c[0], [CanData(*x) for x in c[1]]))

    fp = gen_empty_fingerprint()
    for _, frames in can_msgs[:FRAME_FINGERPRINT]:
      for m in frames:
        if m.src < 64: fp[m.src][m.address] = len(m.dat)
    CI = interfaces[plat]
    ci = CI(CI.get_params(plat, fp, [], False, False, False))
    CC = structs.CarControl().as_reader()
    states = [ci.update([(ts, fr)]) or ci.apply(CC, ts) or ci.update([(ts, fr)]) for ts, fr in can_msgs]

    ref_file = Path(ref_path) / f"{{plat}}_{{seg.replace('/', '_')}}.json"
    if update:
      ref_file.parent.mkdir(parents=True, exist_ok=True)
      json.dump([{{f: (lambda v: v.raw if hasattr(v, "raw") else v)(get_attr(s, f)) for f in FIELDS}} for s in states], open(ref_file, "w"))
      return (plat, seg, [], None)
    else:
      if not ref_file.exists(): return (plat, seg, [], "no ref")
      ref = json.load(open(ref_file))
      diffs = [(f, i, ref[i].get(f), (lambda v: v.raw if hasattr(v, "raw") else v)(get_attr(s, f)))
               for i, s in enumerate(states) for f in FIELDS
               if differs((lambda v: v.raw if hasattr(v, "raw") else v)(get_attr(s, f)), ref[i].get(f))]
      return (plat, seg, diffs, None)
  except Exception as e:
    return (plat, seg, [], str(e))

work = [(p, s) for p in platforms for s in segments.get(p, [])]
results = []
with ProcessPoolExecutor(max_workers={workers}) as ex:
  for f in as_completed([ex.submit(process_segment, w) for w in work]):
    results.append(f.result())
    print(f"Done: {{f.result()[0]}} {{f.result()[1]}}")
print("RESULTS:", repr(results))
'''
  r = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, cwd=cwd)
  print(r.stdout); r.stderr and print(r.stderr, file=sys.stderr)
  if "RESULTS:" in r.stdout:
    data = eval(r.stdout.split("RESULTS:")[1].strip())
    return [(p, s, [Diff(d[0], d[1], d[2], d[3]) for d in diffs], e) for p, s, diffs, e in data]
  return []

def test_replay(platform: str | None = None, segments_per_platform: int = 3) -> int:
  cwd = Path(__file__).parent.parent.resolve()
  ref_path = tempfile.mkdtemp(prefix="car_ref_")

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
    run_replay(platforms, segments, ref_path, True, cwd)

    print("\nTesting HEAD...")
    run_git(["checkout", head], cwd=cwd)
    results = run_replay(platforms, segments, ref_path, False, cwd)

    with_diffs = [(p, s, d) for p, s, d, e in results if d]
    errors = [(p, s, e) for p, s, d, e in results if e]

    print(f"\n{'='*60}")
    print(f"Results: {len(results)-len(with_diffs)-len(errors)} passed, {len(with_diffs)} with diffs, {len(errors)} errors")

    if with_diffs:
      print("\nDifferences:")
      for plat, seg, diffs in with_diffs:
        print(f"\n{plat} - {seg}")
        by_field = defaultdict(list)
        for d in diffs: by_field[d.field].append(d)
        for field, fd in sorted(by_field.items()):
          print(f"  {field}: {len(fd)} diffs")
          for line in format_waveform(fd): print(line)

    return 1 if with_diffs else 0
  finally:
    run_git(["checkout", head], cwd=cwd)

if __name__ == "__main__":
  p = argparse.ArgumentParser()
  p.add_argument("--platform")
  p.add_argument("--segments-per-platform", type=int, default=10)
  a = p.parse_args()
  sys.exit(test_replay(a.platform, a.segments_per_platform))
