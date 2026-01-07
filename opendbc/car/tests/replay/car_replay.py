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

BASE_URL = "https://commadataci.blob.core.windows.net/openpilotci/"
REF_COMMIT_FN = Path(__file__).parent / "ref_commit"


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


def download_refs(ref_path, platforms, segments, commit):
  import requests
  for p in platforms:
    for seg in segments.get(p, []):
      fn = f"{p}_{seg.replace('/', '_')}.zst"
      r = requests.get(f"{BASE_URL}car_replay/{commit}/{fn}")
      if r.status_code == 200:
        (Path(ref_path) / fn).write_bytes(r.content)


def upload_refs(ref_path, platforms, segments, commit):
  from openpilot.tools.lib.openpilotci import upload_file
  for p in platforms:
    for seg in segments.get(p, []):
      fn = f"{p}_{seg.replace('/', '_')}.zst"
      local = Path(ref_path) / fn
      if local.exists():
        upload_file(str(local), f"car_replay/{commit}/{fn}")


def run_worker(platforms, segments, ref_path, update, cwd, worker_path, workers=8):
  cmd = [sys.executable, worker_path,
         "--platforms", json.dumps(platforms),
         "--segments", json.dumps(segments),
         "--ref-path", ref_path,
         "--workers", str(workers)]
  if update:
    cmd.append("--update")

  r = subprocess.run(cmd, capture_output=True, text=True, cwd=cwd)
  if "RESULTS:" in r.stdout:
    return json.loads(r.stdout.split("RESULTS:")[1].strip())
  return []


def format_diff(diffs):
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
    first, last = rdiffs[0], rdiffs[-1]
    if first[2] and not first[3]:
      b_st, m_st = False, False
    elif not first[2] and first[3]:
      b_st, m_st = True, True
    else:
      b_st, m_st = False, False

    converge_frame = last[1] + 1
    converge_val = last[2]

    for f in range(t0, t1):
      if f in diff_map:
        b_st, m_st = diff_map[f][2], diff_map[f][3]
        if len(diff_map[f]) > 4:
          ts_map[f] = diff_map[f][4]
      elif f >= converge_frame:
        b_st = m_st = converge_val
      b_vals.append(b_st)
      m_vals.append(m_st)

    ts_start = ts_map.get(t0, rdiffs[0][4] if len(rdiffs[0]) > 4 else 0)
    ts_end = ts_map.get(t1 - 1, rdiffs[-1][4] if len(rdiffs[-1]) > 4 else 0)
    t0_sec = ts_start / 1e9
    t1_sec = ts_end / 1e9

    # ms per frame from timestamps
    if len(ts_map) >= 2:
      ts_vals = sorted(ts_map.items())
      frame_ms = (ts_vals[-1][1] - ts_vals[0][1]) / 1e6 / (ts_vals[-1][0] - ts_vals[0][0])
    else:
      frame_ms = 10

    lines.append(f"\n  frames {t0}-{t1-1} (t={t0_sec:.2f}s - {t1_sec:.2f}s)")
    pad = 12
    init_b = not (first[2] and not first[3])
    init_m = not first[2] and first[3]
    for label, vals, init in [("master", b_vals, init_b), ("PR", m_vals, init_m)]:
      line = f"  {label}:".ljust(pad)
      for i, v in enumerate(vals):
        pv = vals[i - 1] if i > 0 else init
        if v and not pv:
          line += "/"
        elif not v and pv:
          line += "\\"
        elif v:
          line += "‾"
        else:
          line += "_"
      lines.append(line)

    b_rises = [i for i, v in enumerate(b_vals) if v and (i == 0 or not b_vals[i - 1])]
    m_rises = [i for i, v in enumerate(m_vals) if v and (i == 0 or not m_vals[i - 1])]
    b_falls = [i for i, v in enumerate(b_vals) if not v and i > 0 and b_vals[i - 1]]
    m_falls = [i for i, v in enumerate(m_vals) if not v and i > 0 and m_vals[i - 1]]

    if b_rises and m_rises:
      delta = m_rises[0] - b_rises[0]
      if delta:
        ms = int(abs(delta) * frame_ms)
        direction = "lags" if delta > 0 else "leads"
        lines.append(" " * pad + f"rise: PR {direction} by {abs(delta)} frames ({ms}ms)")
    if b_falls and m_falls:
      delta = m_falls[0] - b_falls[0]
      if delta:
        ms = int(abs(delta) * frame_ms)
        direction = "lags" if delta > 0 else "leads"
        lines.append(" " * pad + f"fall: PR {direction} by {abs(delta)} frames ({ms}ms)")

  return lines


def main(platform=None, segments_per_platform=10, update_refs=False):
  cwd = Path(__file__).resolve().parents[4]
  ref_path = tempfile.mkdtemp(prefix="car_ref_")

  worker_src = Path(__file__).parent / "worker.py"
  worker_tmp = Path(ref_path) / "worker.py"
  shutil.copy(worker_src, worker_tmp)

  database = get_database()
  platforms = [platform] if platform else (list(database.keys()) if update_refs else get_changed_platforms(cwd, database)[:10])
  if not platforms:
    print("No platforms detected from changes")
    return 0

  segments = {p: database.get(p, [])[:segments_per_platform] for p in platforms}
  n_segments = sum(len(s) for s in segments.values())
  print(f"{'Generating' if update_refs else 'Comparing'} {n_segments} segments for: {', '.join(platforms)}")

  commit = run_git(["rev-parse", "--short=12", "HEAD"], cwd=cwd)
  if update_refs:
    run_worker(platforms, segments, ref_path, True, cwd, str(worker_tmp))
    upload_refs(ref_path, platforms, segments, commit)
    REF_COMMIT_FN.write_text(commit + "\n")
    print(f"Uploaded refs for {commit}")
    return 0

  ref_commit = REF_COMMIT_FN.read_text().strip() if REF_COMMIT_FN.exists() else None
  if not ref_commit:
    print("No ref_commit found")
    return 1

  print(f"Comparing against ref {ref_commit}")
  download_refs(ref_path, platforms, segments, ref_commit)
  results = run_worker(platforms, segments, ref_path, False, cwd, str(worker_tmp))

  passed = [(p, s) for p, s, d, e, n in results if not d and not e]
  with_diffs = [(p, s, d, n) for p, s, d, e, n in results if d]
  errors = [(p, s, e) for p, s, d, e, n in results if e]

  print(f"\nResults: {len(passed)} passed, {len(with_diffs)} with diffs, {len(errors)} errors")

  for plat, seg, diffs, _ in with_diffs:
    print(f"\n{plat} - {seg}")
    by_field = defaultdict(list)
    for d in diffs:
      by_field[d[0]].append(d)
    for field, fd in sorted(by_field.items()):
      print(f"  {field}: {len(fd)} diffs")
      for line in format_diff(fd):
        print(line)

  return 0


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--platform")
  parser.add_argument("--segments-per-platform", type=int, default=10)
  parser.add_argument("--update-refs", action="store_true")
  args = parser.parse_args()
  sys.exit(main(args.platform, args.segments_per_platform, args.update_refs))
