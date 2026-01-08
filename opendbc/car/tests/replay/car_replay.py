#!/usr/bin/env python3
import argparse
import json
import re
import requests
import shutil
import subprocess
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

BASE_URL = "https://commadataci.blob.core.windows.net/openpilotci/"
REF_COMMIT_FN = Path(__file__).parent / "ref_commit"


def get_changed_platforms(cwd, database):
  from openpilot.common.utils import run_cmd
  changed = run_cmd(["git", "diff", "--name-only", "origin/master...HEAD"], cwd=cwd)
  brands = set()
  for line in changed.splitlines():
    if m := re.search(r"opendbc/car/(\w+)/", line):
      brands.add(m.group(1))
    if m := re.search(r"opendbc/dbc/(\w+?)_", line):
      brands.add(m.group(1).lower())
  return [p for p in database if any(b.upper() in p for b in brands)]


def download_refs(ref_path, platforms, segments, commit):
  for platform in platforms:
    for seg in segments.get(platform, []):
      filename = f"{platform}_{seg.replace('/', '_')}.zst"
      resp = requests.get(f"{BASE_URL}car_replay/{commit}/{filename}")
      if resp.status_code == 200:
        (Path(ref_path) / filename).write_bytes(resp.content)


def upload_refs(ref_path, platforms, segments, commit):
  from openpilot.tools.lib.openpilotci import upload_file
  for platform in platforms:
    for seg in segments.get(platform, []):
      filename = f"{platform}_{seg.replace('/', '_')}.zst"
      local_path = Path(ref_path) / filename
      if local_path.exists():
        upload_file(str(local_path), f"car_replay/{commit}/{filename}")


def run_worker(platforms, segments, ref_path, update, cwd, worker_path, workers=8):
  cmd = [sys.executable, worker_path,
         "--platforms", json.dumps(platforms),
         "--segments", json.dumps(segments),
         "--ref-path", ref_path,
         "--workers", str(workers)]
  if update:
    cmd.append("--update")

  result = subprocess.run(cmd, capture_output=True, text=True, cwd=cwd)
  if "RESULTS:" in result.stdout:
    return json.loads(result.stdout.split("RESULTS:")[1].strip())
  return []


def format_diff(diffs):
  if not diffs:
    return []
  lines = [f"    frame {d[1]}: {d[2]} -> {d[3]}" for d in diffs[:10]]
  if len(diffs) > 10:
    lines.append(f"    ... and {len(diffs) - 10} more")
  return lines


def main(platform=None, segments_per_platform=10, update_refs=False):
  from openpilot.common.git import get_commit
  from openpilot.tools.lib.comma_car_segments import get_comma_car_segments_database

  cwd = Path(__file__).resolve().parents[4]
  ref_path = tempfile.mkdtemp(prefix="car_ref_")

  worker_src = Path(__file__).parent / "worker.py"
  worker_tmp = Path(ref_path) / "worker.py"
  shutil.copy(worker_src, worker_tmp)

  database = get_comma_car_segments_database()
  platforms = [platform] if platform else get_changed_platforms(cwd, database)

  if not platforms:
    print("No platforms detected from changes")
    return 0

  segments = {p: database.get(p, [])[:segments_per_platform] for p in platforms}
  n_segments = sum(len(s) for s in segments.values())
  print(f"{'Generating' if update_refs else 'Testing'} {n_segments} segments for: {', '.join(platforms)}")

  commit = get_commit(cwd=str(cwd))[:12]
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

  for plat, seg, err in errors:
    print(f"\nERROR {plat} - {seg}: {err}")

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
