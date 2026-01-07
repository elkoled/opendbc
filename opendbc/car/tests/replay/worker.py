#!/usr/bin/env python3
"""
Worker script for car behavior replay. Runs in subprocess to ensure fresh imports.
Called by compare.py after git checkout to test a specific code version.
"""
import argparse
import json
import tempfile
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

CARSTATE_FIELDS = [
  "vEgo", "aEgo", "standstill", "steeringAngleDeg", "steeringTorque", "steeringPressed",
  "gas", "gasPressed", "brake", "brakePressed", "gearShifter", "leftBlinker", "rightBlinker",
  "cruiseState.enabled", "cruiseState.speed", "cruiseState.available", "doorOpen", "seatbeltUnlatched",
]


def get_attr(obj, path):
  """Get nested attribute like 'cruiseState.enabled'"""
  for p in path.split("."):
    obj = getattr(obj, p, None)
  return obj


def get_value(obj, field):
  """Extract value from CarState field, handling enum .raw conversion"""
  v = get_attr(obj, field)
  return v.raw if hasattr(v, "raw") else v


def differs(v1, v2):
  """Check if two values differ (with float tolerance)"""
  if v1 is None and v2 is None:
    return False
  if v1 is None or v2 is None:
    return True
  if isinstance(v1, float) and isinstance(v2, float):
    return abs(v1 - v2) > 1e-3
  return v1 != v2


def download_segment(seg: str) -> str:
  """Download segment rlog and return local path"""
  import requests
  from openpilot.tools.lib.comma_car_segments import get_url

  seg = seg.rstrip("/s")
  parts = seg.split("/")
  url = get_url(f"{parts[0]}/{parts[1]}", parts[2])

  resp = requests.get(url)
  resp.raise_for_status()

  tmp = tempfile.NamedTemporaryFile(suffix=".zst", delete=False)
  tmp.write(resp.content)
  tmp.close()
  return tmp.name


def load_can_messages(path: str) -> list:
  """Load CAN messages from rlog file"""
  from opendbc.car.can_definitions import CanData
  from openpilot.tools.lib.logreader import _LogFileReader
  from openpilot.selfdrive.pandad import can_capnp_to_list

  can_msgs = []
  for m in _LogFileReader(path):
    if m.which() == "can":
      c = can_capnp_to_list((m.as_builder().to_bytes(),))[0]
      can_msgs.append((c[0], [CanData(*x) for x in c[1]]))
  return can_msgs


def run_car_interface(platform: str, can_msgs: list) -> list:
  """Run CAN messages through car interface, return CarState list"""
  from opendbc.car import gen_empty_fingerprint, structs
  from opendbc.car.car_helpers import FRAME_FINGERPRINT, interfaces

  # Build fingerprint from first N frames
  fp = gen_empty_fingerprint()
  for _, frames in can_msgs[:FRAME_FINGERPRINT]:
    for m in frames:
      if m.src < 64:
        fp[m.src][m.address] = len(m.dat)

  # Initialize car interface
  CI = interfaces[platform]
  ci = CI(CI.get_params(platform, fp, [], False, False, False))
  CC = structs.CarControl().as_reader()

  # Process all CAN messages
  states = []
  for ts, frames in can_msgs:
    ci.update([(ts, frames)])
    ci.apply(CC, ts)
    states.append(ci.update([(ts, frames)]))
  return states


def process_segment(args: tuple) -> tuple:
  """Process a single segment - download, replay, compare/save"""
  platform, seg, ref_path, update = args
  try:
    path = download_segment(seg)
    can_msgs = load_can_messages(path)
    states = run_car_interface(platform, can_msgs)

    ref_file = Path(ref_path) / f"{platform}_{seg.replace('/', '_')}.json"

    if update:
      # Save reference data
      ref_file.parent.mkdir(parents=True, exist_ok=True)
      data = [{f: get_value(s, f) for f in CARSTATE_FIELDS} for s in states]
      json.dump(data, open(ref_file, "w"))
      return (platform, seg, [], None)
    else:
      # Compare against reference
      if not ref_file.exists():
        return (platform, seg, [], "no ref")
      ref = json.load(open(ref_file))
      diffs = []
      for i, state in enumerate(states):
        for field in CARSTATE_FIELDS:
          new_val = get_value(state, field)
          old_val = ref[i].get(field)
          if differs(new_val, old_val):
            diffs.append((field, i, old_val, new_val))
      return (platform, seg, diffs, None)
  except Exception as e:
    return (platform, seg, [], str(e))


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--platforms", required=True, help="JSON list of platforms")
  parser.add_argument("--segments", required=True, help="JSON dict of platform -> segments")
  parser.add_argument("--ref-path", required=True, help="Path to store/load reference data")
  parser.add_argument("--update", action="store_true", help="Update reference data instead of comparing")
  parser.add_argument("--workers", type=int, default=8)
  args = parser.parse_args()

  platforms = json.loads(args.platforms)
  segments = json.loads(args.segments)

  # Build work list
  work = [(p, s, args.ref_path, args.update) for p in platforms for s in segments.get(p, [])]

  # Process in parallel
  results = []
  with ProcessPoolExecutor(max_workers=args.workers) as executor:
    futures = [executor.submit(process_segment, w) for w in work]
    for future in as_completed(futures):
      result = future.result()
      results.append(result)
      print(f"Done: {result[0]} {result[1]}", flush=True)

  # Output results as JSON for parent process to parse
  print("RESULTS:" + json.dumps(results))


if __name__ == "__main__":
  main()
