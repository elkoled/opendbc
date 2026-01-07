#!/usr/bin/env python3
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
  for p in path.split("."):
    obj = getattr(obj, p, None)
  return obj


def get_value(obj, field):
  v = get_attr(obj, field)
  return v.raw if hasattr(v, "raw") else v


def differs(v1, v2):
  if isinstance(v1, float) and isinstance(v2, float):
    return abs(v1 - v2) > 1e-3
  return v1 != v2


def download_segment(seg):
  import requests
  from openpilot.tools.lib.comma_car_segments import get_url

  seg = seg.rstrip("/s")
  parts = seg.split("/")
  url = get_url(f"{parts[0]}/{parts[1]}", parts[2])

  resp = requests.get(url)
  resp.raise_for_status()

  with tempfile.NamedTemporaryFile(suffix=".zst", delete=False) as tmp:
    tmp.write(resp.content)
    return tmp.name


def load_can_messages(path):
  from opendbc.car.can_definitions import CanData
  from openpilot.selfdrive.pandad import can_capnp_to_list
  from openpilot.tools.lib.logreader import _LogFileReader

  can_msgs = []
  for msg in _LogFileReader(path):
    if msg.which() == "can":
      ts, data = can_capnp_to_list((msg.as_builder().to_bytes(),))[0]
      can_msgs.append((ts, [CanData(*x) for x in data]))
  return can_msgs


def replay_segment(platform, can_msgs):
  from opendbc.car import gen_empty_fingerprint, structs
  from opendbc.car.car_helpers import FRAME_FINGERPRINT, interfaces

  fp = gen_empty_fingerprint()
  for _, frames in can_msgs[:FRAME_FINGERPRINT]:
    for msg in frames:
      if msg.src < 64:
        fp[msg.src][msg.address] = len(msg.dat)

  CI = interfaces[platform]
  ci = CI(CI.get_params(platform, fp, [], False, False, False))
  CC = structs.CarControl().as_reader()

  states = []
  timestamps = []
  for ts, frames in can_msgs:
    ci.update([(ts, frames)])
    ci.apply(CC, ts)
    states.append(ci.update([(ts, frames)]))
    timestamps.append(ts)
  return states, timestamps


def process_segment(args):
  platform, seg, ref_path, update = args
  try:
    can_msgs = load_can_messages(download_segment(seg))
    states, timestamps = replay_segment(platform, can_msgs)
    ref_file = Path(ref_path) / f"{platform}_{seg.replace('/', '_')}.json"

    if update:
      ref_file.parent.mkdir(parents=True, exist_ok=True)
      data = [{"ts": ts, **{field: get_value(s, field) for field in CARSTATE_FIELDS}}
              for s, ts in zip(states, timestamps, strict=True)]
      with open(ref_file, "w") as f:
        json.dump(data, f)
      return (platform, seg, [], None, len(states))

    if not ref_file.exists():
      return (platform, seg, [], "no ref", 0)

    with open(ref_file) as f:
      ref = json.load(f)
    diffs = [(field, i, ref[i].get(field), get_value(state, field), timestamps[i])
             for i, state in enumerate(states) for field in CARSTATE_FIELDS
             if differs(get_value(state, field), ref[i].get(field))]
    return (platform, seg, diffs, None, len(states))
  except Exception as e:
    return (platform, seg, [], str(e), 0)


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--platforms", required=True)
  parser.add_argument("--segments", required=True)
  parser.add_argument("--ref-path", required=True)
  parser.add_argument("--update", action="store_true")
  parser.add_argument("--workers", type=int, default=8)
  args = parser.parse_args()

  platforms = json.loads(args.platforms)
  segments = json.loads(args.segments)
  work = [(p, s, args.ref_path, args.update) for p in platforms for s in segments.get(p, [])]

  results = []
  with ProcessPoolExecutor(max_workers=args.workers) as pool:
    for r in as_completed([pool.submit(process_segment, w) for w in work]):
      results.append(r.result())
      print(f"Done: {r.result()[0]} {r.result()[1]}", flush=True)

  print("RESULTS:" + json.dumps(results))
