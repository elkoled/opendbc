#!/usr/bin/env python3
import sys
from openpilot.tools.lib.logreader import LogReader, ReadMode
from opendbc.car.fw_versions import match_fw_to_car

# Usage: python fingerprint.py <route>
if __name__ == "__main__":
  # Handle "route" or "route--5" inputs
  route = sys.argv[1]
  if route[-1].isdigit() and "--" in route:
    route = route.rsplit("--", 1)[0]

  print(f"Fingerprinting {route}...")

  # Load segment 0
  lr = LogReader(f"{route}--0", default_mode=ReadMode.QLOG)

  # Extract carParams and match
  try:
    cp = next(e.carParams for e in lr if e.which() == 'carParams')
    exact, matches = match_fw_to_car(list(cp.carFw), cp.carVin)
    print(f"Match: {'Exact' if exact else 'Fuzzy'}\nCandidates: {', '.join(sorted(matches))}")
  except StopIteration:
    print("Error: carParams not found in log.")