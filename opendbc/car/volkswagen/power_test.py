import json
import math
import os
import time

import numpy as np

# Local only. Maps the MEB EPS response (delay, lag, gain, rate limit and frequency response) at HCA_03 steering powers past
# stock, and which powers it accepts. Arm by creating ARM_FILE on the device. While engaged and hands off, whichever
# unfinished speed band matches the current speed runs its next maneuver. Every maneuver is short (3 s at most) and starts
# over if interrupted, so a band can be done over several short passes in a small lot. Leaving the band or touching the
# wheel pauses it. An EPS fault at a power skips that band's remaining steps at or above it. Progress survives restarts.
# While rolling, the car steers back to the line it started the pass on before each maneuver, so it doesn't wander off.
# ARM_FILE may hold "max_speed=3" to skip faster bands in a small lot. When every band is done ARM_FILE is renamed.
# Status for the onroad UI goes to STATUS_FILE.
ARM_FILE = "/data/meb_power_test"
LOG_DIR = "/data/meb_power_test_logs"
STATE_FILE = "state_v2.json"
STATUS_FILE = "/dev/shm/meb_power_test.json"

STOCK_POWER = 50
LEVELS = (50, 65, 80, 100)
JUMPS = (15, 30, 50)
RAMP = 1  # percent per steering message, slower than stock
# (center speed, tolerance) in m/s, the first band is a standstill
BANDS = ((0., 0.3), (1.5, 0.7), (3., 0.8), (5., 1.))
DT = 0.02  # steering message period

CURVATURE_BIG = 0.1  # m^-1, about 260 deg of wheel
CURVATURE_SMALL = 0.02  # m^-1, about 50 deg of wheel
LAT_ACCEL_BIG = 2.5  # m/s^2 while rolling
LAT_ACCEL_SMALL = 0.5
SWEEP_JERK = 3.  # m/s^3 at the top sweep frequency, under the jerk limit
SWEEP_F = (0.5, 3.)  # Hz, below that the doublets cover it
SWEEP_LEVELS = (50, 100)

# stock power EPS, fitted from normal driving (tools/lateral_maneuvers/meb_eps_fit.py), the screen draws it against the live response
STOCK_EPS_V = (3.5, 6.5, 10., 16., 25., 37.5)  # m/s
STOCK_EPS_DELAY = (0.04, 0.04, 0., 0., 0., 0.)  # s
STOCK_EPS_TAU = (0.172, 0.159, 0.401, 0.635, 0.588, 0.545)  # s
STOCK_EPS_GAIN = (0.94, 0.95, 0.92, 0.91, 0.9, 0.92)

# steer back to the start line of the pass between maneuvers
RECENTER_LOOKAHEAD = 2.  # s
RECENTER_LAT_ACCEL = 1.  # m/s^2
RECENTER_DONE = (0.5, 0.03)  # m, rad
RECENTER_MAX = 100  # steering messages

MAX_DRIVER_TORQUE = 150  # 1.5 Nm, the fast swings themselves show up as some torque
QUIET_BEFORE_START = 25  # steering messages engaged, in band, hands off and without a fault before (re)starting
STATUS_EVERY = 5  # steering messages, 10 Hz

LOG_COLUMNS = ("t", "band", "step", "phase", "maneuver", "frame", "target_power", "power", "curvature_req", "curvature_sent", "power_sent",
               "curvature_meas", "steering_angle", "steering_rate", "yaw_rate", "hca_status", "driver_torque", "v_ego")
FAULT_STATUSES = ("FAULT", "REJECTED", "PREEMPTED", "DISABLED")

# maneuver: (name, length in steering messages at standstill, rolling)
MANEUVERS = {
  "hold": (50, 50),
  "small": (80, 80),  # doublet: +, -, 0 for 30, 30, 20 messages
  "big": (145, 125),  # doublet: +, -, 0 for 60/50, 60/50, 25 messages
  "sweep": (150, 150),  # log chirp over SWEEP_F
}
DESCRIPTION = {
  "hold": "hold center",
  "small": "small step",
  "big": "big step",
  "sweep": "sweep",
}


def plan() -> list[tuple[str, int, int | None, str]]:
  # each step: (phase, target power, ramp per message or None for an instant jump, maneuver)
  steps: list[tuple[str, int, int | None, str]] = []
  for level in LEVELS:
    for maneuver in ("small", "big") + (("sweep",) if level in SWEEP_LEVELS else ()):
      steps.append(("level", level, RAMP, maneuver))
  for jump in JUMPS:
    steps.append(("settle", STOCK_POWER, RAMP, "hold"))
    steps.append(("jump", STOCK_POWER + jump, None, "big"))
  steps.append(("settle", STOCK_POWER, RAMP, "hold"))
  return steps


def maneuver_length(maneuver: str, band: int) -> int:
  return MANEUVERS[maneuver][0 if band == 0 else 1]


def maneuver_curvature(maneuver: str, k: int, band: int) -> float:
  v = BANDS[band][0]
  standstill = band == 0
  if maneuver == "small":
    amp = CURVATURE_SMALL if standstill else min(CURVATURE_SMALL, LAT_ACCEL_SMALL / v ** 2)
    return amp if k < 30 else -amp if k < 60 else 0.
  if maneuver == "big":
    amp = CURVATURE_BIG if standstill else min(CURVATURE_BIG, LAT_ACCEL_BIG / v ** 2)
    half = 60 if standstill else 50
    return amp if k < half else -amp if k < 2 * half else 0.
  if maneuver == "sweep":
    amp = CURVATURE_SMALL if standstill else min(CURVATURE_SMALL, SWEEP_JERK / (2 * math.pi * SWEEP_F[1] * v ** 2))
    n = maneuver_length("sweep", band)
    t, length = k * DT, n * DT
    # log chirp, tapered over the first and last 0.25 s
    rate = math.log(SWEEP_F[1] / SWEEP_F[0]) / length
    phase = 2 * math.pi * SWEEP_F[0] * (math.exp(rate * t) - 1) / rate
    taper = min(1., t / 0.25, (length - t) / 0.25)
    return amp * taper * math.sin(phase)
  return 0.


class MebPowerTest:
  def __init__(self):
    self.done = [False] * len(BANDS)
    self.steps = plan()
    self.step_idx = [0] * len(BANDS)
    self.step_frame = 0
    # lowest power that faulted the EPS per band, for levels and for jumps
    self.level_cap: list[int | None] = [None] * len(BANDS)
    self.jump_cap: list[int | None] = [None] * len(BANDS)
    self.faults: list[str] = []
    self._load()
    self.band: int | None = None
    self.quiet = 0
    self.power = STOCK_POWER
    self.curvature = 0.
    self.log = None
    self.finished = False
    self.frame = 0
    self.note = ""
    self.trace: list[tuple[float, float]] = []
    self.max_speed = float("inf")
    self.pose = [0., 0., 0.]  # x, y, heading from the start of the pass
    self.recenter = 0  # steering messages left to steer back to the line, 0 when on it

  def _write(self, line: str):
    if self.log is None:
      os.makedirs(LOG_DIR, exist_ok=True)
      path = os.path.join(LOG_DIR, time.strftime("%Y%m%d-%H%M%S") + ".csv")
      new = not os.path.exists(path)
      self.log = open(path, "a")
      if new:
        self.log.write(",".join(LOG_COLUMNS) + "\n")
    self.log.write(line + "\n")
    self.log.flush()

  def _load(self):
    try:
      with open(os.path.join(LOG_DIR, STATE_FILE)) as f:
        state = json.load(f)
      if len(state["done"]) == len(BANDS) and state["steps"] == len(self.steps):
        self.done, self.step_idx = state["done"], state["step_idx"]
        self.level_cap, self.jump_cap = state["level_cap"], state["jump_cap"]
        self.faults = state.get("faults", [])
    except (OSError, ValueError, KeyError):
      pass

  def _save(self):
    try:
      os.makedirs(LOG_DIR, exist_ok=True)
      with open(os.path.join(LOG_DIR, STATE_FILE), "w") as f:
        json.dump({"done": self.done, "step_idx": self.step_idx, "level_cap": self.level_cap, "jump_cap": self.jump_cap,
                   "faults": self.faults, "steps": len(self.steps)}, f)
    except OSError:
      pass

  def _skipped(self, b: int, phase: str, target: int) -> bool:
    caps = [self.level_cap[b]] + ([self.jump_cap[b]] if phase == "jump" else [])
    return phase != "settle" and any(cap is not None and target >= cap for cap in caps)

  def _next_step(self, b: int):
    # past the steps a fault ruled out, done after the last one
    self.step_frame = 0
    self.step_idx[b] += 1
    while self.step_idx[b] < len(self.steps) and self._skipped(b, *self.steps[self.step_idx[b]][:2]):
      self.step_idx[b] += 1
    if self.step_idx[b] >= len(self.steps):
      self.done[b] = True
    self._save()

  @staticmethod
  def _result(line: str):
    try:
      os.makedirs(LOG_DIR, exist_ok=True)
      with open(os.path.join(LOG_DIR, "result.txt"), "a") as f:
        f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {line}\n")
    except OSError:
      pass

  def _wanted(self, i: int) -> bool:
    return not self.done[i] and BANDS[i][0] <= self.max_speed

  def _matching_band(self, v_ego: float) -> int | None:
    for i, (center, tol) in enumerate(BANDS):
      if self._wanted(i) and abs(v_ego - center) <= tol:
        return i
    return None

  def _next_band(self) -> int | None:
    return next((i for i in range(len(BANDS)) if self._wanted(i)), None)

  def _read_arm_file(self):
    try:
      with open(ARM_FILE) as f:
        for line in f.read().split():
          if line.startswith("max_speed="):
            self.max_speed = float(line.split("=")[1])
    except (OSError, ValueError):
      pass

  def _recenter_curvature(self, v_ego: float) -> float:
    # pure pursuit back onto the start line
    x, y, heading = self.pose
    lookahead = max(RECENTER_LOOKAHEAD * v_ego, 3.)
    limit = RECENTER_LAT_ACCEL / max(v_ego, 1.) ** 2
    return max(min(-2. * (y + lookahead * math.sin(heading)) / lookahead ** 2, limit), -limit)

  def _pause(self, reason: str):
    if self.band is not None:
      phase, _, _, maneuver = self.steps[self.step_idx[self.band]]
      self._result(f"band {BANDS[self.band][0]:.1f} m/s paused in {phase} {maneuver} at power {self.power}: {reason}")
    self.note = reason
    self.band = None
    self.quiet = 0
    self.step_frame = 0

  def _end_band(self, line: str):
    self._result(line)
    self.band = None
    self.quiet = 0
    if all(self.done):
      self.finished = True
      self._result("all bands done")
      if self.log is not None:
        self.log.close()
        self.log = None
      try:
        os.rename(ARM_FILE, ARM_FILE + ".done")
      except OSError:
        pass

  def _status(self, lat_active: bool, v_ego: float, driver_torque: float, hca_status: str | None, curvature_meas: float,
              steering_angle: float, power_last: int, curvature_last: float, force: bool = False):
    # what was sent, after the safety limits, against what the EPS measured
    self.trace.append((curvature_last, curvature_meas))
    if not force and self.frame % STATUS_EVERY != 0:
      return
    b = self.band if self.band is not None else self._next_band()
    step = self.steps[self.step_idx[b]] if b is not None and self.step_idx[b] < len(self.steps) else None
    total = len(BANDS) * len(self.steps)
    progress = sum(len(self.steps) if self.done[i] else self.step_idx[i] for i in range(len(BANDS))) / total
    length = maneuver_length(step[3], b) if step is not None else 0
    status = {
      "t": time.monotonic(),
      "armed": not self.finished,
      "active": self.band is not None,
      "band": b,
      "band_speed": BANDS[b][0] if b is not None else None,
      "band_tol": BANDS[b][1] if b is not None else None,
      "bands_done": self.done,
      "step": self.step_idx[b] if b is not None else None,
      "steps": len(self.steps),
      "phase": step[0] if step is not None else None,
      "maneuver": DESCRIPTION[step[3]] if step is not None else None,
      "recenter": self.recenter > 0,
      "max_speed": self.max_speed if math.isfinite(self.max_speed) else None,
      "maneuver_progress": min(self.step_frame / max(length, 1), 1.) if self.band is not None else 0.,
      "maneuver_s": length * DT,
      "maneuver_m": length * DT * (BANDS[b][0] if b is not None else 0.),
      "target_power": step[1] if step is not None else None,
      "power": power_last,
      "curvature_cmd": self.curvature,
      "curvature_meas": curvature_meas,
      "steering_angle": steering_angle,
      "driver_torque": driver_torque,
      "hca_status": hca_status,
      "lat_active": lat_active,
      "v_ego": v_ego,
      "quiet": self.quiet / QUIET_BEFORE_START,
      "note": self.note,
      "level_cap": self.level_cap,
      "jump_cap": self.jump_cap,
      "faults": self.faults[-3:],
      "progress": progress,
      "stock_eps": {"delay": float(np.interp(v_ego, STOCK_EPS_V, STOCK_EPS_DELAY)), "tau": float(np.interp(v_ego, STOCK_EPS_V, STOCK_EPS_TAU)),
                    "gain": float(np.interp(v_ego, STOCK_EPS_V, STOCK_EPS_GAIN))},
      "trace": self.trace,
    }
    self.trace = []
    try:
      with open(STATUS_FILE + ".tmp", "w") as f:
        json.dump(status, f)
      os.replace(STATUS_FILE + ".tmp", STATUS_FILE)
    except OSError:
      pass

  def update(self, lat_active: bool, v_ego: float, driver_torque: float, hca_status: str | None, steering_pressed: bool,
             curvature_meas: float, steering_angle: float, steering_rate: float, yaw_rate: float,
             curvature_last: float, power_last: int) -> tuple[float, int] | None:
    """Call once per steering message. Returns (curvature, power) while the test drives the EPS, else None."""
    if self.finished or not os.path.exists(ARM_FILE):
      return None
    if self.frame % 50 == 0:
      self._read_arm_file()
    self.frame += 1
    out = self._update(lat_active, v_ego, driver_torque, hca_status, curvature_meas, steering_angle, steering_rate, yaw_rate,
                       curvature_last, power_last)
    if out is None:
      self.curvature = 0.
    self._status(lat_active, v_ego, driver_torque, hca_status, curvature_meas, steering_angle, power_last, curvature_last)
    return out

  def _update(self, lat_active: bool, v_ego: float, driver_torque: float, hca_status: str | None, curvature_meas: float,
              steering_angle: float, steering_rate: float, yaw_rate: float, curvature_last: float, power_last: int) -> tuple[float, int] | None:
    fault = hca_status in FAULT_STATUSES
    hands_off = abs(driver_torque) < MAX_DRIVER_TORQUE

    if self.band is None:
      band = self._matching_band(v_ego)
      ok = lat_active and hands_off and not fault and band is not None
      self.quiet = self.quiet + 1 if ok else 0
      if not lat_active:
        self.note = "engage to start"
      elif fault:
        self.note = f"EPS {hca_status}"
      elif not hands_off:
        self.note = "hands off the wheel"
      elif band is None:
        nb = self._next_band()
        self.note = f"drive {BANDS[nb][0]:.1f} m/s" if nb is not None and BANDS[nb][0] > 0 else "stop the car"
      else:
        self.note = "starting"
      if self.quiet < QUIET_BEFORE_START:
        return None
      self.band = band
      self.power = power_last
      self.pose = [0., 0., 0.]
      self.recenter = 0
      self.step_frame = 0
      self.note = ""
      self._result(f"band {BANDS[band][0]:.1f} m/s started at step {self.step_idx[band]}")

    b = self.band
    phase, target, ramp, maneuver = self.steps[self.step_idx[b]]
    if fault:
      cap = max(self.power, power_last)
      if phase == "jump":
        self.jump_cap[b] = min(self.jump_cap[b] or 1000, cap)
      else:
        self.level_cap[b] = min(self.level_cap[b] or 1000, cap)
      what = f"EPS {hca_status} in {phase} {maneuver} at power {cap} (target {target})"
      self.faults.append(f"{BANDS[b][0]:.1f} m/s: {hca_status} at {cap}%{' jump' if phase == 'jump' else ''}")
      self._next_step(b)
      self._end_band(f"band {BANDS[b][0]:.1f} m/s {what}, skipping its steps from there, next step {self.step_idx[b]}")
      self.note = what
      return None
    center, tol = BANDS[b]
    if not lat_active or not hands_off or abs(v_ego - center) > tol:
      reason = "disengaged" if not lat_active else "hands on the wheel" if not hands_off else f"left the band at {v_ego:.1f} m/s"
      self._pause(reason)
      return None

    x, y, heading = self.pose
    self.pose = [x + v_ego * math.cos(heading) * DT, y + v_ego * math.sin(heading) * DT, heading + yaw_rate * DT]

    # reach the target power and the start line first, then run the maneuver
    if ramp is None:
      self.power = target
    elif self.power != target:
      self.power += max(min(target - self.power, ramp), -ramp)
    if self.recenter > 0:
      self.recenter -= 1
      if abs(self.pose[1]) < RECENTER_DONE[0] and abs(self.pose[2]) < RECENTER_DONE[1]:
        self.recenter = 0
    self.curvature = self._recenter_curvature(v_ego) if b != 0 else 0.
    if self.power == target and self.recenter == 0:
      self.curvature = maneuver_curvature(maneuver, self.step_frame, b)
      self.step_frame += 1

    label = "recenter" if self.recenter > 0 else maneuver if self.power == target else "ramp"
    row = (f"{time.monotonic():.3f}", f"{center:.1f}", self.step_idx[b], phase, label, self.step_frame, target, self.power,
           f"{self.curvature:.5f}", f"{curvature_last:.5f}", power_last, f"{curvature_meas:.5f}", f"{steering_angle:.1f}",
           f"{steering_rate:.1f}", f"{yaw_rate:.4f}", hca_status, driver_torque, f"{v_ego:.2f}")
    self._write(",".join(str(x) for x in row))

    if self.step_frame >= maneuver_length(maneuver, b):
      self._next_step(b)
      self.recenter = RECENTER_MAX if b != 0 else 0
      if self.done[b]:
        self._end_band(f"band {center:.1f} m/s completed")
        return None
    return self.curvature, int(self.power)
