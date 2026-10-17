import json
import math
import os
import time

import numpy as np

# Local only. Maps the MEB EPS response (delay, lag, gain) at HCA_03 steering powers past stock, and which powers it accepts.
# Arm by creating ARM_FILE on the device, then engage at a stop, on a straight road. The test drives itself: it runs the
# standstill band with steps to both end stops, then drives off and runs each speed band with small steps on top of
# openpilot's own steering, so the car keeps its lane, and stops and holds at the end until disengaged. Gas, brake or a
# hand on the wheel take over as usual. Each step starts over if interrupted. An EPS fault at a power skips that band's
# remaining steps at or above it. Progress survives restarts.
# ARM_FILE may hold "max_speed=3" to skip faster bands. When every band is done ARM_FILE is renamed.
# Status for the onroad UI goes to STATUS_FILE.
ARM_FILE = "/data/meb_power_test"
LOG_DIR = "/data/meb_power_test_logs"
STATE_FILE = "state_v3.json"
STATUS_FILE = "/dev/shm/meb_power_test.json"

STOCK_POWER = 50
LEVELS = (50, 75, 100)
JUMPS = (50,)
RAMP = 5  # percent per steering message, like stock
# (center speed, tolerance) in m/s, the first band is a standstill
BANDS = ((0., 0.3), (1.5, 0.7), (3., 0.8), (5., 1.))
DT = 0.02  # steering message period

# stock power EPS, fitted from normal driving (tools/lateral_maneuvers/meb_eps_fit.py), the screen draws it against the live response
STOCK_EPS_V = (3.5, 6.5, 10., 16., 25., 37.5)  # m/s
STOCK_EPS_DELAY = (0.04, 0.04, 0., 0., 0., 0.)  # s
STOCK_EPS_TAU = (0.172, 0.159, 0.401, 0.635, 0.588, 0.545)  # s
STOCK_EPS_GAIN = (0.94, 0.95, 0.92, 0.91, 0.9, 0.92)

RELEASE = (10, 10)  # steering messages holding the base curvature before each maneuver, at standstill and rolling

# rolling steps go on top of openpilot's steering, sized to move the car at most DRIFT sideways
DRIFT = 0.3  # m
MAX_LAT_ACCEL = 3.  # m/s^2, openpilot's steering and the step together
CURVATURE_MAX = 0.19  # m^-1, just under full lock
REPEATS = 3  # of every step, for a solid model

# the test drives the speed: to each band, then to a stop and a hold
SPEED_GAIN = 1.  # 1/s
SPEED_ACCEL = (-1., 0.8)  # m/s^2
STOP_SPEED = 0.3  # m/s
STOP_ACCEL = -0.5  # m/s^2, while holding

MAX_DRIVER_TORQUE = 150  # 1.5 Nm
MAX_DRIVER_TORQUE_STEP = 400  # during a step the wheel's own fast motion reads up to about 3 Nm
HANDS_ON = 15  # steering messages over the limit before it counts as hands on
QUIET_BEFORE_START = 25  # steering messages engaged, in band, hands off and without a fault before (re)starting
STATUS_EVERY = 5  # steering messages, 10 Hz
DONE_SHOW_S = 10.

LOG_COLUMNS = ("t", "band", "step", "phase", "maneuver", "frame", "target_power", "power", "curvature_req", "curvature_sent", "power_sent",
               "curvature_meas", "steering_angle", "steering_rate", "yaw_rate", "hca_status", "driver_torque", "v_ego")
FAULT_STATUSES = ("FAULT", "REJECTED", "PREEMPTED", "DISABLED")

# doublet: + then - for half, then zero for TAIL, edges ramped over EDGE, in steering messages
# (curvature, half at standstill, half rolling)
DOUBLETS = {
  "small": (0.02, 50, 25),  # about 50 deg of wheel, the wheel moves slowest at standstill
  "big": (0.05, 100, 30),  # about 130 deg, long enough to settle and to reach the EPS rate limit
  "turn": (0.11, 100, 100),  # about 290 deg, a tight 90 degree turn as fast as the safety limits allow, held to see how far it gets
}
TAIL = 20
EDGE = 5
# log chirp for the small signal frequency response, which more power may change
SWEEP_F = (0.5, 3.)  # Hz
SWEEP_LENGTH = 150  # steering messages
SWEEP_CURVATURE = 0.01  # m^-1, about 25 deg of wheel
SWEEP_JERK = 3.  # m/s^3 at the top frequency while rolling, under the jerk limit
SWEEP_LEVELS = (50, 100)
SWEEP_REPEATS = 2

STANDSTILL_ONLY = ("turn", "lock", "stairs")
# standstill only, the full range to both end stops: (power, maneuver)
STANDSTILL_STEPS = ((50, "lock"), (75, "lock"), (100, "lock"), (50, "stairs"), (100, "stairs"))
LOCK = CURVATURE_MAX  # the ID.4 turns about 5.1 m at full lock, so this reaches the end stops
# lock to lock: left, right, back to center, held this long in steering messages
LOCK_HOLDS = (400, 600, 300)
# staircase from right lock to left lock and back, STAIRS levels held STAIR_HOLD each
STAIRS = 13
STAIR_HOLD = 75


def _profile(levels: list[float], holds: list[int]) -> tuple[np.ndarray, np.ndarray]:
  # piecewise constant curvature with ramped edges, starting at center
  t, c, k = [0], [0.], 0
  for level, hold in zip(levels, holds, strict=True):
    t += [k + EDGE, k + hold]
    c += [level, level]
    k += hold
  return np.array(t, float), np.array(c)


_stairs = list(np.linspace(-LOCK, LOCK, STAIRS))
PROFILES = {
  "lock": _profile([LOCK, -LOCK, 0.], list(LOCK_HOLDS)),
  "stairs": _profile([-LOCK] + _stairs[1:] + _stairs[-2::-1] + [0.], [200] + [STAIR_HOLD] * (2 * STAIRS - 2) + [150]),
}
DESCRIPTION = {
  "hold": "settle",
  "small": "small step",
  "big": "big step",
  "turn": "turn",
  "sweep": "sweep",
  "lock": "lock to lock",
  "stairs": "angle steps",
}


def plan() -> list[tuple[str, int, int | None, str]]:
  # each step: (phase, target power, ramp per message or None for an instant jump, maneuver)
  steps: list[tuple[str, int, int | None, str]] = []
  for level in LEVELS:
    steps += [("level", level, RAMP, m) for m in ("small", "big", "turn") for _ in range(REPEATS)]
    steps += [("level", level, RAMP, "sweep")] * (SWEEP_REPEATS if level in SWEEP_LEVELS else 0)
  for jump in JUMPS:
    for _ in range(REPEATS):
      steps.append(("settle", STOCK_POWER, RAMP, "hold"))
      steps.append(("jump", STOCK_POWER + jump, None, "big"))
  steps += [("standstill", power, RAMP, m) for power, m in STANDSTILL_STEPS]
  steps.append(("settle", STOCK_POWER, RAMP, "hold"))
  return steps


def plan_id() -> str:
  # progress from a different plan doesn't carry over
  return repr((BANDS, LEVELS, JUMPS, RAMP, DOUBLETS, TAIL, EDGE, REPEATS, DRIFT, SWEEP_F, SWEEP_LENGTH, SWEEP_LEVELS, SWEEP_REPEATS,
               STANDSTILL_STEPS, LOCK, LOCK_HOLDS, STAIRS, STAIR_HOLD))


def maneuver_length(maneuver: str, band: int) -> int:
  if maneuver == "sweep":
    return SWEEP_LENGTH
  if maneuver in PROFILES:
    return int(PROFILES[maneuver][0][-1])
  if maneuver not in DOUBLETS:
    return 0
  return 2 * DOUBLETS[maneuver][1 if band == 0 else 2] + TAIL


def maneuver_curvature(maneuver: str, k: int, band: int, base: float = 0.) -> float:
  # base is openpilot's own steering while rolling, zero at standstill
  if maneuver == "sweep":
    v = BANDS[band][0]
    amp = SWEEP_CURVATURE if band == 0 else min(SWEEP_CURVATURE, SWEEP_JERK / (2 * math.pi * SWEEP_F[1] * v ** 2))
    t, length = k * DT, SWEEP_LENGTH * DT
    rate = math.log(SWEEP_F[1] / SWEEP_F[0]) / length
    phase = 2 * math.pi * SWEEP_F[0] * (math.exp(rate * t) - 1) / rate
    # tapered over the first and last 0.25 s
    return base + amp * min(1., t / 0.25, (length - t) / 0.25) * math.sin(phase)
  if maneuver in PROFILES:
    return base + float(np.interp(k, *PROFILES[maneuver]))
  if maneuver not in DOUBLETS:
    return base
  amp, half_standstill, half = DOUBLETS[maneuver]
  if band == 0:
    half = half_standstill
  else:
    # a doublet moves the car about curvature * (v * half time)^2 sideways
    v = BANDS[band][0]
    amp = max(min(amp, DRIFT / (v * half * DT) ** 2, MAX_LAT_ACCEL / v ** 2 - abs(base), CURVATURE_MAX - abs(base)), 0.)
  # ramped edges keep the wheel's inertia from reading as driver torque
  return base + float(np.interp(k, [0, EDGE, half, half + 2 * EDGE, 2 * half + EDGE, 2 * half + 2 * EDGE], [0., amp, amp, -amp, -amp, 0.]))


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
    self.finished_t = 0.
    self.holding = False  # the test has the pedals, kept after the end to hold the car until disengaged
    self.plan_curvature = 0.
    self.frame = 0
    self.note = ""
    self.trace: list[tuple[float, float]] = []
    self.max_speed = float("inf")
    self.release = 0  # steering messages held at the base curvature before this maneuver
    self.torque_frames = 0

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
      if len(state["done"]) == len(BANDS) and state["plan"] == plan_id():
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
                   "faults": self.faults, "plan": plan_id()}, f)
    except OSError:
      pass

  def _skipped(self, b: int, phase: str, target: int, maneuver: str) -> bool:
    if b != 0 and maneuver in STANDSTILL_ONLY:
      return True
    caps = [self.level_cap[b]] + ([self.jump_cap[b]] if phase == "jump" else [])
    return phase != "settle" and any(cap is not None and target >= cap for cap in caps)

  def _next_step(self, b: int):
    # past the steps a fault ruled out, done after the last one
    self.step_frame = 0
    self.release = 0
    self.step_idx[b] += 1
    while self.step_idx[b] < len(self.steps) and self._skipped(b, *self.steps[self.step_idx[b]][:2], self.steps[self.step_idx[b]][3]):
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

  def _target_speed(self) -> float:
    b = self.band if self.band is not None else self._next_band()
    return BANDS[b][0] if b is not None and not self.finished else 0.

  def long_command(self, v_ego: float, long_active: bool) -> tuple[float, bool] | None:
    """Called while openpilot has the pedals. Returns (accel, stopping) while the test drives, else None."""
    if not long_active:
      self.holding = False
      return None
    if not self.holding and (self.finished or not os.path.exists(ARM_FILE)):
      return None
    self.holding = True
    target = self._target_speed()
    if target == 0. and v_ego < STOP_SPEED:
      return STOP_ACCEL, True
    return min(max(SPEED_GAIN * (target - v_ego), SPEED_ACCEL[0]), SPEED_ACCEL[1]), False

  def _base(self, b: int) -> float:
    return 0. if b == 0 else self.plan_curvature

  def _read_arm_file(self):
    try:
      with open(ARM_FILE) as f:
        for line in f.read().split():
          if line.startswith("max_speed="):
            self.max_speed = float(line.split("=")[1])
    except (OSError, ValueError):
      pass

  def _pause(self, reason: str):
    if self.band is not None:
      phase, _, _, maneuver = self.steps[self.step_idx[self.band]]
      self._result(f"band {BANDS[self.band][0]:.1f} m/s paused in {phase} {maneuver} at power {self.power}: {reason}")
    self.note = reason
    self.band = None
    self.quiet = 0
    self.step_frame = 0
    self.release = 0

  def _end_band(self, line: str):
    self._result(line)
    self.band = None
    self.quiet = 0
    if all(self.done):
      self.finished = True
      self.finished_t = time.monotonic()
      self.note = "test done"
      self._result("all bands done")
      # the next arm starts a fresh run
      try:
        os.remove(os.path.join(LOG_DIR, STATE_FILE))
      except OSError:
        pass
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
             curvature_last: float, power_last: int, plan_curvature: float = 0.) -> tuple[float, int] | None:
    """Call once per steering message. Returns (curvature, power) while the test steers, else None."""
    self.plan_curvature = plan_curvature
    if self.finished and os.path.exists(ARM_FILE):
      # armed again, fresh run
      self.__init__()
    if self.finished:
      # keep "test done" on the screen for a moment
      self.frame += 1
      if time.monotonic() - self.finished_t < DONE_SHOW_S:
        self._status(lat_active, v_ego, driver_torque, hca_status, curvature_meas, steering_angle, power_last, curvature_last)
      return None
    if not os.path.exists(ARM_FILE):
      return None
    if self.frame % 50 == 0:
      self._read_arm_file()
    self.frame += 1
    out = self._update(lat_active, v_ego, driver_torque, hca_status, curvature_meas, steering_angle, steering_rate, yaw_rate,
                       curvature_last, power_last)
    if out is None or out[0] is None:
      self.curvature = 0.
    self._status(lat_active, v_ego, driver_torque, hca_status, curvature_meas, steering_angle, power_last, curvature_last)
    return out

  def _update(self, lat_active: bool, v_ego: float, driver_torque: float, hca_status: str | None, curvature_meas: float,
              steering_angle: float, steering_rate: float, yaw_rate: float, curvature_last: float,
              power_last: int) -> tuple[float, int] | None:
    fault = hca_status in FAULT_STATUSES
    limit = MAX_DRIVER_TORQUE_STEP if self.band is not None and self.step_frame > 0 else MAX_DRIVER_TORQUE
    self.torque_frames = self.torque_frames + 1 if abs(driver_torque) > limit else 0
    hands_off = self.torque_frames < HANDS_ON

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
        target = self._target_speed()
        self.note = f"driving to {target:.1f} m/s" if target > 0 else "stopping"
      else:
        self.note = "starting"
      if self.quiet < QUIET_BEFORE_START:
        return None
      self.band = band
      self.power = power_last
      self.release = 0
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

    # hold the base curvature while the power ramps to the target and for a moment after, a jump happens as the maneuver starts
    if ramp is not None and self.power != target:
      self.power += max(min(target - self.power, ramp), -ramp)
    if self.step_frame == 0 and (self.release < RELEASE[b != 0] or (ramp is not None and self.power != target)):
      self.release += 1
      self.curvature = self._base(b)
    else:
      self.power = target
      self.curvature = maneuver_curvature(maneuver, self.step_frame, b, self._base(b))
      self.step_frame += 1

    label = maneuver if self.step_frame > 0 else "hold"
    row = (f"{time.monotonic():.3f}", f"{center:.1f}", self.step_idx[b], phase, label, self.step_frame, target, self.power,
           f"{self.curvature:.5f}", f"{curvature_last:.5f}", power_last, f"{curvature_meas:.5f}", f"{steering_angle:.1f}",
           f"{steering_rate:.1f}", f"{yaw_rate:.4f}", hca_status, driver_torque, f"{v_ego:.2f}")
    self._write(",".join(str(x) for x in row))

    out = (self.curvature, int(self.power))
    if self.step_frame >= maneuver_length(maneuver, b):
      self._next_step(b)
      if self.done[b]:
        self._end_band(f"band {center:.1f} m/s completed")
    return out
