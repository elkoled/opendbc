import json
import math
from collections import defaultdict
from types import SimpleNamespace

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.volkswagen import power_test
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, DBC, VolkswagenSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py

PLATFORM = CAR.VOLKSWAGEN_ID4_MK2
DEG_PER_CURVATURE = 2600.


def run_test(tmp_path, monkeypatch, speeds: list[float], fault_power: int = 1000, max_s: float = 300.) -> tuple[CarController, int, list[dict]]:
  # closed loop: car controller with the power test, panda safety and a rate limited first order EPS, at each speed in turn
  monkeypatch.setattr(power_test, "ARM_FILE", str(tmp_path / "arm"))
  monkeypatch.setattr(power_test, "LOG_DIR", str(tmp_path / "logs"))
  monkeypatch.setattr(power_test, "STATUS_FILE", str(tmp_path / "status.json"))
  (tmp_path / "arm").write_text(f"max_speed={max(speeds)}")

  packer = CANPacker(DBC[PLATFORM][Bus.pt])
  safety = libsafety_py.libsafety
  safety.set_safety_hooks(structs.CarParams.SafetyModel.volkswagenMeb, VolkswagenSafetyFlags.MEB_ALT_CRC)
  safety.init_tests()
  safety.set_controls_allowed(True)

  def rx(name, values):
    addr, dat, _ = packer.make_can_msg(name, 0, values)
    safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, 0, dat))

  CP = CarInterface.get_non_essential_params(PLATFORM)
  CP.openpilotLongitudinalControl = False
  cc = CarController(DBC[PLATFORM], CP)
  eps, hist, power, faulted, blocked, statuses = 0., [0.] * 5, 0, False, 0, []
  engage_frame = 0
  for frame in range(int(max_s / DT_CTRL)):
    t = frame * DT_CTRL
    band = next((i for i, d in enumerate(cc.power_test.done) if not d and power_test.BANDS[i][0] <= max(speeds)), None)
    if band is None:
      break
    v = power_test.BANDS[band][0]
    hist.append(cc.apply_curvature_last if power > 0 else eps)
    c = hist.pop(0)
    faulted |= power >= fault_power
    if not faulted and power > 0:
      rate = 230. * power / 50. / DEG_PER_CURVATURE
      eps += float(np.clip(DT_CTRL / 0.15 * (0.95 * c - eps), -rate * DT_CTRL, rate * DT_CTRL))
    if faulted and cc.steering_power_last == 0:
      # power cycle, then engaged again a second later
      faulted = False
      cc = CarController(DBC[PLATFORM], CP)
      engage_frame = frame + 50

    safety.set_timer(int(t * 1e6))
    rx("ESC_51", {f"{w}_Radgeschw": v * 3.6 for w in ("HL", "HR", "VL", "VR")})
    rx("ESP_21", {"ESP_v_Signal": v * 3.6})
    rx("ESC_50", {"Yaw_Rate": abs(math.degrees(eps * v)), "Yaw_Rate_Sign": eps * v > 0})
    rx("QFK_01", {"Curvature": abs(eps), "Curvature_VZ": eps > 0})

    CC = structs.CarControl(latActive=not faulted and frame >= engage_frame, enabled=True)
    CC.currentCurvature = eps
    CS = SimpleNamespace(out=structs.CarState(vEgo=v, vEgoRaw=v, yawRate=eps * v, steeringAngleDeg=eps * DEG_PER_CURVATURE, standstill=v == 0.),
                         curvature_meas=eps, eps_stock_values=defaultdict(int), klr_stock_values=defaultdict(int), acc_type=0,
                         gra_stock_values=defaultdict(int), ldw_stock_values=defaultdict(int), gra_stock_counter=0,
                         hca_status="FAULT" if faulted else "ACTIVE" if power else "READY")
    _, sends = cc.update(CC.as_reader(), CS, int(t * 1e9))
    for addr, dat, bus in sends:
      if bus == 0 and not safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, 0, dat)):
        blocked += 1
    if frame % 2 == 0:
      power = cc.steering_power_last
    if frame % 100 == 0 and (tmp_path / "status.json").exists():
      statuses.append(json.loads((tmp_path / "status.json").read_text()))
  return cc, blocked, statuses


class TestPowerTest:
  def test_standstill_and_rolling_complete(self, tmp_path, monkeypatch):
    cc, blocked, statuses = run_test(tmp_path, monkeypatch, [0., 1.5])
    assert cc.power_test.done[:2] == [True, True]
    assert blocked == 0
    assert any(s["active"] and s["power"] == 100 for s in statuses)
    # the logs fit back to the EPS model
    groups = defaultdict(list)
    for run in power_test_fit_runs(tmp_path):
      groups[(run[0], run[1])].append(run[2:])
    from opendbc.car.volkswagen.power_test_fit import fit
    f = fit(groups[(1.5, 50)])
    assert f is not None and 0.1 < f["tau"] < 0.25 and 0.85 < f["gain"] < 1.05

  def test_fault_skips_higher_powers(self, tmp_path, monkeypatch):
    cc, blocked, _ = run_test(tmp_path, monkeypatch, [0.], fault_power=80)
    assert cc.power_test.done[0]
    assert cc.power_test.level_cap[0] is not None and cc.power_test.level_cap[0] <= 80
    assert blocked == 0
    logged = (tmp_path / "logs" / "result.txt").read_text()
    assert "FAULT" in logged and "completed" in logged


def power_test_fit_runs(tmp_path):
  from opendbc.car.volkswagen.power_test_fit import load
  for run in load([str(tmp_path / "logs")]):
    rows = [r for r in run if r["maneuver"] in ("small", "big", "turn", "sweep") and r["power_sent"] == r["target_power"]]
    if len(rows) >= 25:
      yield (float(rows[0]["band"]), int(rows[0]["target_power"]), np.array([float(r["curvature_sent"]) for r in rows]),
             np.array([float(r["curvature_meas"]) for r in rows]))


def test_long_command_drives_the_test(tmp_path, monkeypatch):
  monkeypatch.setattr(power_test, "ARM_FILE", str(tmp_path / "arm"))
  monkeypatch.setattr(power_test, "LOG_DIR", str(tmp_path / "logs"))
  t = power_test.MebPowerTest()
  assert t.long_command(0., True) is None
  (tmp_path / "arm").write_text("")
  # standstill band first: hold
  assert t.long_command(0., True) == (power_test.STOP_ACCEL, True)
  # then drive to the next band
  t.done = [True, False, False, False]
  accel, stopping = t.long_command(0., True)
  assert accel > 0. and not stopping
  # a gas override hands the pedals back
  assert t.long_command(1., False) is None
  assert t.long_command(3., True) is not None
  # finished: stop and hold until disengaged
  t.done = [True] * 4
  t.finished = True
  assert t.long_command(2., True) == (power_test.SPEED_ACCEL[0], False)
  assert t.long_command(0., True) == (power_test.STOP_ACCEL, True)
  assert t.long_command(0., False) is None
  assert t.long_command(0., True) is None
