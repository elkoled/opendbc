import unittest
from types import SimpleNamespace

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.interface import CarInterface
from opendbc.car.psa.values import CAR, CarControllerParams


class TestPsaCarController(unittest.TestCase):
  def setUp(self):
    CP = CarInterface.get_params(CAR.PSA_PEUGEOT_508, {0: {}, 1: {}, 2: {}}, [], False, False, False)
    self.cc = CarController({Bus.main: 'psa_aee2010_r3', Bus.pt: 'psa_aee2010_r3'}, CP.as_reader())
    self.parser = CANParser('psa_aee2010_r3', [('LANE_KEEP_ASSIST', float('nan'))], 0)
    self.t = 0

  def _step(self, request=0., angle=0., v_ego=10., eps_state=2, pressed=False):
    CC = structs.CarControl(enabled=True, latActive=True)
    CC.actuators.steeringAngleDeg = request
    CS = SimpleNamespace(out=structs.CarState(vEgo=v_ego, vEgoRaw=v_ego, steeringAngleDeg=angle, steeringPressed=pressed),
                         eps_state=eps_state, eps_active=eps_state == 3)
    _, sends = self.cc.update(CC.as_reader(), CS, self.t)
    self.t += 10_000_000
    self.parser.update([(self.t, [s for s in sends if s[0] == 0x3F2])])
    return self.parser.vl['LANE_KEEP_ASSIST']

  def test_activation_sequence(self):
    # the 508 EPS ignores single frame steps, each step is held
    status = [self._step()['STATUS'] for _ in range(60)]
    runs = [1]
    for a, b in zip(status[:-1], status[1:], strict=True):
      if a == b:
        runs[-1] += 1
      else:
        assert (a, b) in ((2, 3), (3, 4), (4, 2)), (a, b)
        runs.append(1)
    self.assertGreaterEqual(min(runs[1:-1]), CarControllerParams.STATUS_STEP)
    self.assertEqual(set(status), {2, 3, 4})

  def test_hold_active_while_authorized(self):
    for _ in range(50):
      self.assertEqual(self._step(eps_state=1)['STATUS'], 4)

  def test_follow_wheel_until_eps_active(self):
    # ramps to the wheel within the rate limit, then stays there whatever openpilot requests
    for i in range(100):
      out = self._step(request=10., angle=3.)
      if i >= 10:
        self.assertAlmostEqual(out['SET_ANGLE'], 3., delta=0.1)
    for _ in range(100):
      out = self._step(request=10., angle=3., eps_state=3)
    self.assertAlmostEqual(out['SET_ANGLE'], 10., delta=0.1)


if __name__ == '__main__':
  unittest.main()
