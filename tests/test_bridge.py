"""Real MuJoCo protocol regressions using a small local model, not Menagerie."""
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('bridge', ROOT/'bridge/zron_mujoco.py')
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)
MODEL = '''<mujoco><worldbody><body><joint name="j" type="hinge"/>
<geom type="capsule" size=".03 .1" mass="1"/></body></worldbody>
<actuator><motor joint="j"/></actuator><keyframe><key qpos=".2" ctrl=".1"/></keyframe></mujoco>'''

class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        (self.root/'models').mkdir()
        self.path = self.root/'models/robot.xml'
        self.path.write_text(MODEL)

    def test_invalid_steps_do_not_mutate_simulation(self):
        model, data = bridge.load(self.path)
        for ctrl, n in [([], 1), ([1, 2], 1), ([[1]], 1), ([True], 1),
                        ([float('nan')], 1), ([float('inf')], 1), ([1], 0),
                        ([1], -2), ([1], 1.5), ([1], True), ([1], 10001)]:
            with self.subTest(ctrl=ctrl, n=n):
                with self.assertRaises(ValueError): bridge.step(model, data, ctrl, n)
                self.assertEqual(data.time, 0)
                self.assertAlmostEqual(data.ctrl[0], .1)
                self.assertAlmostEqual(data.qpos[0], .2)

    def test_relative_model_reset_and_recovery_after_bad_requests(self):
        requests = [[], None, 2, {'cmd':'step', 'ctrl':[], 'n':1},
                    {'cmd':'step', 'ctrl':[.2], 'n':2}, {'cmd':'reset'},
                    {'cmd':'step', 'n':1}, {'cmd':'info'}, {'cmd':'quit'}]
        result = subprocess.run([sys.executable, str(ROOT/'bridge/zron_mujoco.py'), 'models/robot.xml'],
            cwd=self.root, input=''.join(json.dumps(r)+'\n' for r in requests),
            text=True, capture_output=True, timeout=15, check=True)
        replies = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertTrue(replies[0]['ok'])
        self.assertTrue(all(not r['ok'] for r in replies[1:5]))
        self.assertTrue(replies[5]['finite'])
        self.assertTrue(replies[6]['ok'])
        self.assertAlmostEqual(replies[7]['time'], .002)
        self.assertEqual(replies[8]['nu'], 1)

    def test_compiled_client_drives_real_physics(self):
        result = subprocess.run([str(ROOT/'zron'), 'mujoco', 'models/robot.xml',
            '--bridge', f'{sys.executable} {ROOT}/bridge/zron_mujoco.py', '--cycles', '10'],
            cwd=self.root, text=True, capture_output=True, timeout=15, check=True)
        reply = json.loads(result.stdout)
        self.assertTrue(reply['ok'])
        self.assertTrue(reply['stable'])
        self.assertEqual(reply['cycles'], 10)
        self.assertAlmostEqual(reply['t'], .1)

    def test_load_preserves_callers_working_directory(self):
        before = Path.cwd()
        bridge.load(self.path)
        self.assertEqual(Path.cwd(), before)

if __name__ == '__main__': unittest.main()
