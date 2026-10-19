"""Exercise the compiled client against deliberate protocol failures."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class ClientTests(unittest.TestCase):
    def run_case(self, reply):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            script = root/'fake.py'
            model = 'a model "quoted" $(touch INJECTED).xml'
            script.write_text('import json,sys\n'
                + 'assert sys.argv[1] == '+repr(model)+'\n'
                + 'print(json.dumps({"ok":True}),flush=True)\n'
                + 'for line in sys.stdin:\n'
                + ' r=json.loads(line)\n'
                + ' if r["cmd"]=="quit":break\n'
                + ' if r["cmd"]=="info":out={"ok":True,"nu":1,"nq":1}\n'
                + ' else:out='+repr(reply)+'\n'
                + ' print(json.dumps(out),flush=True)\n')
            result = subprocess.run([str(ROOT/'zron'), 'mujoco', model, '--bridge',
                f'{sys.executable} {script}', '--cycles', '1'], cwd=root,
                text=True, capture_output=True, timeout=10)
            self.assertFalse((root/'INJECTED').exists())
            return result.returncode, json.loads(result.stdout)

    def test_valid_reply_and_opaque_model_path(self):
        status, reply = self.run_case({'ok':True,'finite':True,'stable':True,'time':.01,'qvel_max':.2})
        self.assertEqual(status, 0)
        self.assertTrue(reply['ok'])
        self.assertIn('"quoted"', reply['model'])

    def test_errors_and_missing_state_fail_closed(self):
        valid={'ok':True,'finite':True,'stable':True,'time':.01,'qvel_max':.2}
        cases=[{'ok':False,'error':'bad control'}, {},
               {**valid,'finite':False}, {**valid,'stable':False},
               {**valid,'qvel_max':None}, {**valid,'qvel_max':'nonsense'},
               {**valid,'qvel_max':1001}, {**valid,'time':0}]
        for case in cases:
            with self.subTest(case=case):
                status, reply=self.run_case(case)
                self.assertNotEqual(status, 0)
                self.assertFalse(reply['ok'])
                self.assertFalse(reply['stable'])

if __name__=='__main__': unittest.main()
