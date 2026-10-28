import json
import pathlib
import subprocess
import tempfile
import unittest
ROOT = pathlib.Path(__file__).resolve().parents[1]
class ReplayTest(unittest.TestCase):
    def test_replay(self):
        with tempfile.TemporaryDirectory() as d:
            csv = pathlib.Path(d)/'capture.csv'
            subprocess.run([str(ROOT/'build-doppler/test'),str(csv)],check=True,capture_output=True)
            cmd = ['python3',str(ROOT/'tools/replay.py'),str(csv),'--height','100']
            first = subprocess.check_output(cmd)
            self.assertEqual(first,subprocess.check_output(cmd))
            rows = [json.loads(line) for line in first.splitlines()]
            self.assertEqual(rows[-1]['status'],'stale')
            self.assertNotIn('lat',rows[-1])
            self.assertEqual(rows[-2]['status'],'valid')
            output = pathlib.Path(d)/'fixes.jsonl';output.write_bytes(first)
            subprocess.run(['python3',str(ROOT/'tools/score_replay.py'),str(output),'--truth','40','-75','100','--max-error','5000'],check=True)
            # Stable sorting must make reverse event arrival order equivalent.
            lines=csv.read_text().splitlines();csv.write_text('\n'.join(lines[:1]+list(reversed(lines[1:])))+'\n')
            self.assertEqual(first,subprocess.check_output(cmd))
            csv.write_text(','.join(['timestamp_ns','sat_id','x4km','y4km','z4km','frequency_hz','sigma_hz'])+'\n-1,0,1,2,3,nan,5\n')
            self.assertNotEqual(subprocess.run(cmd,capture_output=True).returncode,0)
if __name__ == '__main__':unittest.main()
