"""Keep upstream optimizer dependencies isolated from vision and core."""
import json
from pathlib import Path
import select
import subprocess


class DexClient:
    def __init__(self, python, urdf):
        root = Path(__file__).resolve().parents[3]
        self.process = subprocess.Popen([str(python), str(root/'tools/dex_gripper_worker.py'),
                                         '--urdf', str(urdf)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True, bufsize=1)
        try:
            if not self._read(30).get('ready'):
                raise RuntimeError('Dex optimizer did not initialize')
        except Exception:
            self.close()
            raise

    def _read(self, timeout):
        if not select.select([self.process.stdout], [], [], timeout)[0]:
            raise RuntimeError('Dex optimizer response timed out')
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError('Dex optimizer exited; see terminal error')
        return json.loads(line)

    def retarget(self, points):
        self.process.stdin.write(json.dumps(points.tolist(), allow_nan=False)+'\n')
        self.process.stdin.flush()
        value = self._read(2)
        if 'error' in value:
            raise ValueError(value['error'])
        return value

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
