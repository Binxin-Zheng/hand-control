"""Explicit subprocess boundary: SDK and serial ownership stay in robot worker."""
import json
import os
from pathlib import Path
import subprocess
import time
import numpy as np
from ..session import RobotState


class RobotWorkerClient:
    def __init__(self, python, config_path, *, enable_torque=False):
        worker = Path(__file__).resolve().parents[3] / 'tools/robot_worker.py'
        command = [python, str(worker), '--config', str(config_path), '--execute']
        if enable_torque:
            command.append('--enable-torque')
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        os.set_blocking(self.process.stdin.fileno(), False)
        os.set_blocking(self.process.stdout.fileno(), False)
        self.sequence = 0
        self.buffer = b''
        self.state = None
        self.reason = 'starting'

    def read_state(self):
        if self.process.poll() is not None:
            raise RuntimeError('Robot worker exited; physical E-stop may be required')
        try:
            chunk = os.read(self.process.stdout.fileno(), 65536)
        except BlockingIOError:
            chunk = b''
        self.buffer += chunk
        if len(self.buffer) > 65536:
            raise RuntimeError('Robot feedback backlog')
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            packet = json.loads(line)
            if packet['schema'] != 'teleop.robot_state.v1':
                raise ValueError('Invalid robot feedback schema')
            self.state = RobotState(packet['timestamp_s'], np.asarray(packet['q'], dtype=float), packet['aperture'])
            self.reason = packet['reason']
        return self.state

    def send(self, mode, q=None, aperture=None):
        self.sequence += 1
        packet = dict(sequence=self.sequence, deadline_s=time.monotonic()+.08, mode=mode,
                      q=None if q is None else np.asarray(q).tolist(), aperture=aperture)
        data = (json.dumps(packet, allow_nan=False)+'\n').encode()
        count = os.write(self.process.stdin.fileno(), data)
        if count != len(data):
            raise RuntimeError('Partial driver command; worker will expire lease')

    def close(self):
        try:
            if self.process.poll() is None:
                self.process.stdin.close()  # EOF triggers hold and clean disconnect.
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.terminate()
                    raise RuntimeError('Driver unresponsive; use physical E-stop')
        finally:
            self.process.stdout.close()
