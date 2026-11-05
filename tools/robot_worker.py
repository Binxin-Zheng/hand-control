"""EXPLICIT hardware worker. Never launched by tests/demos/imports.

Exclusive serial ownership, no homing/calibration or torque-off on dropout. The
hold-current-position stop strategy requires local physical validation; it is not
an independently certified brake. Physical E-stop is required for bus/process failure.
"""
import argparse
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import select
import sys
import time

# Load the stdlib-only policy without importing NumPy/SciPy into the SDK environment.
spec = importlib.util.spec_from_file_location('driver_policy', Path(__file__).resolve().parents[1] /
                                             'src/anyteleop_so101/runtime/driver_policy.py')
policy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(policy)


class RawFeetechPort:
    def __init__(self, config):
        self.config = config
        self.calibration = policy.Calibration(config['joints'])
        self.bus = None
        self.lock = None
        self.safe_to_hold = False

    def open_and_arm(self, enable_torque=False):
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
        # Kernel exclusive serial mode plus process advisory lock.
        import hashlib
        lock_path = '/tmp/hand-control-' + hashlib.sha256(os.path.realpath(self.config['port']).encode()).hexdigest()[:20] + '.lock'
        self.lock = open(lock_path, 'a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        motors = {name: Motor(c['id'], 'sts3215', MotorNormMode.DEGREES)
                  for name, c in self.config['joints'].items()}
        self.bus = FeetechMotorsBus(port=self.config['port'], motors=motors)
        self.bus.connect()
        self.bus.set_timeout(timeout_ms=20)
        self.bus.port_handler.ser.write_timeout = .02
        import termios
        fcntl.ioctl(self.bus.port_handler.ser.fileno(), termios.TIOCEXCL)
        # Refuse to reconfigure an unknown mode or calibration on a live robot.
        if any(self.bus.read('Mode', name, normalize=False) != 0 for name in motors):
            raise RuntimeError('All motors must already be in validated position mode')
        measured = self.read()
        torques = {name: self.bus.read('Torque_Enable', name, normalize=False) for name in motors}
        if any(value != 1 for value in torques.values()) and not enable_torque:
            raise RuntimeError('Torque is off; explicit --enable-torque is required after local acceptance')
        self.safe_to_hold = True
        self.stop()
        # Only initialize gripper goal if torque was off. Never relax an already loaded gripper.
        if torques['gripper'] != 1:
            raw = self.calibration.to_raw('gripper', measured['aperture'])
            self.bus.sync_write('Goal_Position', {'gripper': raw}, normalize=False)
        if enable_torque:
            self.bus.enable_torque()

    def read(self):
        raw = self.bus.sync_read('Present_Position', normalize=False, num_retry=0)
        values = {name: self.calibration.from_raw(name, raw[name]) for name in self.calibration.joints}
        return {'timestamp_s': time.monotonic(), 'q': [values[n] for n in policy.ARM_NAMES],
                'aperture': values['gripper']}

    def send(self, q, aperture):
        values = dict(zip(policy.ARM_NAMES, q)); values['gripper'] = aperture
        self.bus.sync_write('Goal_Position', {n: self.calibration.to_raw(n, v) for n, v in values.items()}, normalize=False)

    def stop(self):
        # Read CURRENT arm position; do not reuse a distant target or modify gripper load.
        raw = self.bus.sync_read('Present_Position', motors=list(policy.ARM_NAMES), normalize=False, num_retry=0)
        for name, value in raw.items():
            self.calibration.from_raw(name, value)
        self.bus.sync_write('Goal_Position', raw, normalize=False)

    def close(self):
        if self.bus is not None and self.bus.is_connected:
            self.bus.disconnect(disable_torque=False)
        if self.lock is not None:
            self.lock.close()


def validate_acceptance(config):
    required = ('joint_calibration', 'tcp', 'collision_scene', 'command_watchdog',
                'physical_estop', 'hold_stop', 'camera_geometry', 'serial_timeout')
    acceptance = config.get('acceptance', {})
    if not acceptance.get('signed_by') or not acceptance.get('date'):
        raise ValueError('Local hardware acceptance must be signed and dated')
    if any(acceptance.get(key) is not True for key in required):
        raise ValueError('Incomplete local hardware acceptance')
    if not isinstance(config.get('port'), str) or not config['port'].startswith('/dev/'):
        raise ValueError('An explicit Linux serial device is required')
    policy.Calibration(config['joints'])


def run(config, enable_torque):
    validate_acceptance(config)
    port = RawFeetechPort(config)
    lease = policy.CommandLease()
    buffer = b''
    holding = True
    os.set_blocking(sys.stdout.fileno(), False)
    try:
        port.open_and_arm(enable_torque)
        while True:
            state = port.read()
            now = time.monotonic()
            if lease.poll(now):
                port.stop(); holding = True
            readable, _, _ = select.select([sys.stdin], [], [], .01)
            if readable:
                chunk = os.read(sys.stdin.fileno(), 65536)
                if not chunk:
                    break
                buffer += chunk
                if len(buffer) > 65536:
                    raise ValueError('Command buffer overflow')
                complete = buffer.split(b'\n')[:-1]
                if len(complete) > 1:
                    lease.stop('command_backlog'); port.stop(); holding = True
                    buffer = buffer.rsplit(b'\n', 1)[-1]
                # Process every control transition in order. No queued old target
                # can execute: commands are age-checked just before each write.
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    try:
                        command = lease.accept(json.loads(line), time.monotonic(), state)
                        if command is None:
                            if not holding:
                                port.stop(); holding = True
                        else:
                            if time.monotonic() >= lease.deadline:
                                raise ValueError('Command expired during validation')
                            port.send(*command); holding = False
                    except (ValueError, KeyError, TypeError) as exc:
                        lease.stop(str(exc))
                        if not holding: port.stop()
                        holding = True
            state.update(schema='teleop.robot_state.v1', reason=lease.reason)
            try:
                os.write(sys.stdout.fileno(), (json.dumps(state, allow_nan=False)+'\n').encode())
            except BlockingIOError:
                lease.stop('controller_not_reading')
                if not holding: port.stop()
                holding = True
    finally:
        try:
            if port.safe_to_hold and port.bus is not None and port.bus.is_connected:
                port.stop()
        finally:
            port.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--enable-torque', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        parser.error('Physical control requires --execute')
    import signal
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    run(json.loads(args.config.read_text()), args.enable_torque)
