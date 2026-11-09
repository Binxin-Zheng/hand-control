"""Explicit read-only servo inspection. No goal, torque, or calibration writes."""
import argparse
from dataclasses import asdict
import fcntl
import hashlib
import json
import os
from pathlib import Path
import termios


def inspect(port, calibration_path):
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    class ReadOnlyBus(FeetechMotorsBus):
        def write(self, *args, **kwargs):
            raise RuntimeError('Motor register writes are forbidden during inspection')

        def sync_write(self, *args, **kwargs):
            raise RuntimeError('Motor register writes are forbidden during inspection')

        def _write(self, *args, **kwargs):
            raise RuntimeError('Motor register writes are forbidden during inspection')

    source = calibration_path.read_bytes()
    expected = json.loads(source)
    names = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
    if set(expected) != set(names):
        raise ValueError('Expected all six named SO101 motors')
    motors = {name: Motor(expected[name]['id'], 'sts3215', MotorNormMode.DEGREES) for name in names}
    lock_path = '/tmp/hand-control-' + hashlib.sha256(os.path.realpath(port).encode()).hexdigest()[:20] + '.lock'
    with open(lock_path, 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        bus = ReadOnlyBus(port=port, motors=motors)
        try:
            bus.connect()
            fcntl.ioctl(bus.port_handler.ser.fileno(), termios.TIOCEXCL)
            bus.set_timeout(timeout_ms=20)
            bus.port_handler.ser.write_timeout = .02
            actual = {name: asdict(value) for name, value in bus.read_calibration().items()}
            return dict(port=port, calibration_path=str(calibration_path),
                        calibration_sha256=hashlib.sha256(source).hexdigest(),
                        expected_calibration=expected, actual_calibration=actual,
                        calibration_matches=actual == expected,
                        position_raw=bus.sync_read('Present_Position', normalize=False, num_retry=0),
                        mode={n: bus.read('Operating_Mode', n, normalize=False) for n in names},
                        torque_enabled={n: bus.read('Torque_Enable', n, normalize=False) for n in names},
                        note='Read-only snapshot, not URDF alignment or motion acceptance.')
        finally:
            if bus.is_connected:
                bus.disconnect(disable_torque=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--read-hardware', action='store_true')
    args = parser.parse_args()
    if not args.read_hardware:
        parser.error('Explicit --read-hardware is required; close other serial owners first')
    if args.out.exists():
        parser.error('Output already exists; choose a new filename')
    result = inspect(args.port, args.calibration)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('x') as f:
        json.dump(result, f, indent=2)
        f.write('\n')
    print(json.dumps(result, indent=2))
