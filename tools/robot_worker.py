"""EXPLICIT hardware worker. Never launched by tests/demos/imports.

Exclusive serial ownership, no homing/calibration or torque-off on dropout. The
hold-current-position stop strategy requires local physical validation; it is not
an independently certified brake. This supervised lab demo accepts a software
stop; it does not protect against bus/process failure.
"""
import argparse
from dataclasses import asdict
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


def configure_packet_timeout(handler, timeout_ms=20):
    """Bound the installed SDK's per-packet receive timer on this port only.

    LeRobot's Feetech patch replaces a prior set_timeout() on every transaction
    with wire time + 50 ms. Wrap both SDK setters, including connection-time
    calls, rather than setting a timer once. This is not a whole-call deadline:
    OS scheduling, serial flush, and a failed bus still need local validation.
    """
    if not policy.finite(timeout_ms) or timeout_ms <= 0:
        raise ValueError('Packet timeout must be finite and positive')
    packet_timeout = handler.setPacketTimeout
    millis_timeout = handler.setPacketTimeoutMillis
    # The installed SDK otherwise measures time using the adjustable wall clock.
    handler.getCurrentTime = lambda: time.monotonic() * 1000.

    def set_packet_timeout(packet_length):
        packet_timeout(packet_length)
        handler.packet_timeout = min(handler.packet_timeout, timeout_ms)

    def set_millis_timeout(milliseconds):
        if not policy.finite(milliseconds) or milliseconds <= 0:
            raise ValueError('SDK requested an invalid packet timeout')
        millis_timeout(min(milliseconds, timeout_ms))

    handler.setPacketTimeout = set_packet_timeout
    handler.setPacketTimeoutMillis = set_millis_timeout
    handler.setPacketTimeoutMillis(timeout_ms)


def expected_servo_calibration(config):
    """Validate the firmware calibration pinned when the joint map was accepted."""
    expected = config.get('expected_servo_calibration')
    names = set(policy.ARM_NAMES) | {'gripper'}
    fields = {'id', 'drive_mode', 'homing_offset', 'range_min', 'range_max'}
    if not isinstance(expected, dict) or set(expected) != names:
        raise ValueError('Pin expected_servo_calibration for all six motors before hardware startup')
    for name, item in expected.items():
        if not isinstance(item, dict) or set(item) != fields or any(type(v) is not int for v in item.values()):
            raise ValueError('Invalid expected servo calibration: '+name)
        joint = config['joints'][name]
        if (item['id'] != joint['id'] or item['drive_mode'] != 0
                or not -2047 <= item['homing_offset'] <= 2047
                or item['range_min'] != joint['raw_min'] or item['range_max'] != joint['raw_max']):
            raise ValueError('Expected firmware calibration does not match joint mapping: '+name)
    return expected


class RawFeetechPort:
    def __init__(self, config):
        self.config = config
        self.calibration = policy.Calibration(config['joints'])
        self.bus = None
        self.lock = None
        self.safe_to_hold = False
        self.startup_arm_raw = None
        self.startup_aperture = None
        self.feedback_max_age_s = None

    def open_and_arm(self, enable_torque=False, *, deadline_s=None):
        # Only the explicit commissioning mode adds this completed-read guard;
        # normal startup retains its existing acceptance and execution policy.
        self.feedback_max_age_s = policy.CommandLease().max_lease if deadline_s is not None else None
        expected = expected_servo_calibration(self.config)
        policy.gripper_angle_range(self.config)
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
        configure_packet_timeout(self.bus.port_handler)
        self.bus.connect()
        self.bus.port_handler.ser.timeout = 0
        self.bus.port_handler.ser.write_timeout = .02
        import termios
        fcntl.ioctl(self.bus.port_handler.ser.fileno(), termios.TIOCEXCL)
        # Refuse to reconfigure an unknown mode or calibration on a live robot.
        if any(self.bus.read('Operating_Mode', name, normalize=False) != 0 for name in motors):
            raise RuntimeError('All motors must already be in validated position mode')
        actual = {name: asdict(value) for name, value in self.bus.read_calibration().items()}
        if actual != expected:
            changed = sorted(name for name in set(expected) | set(actual) if actual.get(name) != expected.get(name))
            raise RuntimeError('Servo calibration changed since joint mapping was accepted: '+', '.join(changed))
        measured_started = time.monotonic()
        measured = self.read()
        torques = {name: self.bus.read('Torque_Enable', name, normalize=False) for name in motors}
        if any(value != 1 for value in torques.values()) and not enable_torque:
            raise RuntimeError('Torque is off; explicit --enable-torque is required after local acceptance')
        if deadline_s is not None and time.monotonic() >= deadline_s:
            raise TimeoutError('Commissioning duration expired before startup hold')
        if self.feedback_max_age_s is not None:
            validate_commission_feedback(measured, time.monotonic(), measured_started, self.feedback_max_age_s)
        self.safe_to_hold = True
        self.startup_arm_raw = self.stop()
        self.startup_aperture = measured['aperture']
        # Only initialize gripper goal if torque was off. Never relax an already loaded gripper.
        if torques['gripper'] != 1:
            if self.feedback_max_age_s is not None:
                validate_commission_feedback(measured, time.monotonic(), measured_started, self.feedback_max_age_s)
            raw = self.calibration.to_raw('gripper', measured['aperture'])
            self.bus.sync_write('Goal_Position', {'gripper': raw}, normalize=False)
        if enable_torque:
            if deadline_s is not None and time.monotonic() >= deadline_s:
                raise TimeoutError('Commissioning duration expired before torque enabling')
            if self.feedback_max_age_s is not None:
                validate_commission_feedback(measured, time.monotonic(), measured_started, self.feedback_max_age_s)
            self.bus.enable_torque()

    def read(self):
        raw = self.bus.sync_read('Present_Position', normalize=False, num_retry=0)
        values = {name: self.calibration.from_raw(name, raw[name]) for name in self.calibration.joints}
        return {'timestamp_s': time.monotonic(), 'q': [values[n] for n in policy.ARM_NAMES],
                'aperture': values['gripper']}

    def send(self, q, aperture):
        values = dict(zip(policy.ARM_NAMES, q)); values['gripper'] = aperture
        self.bus.sync_write('Goal_Position', {n: self.calibration.to_raw(n, v) for n, v in values.items()}, normalize=False)

    def send_fixed_arm_anchor(self, anchor_raw):
        if self.startup_arm_raw is None or anchor_raw != self.startup_arm_raw:
            raise ValueError('Commissioning may only write its exact startup arm anchor')
        self.bus.sync_write('Goal_Position', dict(anchor_raw), normalize=False)

    def stop(self):
        # Read CURRENT arm position; do not reuse a distant target or modify gripper load.
        read_started = time.monotonic()
        raw = self.bus.sync_read('Present_Position', motors=list(policy.ARM_NAMES), normalize=False, num_retry=0)
        if not isinstance(raw, dict) or set(raw) != set(policy.ARM_NAMES):
            raise ValueError('Complete current arm feedback is required for hold')
        for name, value in raw.items():
            if type(value) is not int:
                raise ValueError('Current arm feedback must contain integer encoder positions')
            self.calibration.from_raw(name, value)
        if self.feedback_max_age_s is not None and not 0 <= time.monotonic()-read_started <= self.feedback_max_age_s:
            raise ValueError('Current arm feedback read exceeded its deadline')
        self.bus.sync_write('Goal_Position', raw, normalize=False)
        return dict(raw)

    def close(self):
        if self.bus is not None and self.bus.is_connected:
            self.bus.disconnect(disable_torque=False)
        if self.lock is not None:
            self.lock.close()


def validate_acceptance(config):
    required = ('joint_calibration', 'tcp', 'collision_scene', 'command_watchdog',
                'software_stop', 'hold_stop', 'camera_geometry', 'serial_timeout')
    acceptance = config.get('acceptance', {})
    missing = [key for key in required if acceptance.get(key) is not True]
    incomplete = [name for name, item in config.get('joints', {}).items()
                  if any(item.get(key) is None for key in ('zero_raw', 'raw_per_unit', 'lower', 'upper'))]
    if missing or incomplete:
        details = ['unverified setup: '+', '.join(missing)] if missing else []
        if incomplete:
            details.append('missing joint mapping: '+', '.join(incomplete))
        raise ValueError('Robot setup incomplete; '+'; '.join(details)+
                         '. See docs/10_DEMO_COMPLETION.md. Do not fill unknown values just to launch.')
    if not isinstance(config.get('port'), str) or not config['port'].startswith('/dev/'):
        raise ValueError('An explicit Linux serial device is required')
    policy.Calibration(config['joints'])
    expected_servo_calibration(config)
    policy.gripper_angle_range(config)


def validate_commission_hold(config, duration_s, pose_confirmed):
    if pose_confirmed is not True:
        raise ValueError('Commissioning requires --pose-confirmed: current arm is supported and clear')
    if not policy.finite(duration_s) or not 0 < duration_s <= 30:
        raise ValueError('Commissioning requires an explicit finite --duration in (0, 30] seconds')
    if not isinstance(config.get('port'), str) or not config['port'].startswith('/dev/'):
        raise ValueError('An explicit Linux serial device is required')
    policy.Calibration(config['joints'])
    expected_servo_calibration(config)
    policy.gripper_angle_range(config)


def validate_commission_feedback(state, now_s, read_started_s, max_age_s):
    timestamp = state.get('timestamp_s')
    q, aperture = state.get('q'), state.get('aperture')
    if (not policy.finite(timestamp) or timestamp > now_s+.005 or now_s-timestamp > max_age_s
            or not 0 <= now_s-read_started_s <= max_age_s):
        raise ValueError('Commissioning feedback is stale, future, or read exceeded its deadline')
    if (not isinstance(q, list) or len(q) != 5 or not all(policy.finite(value) for value in q)
            or not policy.finite(aperture) or not 0 <= aperture <= 1):
        raise ValueError('Invalid commissioning feedback')


class CommissionHold:
    """One bounded active phase at the exact startup hold target; no trajectories."""
    def __init__(self, calibration, anchor_raw, aperture, started_s, duration_s):
        if not isinstance(anchor_raw, dict) or set(anchor_raw) != set(policy.ARM_NAMES):
            raise ValueError('Exact startup raw arm anchor is required')
        if any(type(value) is not int for value in anchor_raw.values()):
            raise ValueError('Startup raw arm anchor must contain integer encoder positions')
        self.anchor_raw = dict(anchor_raw)
        self.anchor_q = [calibration.from_raw(name, anchor_raw[name]) for name in policy.ARM_NAMES]
        if not policy.finite(aperture) or not 0 <= aperture <= 1:
            raise ValueError('Invalid startup aperture')
        self.anchor_aperture = aperture
        self.started_s, self.duration_s = started_s, duration_s
        self.active_started = self.active_finished = False
        self.finish_reason = None

    def accept_active(self, q):
        if self.active_finished:
            raise ValueError('Commissioning active phase ended; start a new invocation for a fresh anchor')
        if q != self.anchor_q:
            raise ValueError('Commissioning active target must equal the startup arm anchor')
        self.active_started = True

    def finish(self, reason, *, force=False):
        if (self.active_started or force) and not self.active_finished:
            self.active_finished = True
            self.finish_reason = reason

    def feedback(self, now_s):
        return dict(mode='hold', anchor_raw=self.anchor_raw, anchor_q=self.anchor_q,
                    anchor_aperture=self.anchor_aperture, active_started=self.active_started,
                    active_finished=self.active_finished, finish_reason=self.finish_reason,
                    elapsed_s=now_s-self.started_s, duration_s=self.duration_s)


def run(config, enable_torque, *, commission_hold=False, duration_s=None, pose_confirmed=False):
    if commission_hold:
        validate_commission_hold(config, duration_s, pose_confirmed)
    else:
        validate_acceptance(config)
    port = RawFeetechPort(config)
    lease = policy.CommandLease()
    commissioning = None
    started_s = time.monotonic()
    end_s = started_s+duration_s if commission_hold else None
    buffer = b''
    holding = True
    os.set_blocking(sys.stdout.fileno(), False)
    try:
        if commission_hold:
            port.open_and_arm(enable_torque, deadline_s=end_s)
            commissioning = CommissionHold(port.calibration, port.startup_arm_raw,
                                           port.startup_aperture, started_s, duration_s)
        else:
            port.open_and_arm(enable_torque)
        while True:
            read_started = time.monotonic()
            state = port.read()
            now = time.monotonic()
            read_duration = now-read_started
            closing_reason = 'commission_duration_elapsed' if commissioning is not None and now >= end_s else None
            if commissioning:
                try:
                    validate_commission_feedback(state, now, read_started, lease.max_lease)
                except ValueError as exc:
                    closing_reason = str(exc)
            closing = closing_reason is not None
            if closing:
                commissioning.finish(closing_reason, force=True)
                lease.stop(closing_reason); port.stop(); holding = True
            if lease.poll(now):
                if commissioning: commissioning.finish('command_expired')
                port.stop(); holding = True
            readable = [] if closing else select.select([sys.stdin], [], [], .01)[0]
            if readable:
                chunk = os.read(sys.stdin.fileno(), 65536)
                if not chunk:
                    if commissioning is None:
                        break
                    commissioning.finish('controller_eof', force=True)
                    lease.stop('controller_eof'); port.stop(); holding = True
                    closing = True
                buffer += chunk
                if len(buffer) > 65536:
                    raise ValueError('Command buffer overflow')
                complete = buffer.split(b'\n')[:-1]
                if len(complete) > 1:
                    if commissioning: commissioning.finish('command_backlog')
                    lease.stop('command_backlog'); port.stop(); holding = True
                    buffer = buffer.rsplit(b'\n', 1)[-1]
                # Process every control transition in order. No queued old target
                # can execute: commands are age-checked just before each write.
                while not closing and b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    try:
                        command_now = time.monotonic()
                        if commissioning and lease.poll(command_now):
                            commissioning.finish('command_expired'); port.stop(); holding = True
                        packet = json.loads(line)
                        command = lease.accept(packet, command_now, state)
                        if command is None:
                            if commissioning: commissioning.finish(lease.reason)
                            if not holding:
                                port.stop(); holding = True
                        else:
                            if time.monotonic() >= lease.deadline:
                                raise ValueError('Command expired during validation')
                            if commissioning:
                                if time.monotonic() >= end_s:
                                    raise ValueError('Commissioning duration expired before write')
                                validate_commission_feedback(state, time.monotonic(), read_started, lease.max_lease)
                                commissioning.accept_active(command[0])
                                port.send_fixed_arm_anchor(commissioning.anchor_raw)
                            else:
                                port.send(*command)
                            holding = False
                    except (ValueError, KeyError, TypeError) as exc:
                        if commissioning: commissioning.finish(str(exc))
                        lease.stop(str(exc))
                        if not holding: port.stop()
                        holding = True
            state.update(schema='teleop.robot_state.v1', reason=lease.reason)
            if commissioning:
                state['commissioning'] = commissioning.feedback(time.monotonic())
                state['commissioning']['read_duration_s'] = read_duration
            try:
                os.write(sys.stdout.fileno(), (json.dumps(state, allow_nan=False)+'\n').encode())
            except BlockingIOError:
                if commissioning: commissioning.finish('controller_not_reading')
                lease.stop('controller_not_reading')
                if not holding: port.stop()
                holding = True
            if closing:
                break
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
    parser.add_argument('--commission-hold', action='store_true',
                        help='Bounded supervised startup-anchor hold only; never accepts a trajectory')
    parser.add_argument('--duration', type=float, help='Required hold-commissioning deadline, at most 30 seconds')
    parser.add_argument('--pose-confirmed', action='store_true',
                        help='Operator confirms the current arm is supported and clear for a hold test')
    args = parser.parse_args()
    if not args.execute:
        parser.error('Physical control requires --execute')
    if not args.commission_hold and (args.duration is not None or args.pose_confirmed):
        parser.error('--duration and --pose-confirmed require --commission-hold')
    import signal
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    run(json.loads(args.config.read_text()), args.enable_torque, commission_hold=args.commission_hold,
        duration_s=args.duration, pose_confirmed=args.pose_confirmed)
