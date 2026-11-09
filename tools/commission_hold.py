"""Explicit, bounded physical hold/lease diagnostic; never a motion controller.

The worker restricts all active writes to the raw arm positions captured by its
startup hold. This script cannot send a hand target or authorize normal robot
operation. Results record observed protocol/feedback behavior, not acceptance of
stopping while moving. Importing this module opens no process or device.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import time
import uuid


ROOT = Path(__file__).resolve().parents[1]
EXERCISES = {'command-expiry': 'command_expired', 'software-stop': 'emergency_stop',
             'release': 'released', 'controller-exit': None}


def validate_options(args):
    if not args.execute or not args.pose_confirmed:
        raise ValueError('This physical diagnostic requires explicit --execute and --pose-confirmed')
    if (not math.isfinite(args.hold_seconds) or not .1 <= args.hold_seconds <= 5
            or not math.isfinite(args.duration) or not args.hold_seconds+2 <= args.duration <= 30):
        raise ValueError('Hold must be 0.1..5 seconds; total duration must be hold+2..30 seconds')
    if args.exercise not in EXERCISES:
        raise ValueError('Unknown hold diagnostic')


def worker_command(args, root=ROOT):
    validate_options(args)
    command = [str(args.robot_python.resolve()), str(root/'tools/robot_worker.py'),
               '--config', str(args.config.resolve()), '--execute', '--commission-hold',
               '--duration', str(args.duration), '--pose-confirmed']
    if args.enable_torque:
        command.append('--enable-torque')
    return command


def worker_environment(python):
    env = os.environ.copy()
    for name in ('PYTHONPATH', 'PYTHONHOME', 'VIRTUAL_ENV', 'CONDA_PREFIX',
                 'CONDA_DEFAULT_ENV', 'CONDA_SHLVL'):
        env.pop(name, None)
    env.update(PYTHONNOUSERSITE='1', PYTHONUNBUFFERED='1',
               PATH=str(Path(python).parent)+os.pathsep+env.get('PATH', ''))
    return env


def supervise(command, session, *, exercise, hold_seconds, duration, operator_input=None):
    """Run one release/hold/test sequence. No recovery or process restart."""
    if exercise not in EXERCISES:
        raise ValueError('Unknown hold diagnostic')
    sequence = 0
    started = time.monotonic()
    phase = 'startup'
    buffer = b''
    anchor_q = anchor_aperture = None
    feedback_timestamp = None
    last_feedback_reason = None
    active_at = next_send = action_at = None
    result = dict(schema='teleop.hold_diagnostic.v1', exercise=exercise,
                  physical_execution=True, motion_stop_validated=False,
                  acceptance_changed=False, requested_duration_s=duration,
                  command=command, outcome='incomplete', feedback_frames=0,
                  started_utc=datetime.now(timezone.utc).isoformat())
    process = None
    with (session/'events.jsonl').open('x') as log, (session/'worker.stderr.log').open('xb') as errors:
        def record(kind, **fields):
            log.write(json.dumps(dict(kind=kind, timestamp_s=time.monotonic(), **fields), allow_nan=False)+'\n')
            log.flush()

        def send(mode):
            nonlocal sequence
            if mode == 'active' and (feedback_timestamp is None
                    or not -.005 <= time.monotonic()-feedback_timestamp <= .15):
                raise RuntimeError('Feedback stopped; refusing to renew the active hold lease')
            packet = dict(sequence=sequence, deadline_s=time.monotonic()+.08, mode=mode,
                          q=anchor_q if mode == 'active' else None,
                          aperture=anchor_aperture if mode == 'active' else None)
            data = (json.dumps(packet, allow_nan=False)+'\n').encode()
            if os.write(process.stdin.fileno(), data) != len(data):
                raise RuntimeError('Partial command write; no further commands will be sent')
            sequence += 1
            record('command', packet=packet)

        def read_feedback(*, drain=False):
            nonlocal buffer, feedback_timestamp, last_feedback_reason
            packets = []
            while True:
                try: chunk = os.read(process.stdout.fileno(), 65536)
                except BlockingIOError: break
                buffer += chunk
                if len(buffer) > 65536:
                    raise ValueError('Worker feedback backlog')
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    packet = json.loads(line)
                    if packet.get('schema') != 'teleop.robot_state.v1':
                        raise ValueError('Unexpected worker feedback schema')
                    info = packet.get('commissioning', {})
                    if info.get('mode') != 'hold':
                        raise ValueError('Worker did not enter restricted hold commissioning')
                    age = time.monotonic()-packet['timestamp_s']
                    if not -.005 <= age <= .15:
                        raise ValueError('Stale or future worker feedback')
                    record('feedback', age_s=age, packet=packet)
                    feedback_timestamp = packet['timestamp_s']
                    last_feedback_reason = packet['reason']
                    result['feedback_frames'] += 1
                    packets.append(packet)
                if not drain or not chunk: break
            return packets

        try:
            process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=errors, bufsize=0, cwd=ROOT,
                                       env=worker_environment(command[0]))
            os.set_blocking(process.stdin.fileno(), False)
            os.set_blocking(process.stdout.fileno(), False)
            record('worker_started', pid=process.pid)
            while True:
                now = time.monotonic()
                if now-started > duration+1:
                    raise TimeoutError('Bounded hold diagnostic timed out')
                ready, _, _ = select.select([process.stdout] + ([operator_input] if operator_input is not None else []), [], [], .005)
                if operator_input is not None and operator_input in ready:
                    line = operator_input.readline()
                    if not line:
                        operator_input = None
                    elif line.strip().lower() in ('x', 'stop', 'quit', 'q'):
                        send('estop')
                        result['operator_stop_requested'] = True
                        action_at = time.monotonic()
                        phase = 'operator_stop'
                packets = []
                if process.stdout in ready:
                    packets = read_feedback()
                # Act once on the freshest feedback; log every received row.
                packet = packets[-1] if packets else None
                if packet is not None:
                    info = packet['commissioning']
                    if phase == 'startup':
                        anchor_q = info['anchor_q']
                        anchor_aperture = info.get('anchor_aperture', packet['aperture'])
                        if (not isinstance(anchor_q, list) or len(anchor_q) != 5
                                or not all(isinstance(x, (int, float)) and not isinstance(x, bool)
                                           and math.isfinite(x) for x in (*anchor_q, anchor_aperture))):
                            raise ValueError('Invalid worker hold anchor')
                        result['anchor_raw'] = info['anchor_raw']
                        send('release'); phase = 'await_release'
                    elif phase == 'await_release' and packet['reason'] == 'released':
                        send('active'); phase = 'await_active'
                    elif phase == 'await_active' and packet['reason'] == 'active':
                        active_at = time.monotonic(); next_send = active_at+.02; phase = 'hold'
                    elif phase == 'operator_stop' and packet['reason'] == 'emergency_stop':
                        result.update(outcome='operator_stopped', observed_reason=packet['reason'],
                                      response_s=time.monotonic()-action_at)
                        break
                    elif phase == 'await_result' and packet['reason'] == EXERCISES[exercise]:
                        result.update(outcome='protocol_transition_observed', observed_reason=packet['reason'],
                                      response_s=time.monotonic()-action_at)
                        break
                    elif phase in ('await_active', 'hold') and info.get('active_finished'):
                        raise RuntimeError('Hold ended before requested exercise: '+str(info.get('finish_reason')))
                now = time.monotonic()
                if phase == 'hold':
                    if now-active_at >= hold_seconds:
                        action_at = now; phase = 'await_result'
                        if exercise == 'software-stop': send('estop')
                        elif exercise == 'release': send('release')
                        elif exercise == 'controller-exit': process.stdin.close()
                        else: record('commands_withheld_for_expiry')
                    elif now >= next_send:
                        send('active'); next_send = now+.02
                code = process.poll()
                if code is not None:
                    read_feedback(drain=True)
                    if buffer:
                        raise ValueError('Worker exited with incomplete feedback')
                    if exercise == 'controller-exit' and phase == 'await_result' and code == 0:
                        if last_feedback_reason != 'controller_eof':
                            raise RuntimeError('Worker exited without confirming controller EOF')
                        result.update(outcome='worker_exited_after_controller_eof',
                                      observed_reason=last_feedback_reason, response_s=time.monotonic()-action_at)
                        break
                    raise RuntimeError('Worker exited before the diagnostic completed: '+str(code))
        except BaseException as error:
            result.update(outcome='failed', error=type(error).__name__+': '+str(error))
            raise
        finally:
            if process is not None:
                if not process.stdin.closed: process.stdin.close()
                try:
                    process.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try: process.wait(timeout=2.)
                    except subprocess.TimeoutExpired:
                        process.kill(); process.wait(timeout=1.)
                        result['forced_kill'] = True
                    result['termination_requested'] = True
                try:
                    read_feedback(drain=True)
                    if buffer:
                        raise ValueError('Worker exited with incomplete feedback')
                except Exception as error:
                    result.update(outcome='failed', cleanup_feedback_error=str(error))
                finally:
                    process.stdout.close()
                result['worker_returncode'] = process.returncode
                if process.returncode != 0 or result.get('termination_requested'):
                    result.update(outcome='failed', cleanup_error='Worker did not exit normally after controller EOF')
            result['elapsed_s'] = time.monotonic()-started
            result['limitations'] = ('A fixed-pose hold cannot establish braking while moving, camera direction, '
                                     'collision-scene correctness or protection from driver/bus failure. '
                                     'Review actual arm behavior and retained feedback; no acceptance flags were set.')
            record('summary', result=result)
            (session/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    if result['outcome'] == 'failed':
        raise RuntimeError(result.get('cleanup_error', result.get('error', 'Hold diagnostic failed')))
    return result


def run(args):
    command = worker_command(args)
    # Read inputs and establish unique evidence output before launching a worker.
    config_bytes = args.config.read_bytes()
    json.loads(config_bytes)
    if not args.robot_python.is_file():
        raise ValueError('Select the prepared robot SDK Python interpreter')
    session = args.out/(datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
    session.mkdir(parents=True, exist_ok=False)
    (session/'inputs.json').write_text(json.dumps(dict(
        config_path=str(args.config.resolve()), config_sha256=hashlib.sha256(config_bytes).hexdigest(),
        worker_sha256=hashlib.sha256((ROOT/'tools/robot_worker.py').read_bytes()).hexdigest(),
        operator_confirmed_current_supported_clear_pose=args.pose_confirmed,
        enable_torque_explicit=args.enable_torque), indent=2)+'\n')
    print('Physical hold diagnostic; no commanded excursion. Evidence: '+str(session), flush=True)
    print('Type X then Enter to request software stop. This is a single bounded test, with no automatic recovery.', flush=True)
    result = supervise(command, session, exercise=args.exercise, hold_seconds=args.hold_seconds,
                       duration=args.duration, operator_input=sys.stdin if sys.stdin.isatty() else None)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--robot-python', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--pose-confirmed', action='store_true',
                        help='Operator confirms current supported/clear pose and supervision of this physical test')
    parser.add_argument('--enable-torque', action='store_true')
    parser.add_argument('--exercise', choices=tuple(EXERCISES), default='command-expiry')
    parser.add_argument('--hold-seconds', type=float, default=1.)
    parser.add_argument('--duration', type=float, default=8.)
    parser.add_argument('--out', type=Path, default=Path('outputs/commission-hold'))
    run(parser.parse_args())
