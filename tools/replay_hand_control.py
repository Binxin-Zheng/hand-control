"""Offline observed-hand replay through the real session/IK/collision guard.

The plant is ideal kinematic execution, not hardware or a dynamics validation.
Every timestamped input is consumed. Untimed capture failures are counted
separately; no capture timestamp is invented. Engagement events must be supplied
explicitly; loss never generates an automatic release/re-engage event.
"""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import numpy as np

from anyteleop_so101.kinematics import URDFChain
from anyteleop_so101.mapping import RelativeMapper
from anyteleop_so101.protocol import HandObservation
from anyteleop_so101.servo import PositionPriorityServo
from anyteleop_so101.session import TeleopSession, RobotState
from anyteleop_so101.runtime.bullet import BulletWorld
from anyteleop_so101.runtime.control import GuardedController


def run(args):
    rows = [json.loads(line) for line in args.recording.read_text().splitlines()]
    seen_timestamp = False
    for row in rows:
        if row.get('timestamp_s') is not None:
            seen_timestamp = True
        elif seen_timestamp:
            raise ValueError('An untimed failure inside the running stream cannot be silently omitted from replay')
    timed = [row for row in rows if row.get('timestamp_s') is not None]
    if not timed:
        raise ValueError('Recording has no timestamped observations')
    times = np.array([row['timestamp_s'] for row in timed])
    if np.any(np.diff(times) <= 0):
        raise ValueError('Capture times must increase')
    events = json.loads(args.events.read_text())
    if any(set(event) != {'at_s', 'command'} or not np.isfinite(event['at_s']) or event['at_s'] < 0
           or event['command'] not in ('release', 'engage', 'estop', 'reset') for event in events):
        raise ValueError('Explicit events require nonnegative at_s and supported command')
    if any(b['at_s'] < a['at_s'] for a, b in zip(events, events[1:])):
        raise ValueError('Events must be ordered')
    config = json.loads(args.config.read_text())
    chain = URDFChain(config['urdf'], 'base_link', 'gripper_link')
    session = TeleopSession(PositionPriorityServo(chain),
        RelativeMapper(config['R_base_camera'], config['translation_scale']), control_mode=args.control_mode)
    q = np.asarray(config['sim_initial_q_rad'], dtype=float)
    aperture = float(config['sim_initial_aperture'])
    origin = float(times[0])
    # Replay virtual clock is unrelated to the running host's motor/camera clock.
    tick_times = np.unique(np.r_[np.arange(0, times[-1]-origin+.02, .02), times-origin,
                                [event['at_s'] for event in events if event['at_s'] <= times[-1]-origin]])
    row_index = event_index = 0
    observation = None
    clutch = estop = False
    counts = Counter()
    consumed = 0
    previous = -.02
    first_faults = []
    args.out.parent.mkdir(parents=True, exist_ok=True)
    guard = BulletWorld(config['urdf'], **config.get('collision', {}))
    controller = GuardedController(session, guard)
    try:
        with args.out.open('x') as output:
            for seconds in tick_times:
                now = float(seconds+1.)
                reset = False
                while row_index < len(timed) and times[row_index]-origin <= seconds+1e-9:
                    payload = timed[row_index]['methods'][args.method]['observation']
                    observation = replace(HandObservation.from_dict(payload), timestamp_s=times[row_index]-origin+1.)
                    row_index += 1; consumed += 1
                while event_index < len(events) and events[event_index]['at_s'] <= seconds+1e-9:
                    command = events[event_index]['command']
                    if command == 'engage': clutch = True
                    if command == 'release': clutch = False
                    if command == 'estop': estop = True
                    if command == 'reset' and not clutch: reset = True; estop = False
                    event_index += 1
                dt = float(seconds-previous); previous = seconds
                if dt < 1e-8:
                    continue
                measured = RobotState(now, q.copy(), aperture)
                decision = controller.tick(observation, measured, now_s=now, dt=dt,
                    clutch=clutch, estop=estop, reset_estop=reset)
                counts[decision.reason] += 1
                if decision.state == 'PAUSED' and decision.reason != 'release_then_engage_required':
                    first_faults.append({'at_s': float(seconds), 'reason': decision.reason})
                actual = chain.fk(q)
                output.write(json.dumps(dict(at_s=float(seconds), state=decision.state, reason=decision.reason,
                    input_sequence=None if observation is None else observation.sequence,
                    target=None if decision.desired_pose is None else decision.desired_pose.as_dict(),
                    measured_pose=actual.as_dict(), measured_q=q.tolist(), measured_aperture=aperture,
                    q_command=None if decision.q_command is None else decision.q_command.tolist(),
                    aperture_command=decision.aperture_command,
                    diagnostics=decision.diagnostics), allow_nan=False)+'\n')
                if decision.q_command is not None:
                    q = decision.q_command.copy(); aperture = decision.aperture_command
    finally:
        guard.close()
    summary = dict(input_attempts=len(rows), timed_inputs=len(timed), timed_inputs_consumed=consumed,
        untimed_capture_failures=len(rows)-len(timed), duration_s=float(times[-1]-origin),
        method=args.method, control_mode=args.control_mode, decisions=dict(counts), first_faults=first_faults,
        explicit_events=events, input_sha256=hashlib.sha256(args.recording.read_bytes()).hexdigest(),
        note='Official collision model and real supervisor/IK; ideal kinematic plant. No devices. Not live accuracy/reliability.')
    args.out.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path, help='Complete analyze_rgbd_recording JSONL output')
    parser.add_argument('--method', default='wrist_surface_fit')
    parser.add_argument('--control-mode', choices=['position', 'full_pose'], default='position')
    parser.add_argument('--config', type=Path, default=Path('config/runtime.sim.json'))
    parser.add_argument('--events', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    run(parser.parse_args())
