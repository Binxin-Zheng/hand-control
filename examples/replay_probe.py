"""Replay every real-video frame through the core, with no fake depth or motor I/O.

This is a safety replay, NOT a moving-arm simulation. RGB-only input must pause.
Clutch is held throughout to test that hand reappearance cannot resume control.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from anyteleop_so101.observation_adapter import adapt_probe
from anyteleop_so101.kinematics import URDFChain
from anyteleop_so101.mapping import RelativeMapper
from anyteleop_so101.servo import PositionPriorityServo
from anyteleop_so101.session import RobotState, TeleopSession


def run(source, output):
    chain = URDFChain(Path(__file__).resolve().parents[1] / 'assets/so101_kinematics_only.urdf',
                      'base_link', 'gripper_link')
    session = TeleopSession(PositionPriorityServo(chain), RelativeMapper(np.eye(3)))
    q = np.array([.1, -.3, .4, .15, .2])
    counts = Counter(); ticks = 0; frames = 0; t = 0.; previous = -1.
    current = None
    output.parent.mkdir(parents=True, exist_ok=True)
    with source.open() as source_file, output.open('w') as target:
        for line in source_file:
            row = json.loads(line)
            pts = row['timestamp_s']
            if row['timestamp_semantics'] != 'video presentation time' or pts <= previous:
                raise ValueError('Expected strictly ordered video presentation timestamps')
            previous = pts
            while t < pts - 1e-9:
                # Controller runs independently at 100 Hz; prior observations can age out.
                decision = session.tick(current, RobotState(1+t, q, .5),
                                        now_s=1+t, dt=.01, clutch=True)
                assert decision.q_command is None
                ticks += 1; t = round(t + .01, 8)
            current, diagnostics = adapt_probe(row, timestamp_s=1+pts, track_id='right:0')
            decision = session.tick(current, RobotState(1+t, q, .5), now_s=1+t, dt=.01, clutch=True)
            assert decision.q_command is None
            counts[diagnostics['reason']] += 1; frames += 1; ticks += 1
            target.write(json.dumps({'sequence': row['sequence'], 'video_time_s': pts,
                                    'adapter': diagnostics, 'state': decision.state,
                                    'reason': decision.reason, 'q_command': None}) + '\n')
            t = round(t + .01, 8)
    summary = dict(frames=frames, controller_ticks=ticks, adapter_reasons=dict(counts),
                   motion_commands=0, kind='real RGB video safety replay; NOT arm simulation')
    output.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path); p.add_argument('--out', type=Path, required=True)
    a = p.parse_args(); run(a.source, a.out)
