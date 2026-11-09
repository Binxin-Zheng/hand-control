"""Supervisor exercises a fake stdlib subprocess; no SDK or device access."""
import importlib.util
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


FAKE_WORKER = r'''
import json, os, select, sys, time
case = sys.argv[1]
reason = 'release_required'
started = finished = False
deadline = None
silenced = False
buffer = b''
next_feedback = 0.
while True:
    now = time.monotonic()
    if deadline is not None and now >= deadline:
        reason = 'command_expired'; deadline = None; finished = True
    if select.select([sys.stdin], [], [], .002)[0]:
        data = os.read(sys.stdin.fileno(), 65536)
        if not data:
            if not silenced:
                info['active_finished'] = True
                reason = 'commission_duration_elapsed' if case == 'exit_wrong_reason' else 'controller_eof'
                print(json.dumps(dict(schema='teleop.robot_state.v1', timestamp_s=time.monotonic(),
                      q=[.1,.2,.3,.4,.5], aperture=.6, reason=reason, commissioning=info)), flush=True)
            break
        buffer += data
        while b'\n' in buffer:
            line, buffer = buffer.split(b'\n', 1)
            command = json.loads(line)
            mode = command['mode']
            if mode == 'release':
                reason = 'released'; finished = started; deadline = None
            elif mode == 'estop':
                reason = 'emergency_stop'; finished = True; deadline = None
            elif mode == 'active':
                assert command['q'] == [.1, .2, .3, .4, .5]
                assert command['aperture'] == .6
                reason = 'active'; started = True; deadline = command['deadline_s']
    now = time.monotonic()
    if now >= next_feedback and not silenced:
        info = dict(mode='hold', anchor_q=[.1,.2,.3,.4,.5], anchor_aperture=.6,
                    anchor_raw=dict(zip(['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll'], range(2000,2005))),
                    active_started=started, active_finished=finished, finish_reason=reason if finished else None)
        if case == 'wrong_mode': info['mode'] = 'normal'
        print(json.dumps(dict(schema='teleop.robot_state.v1', timestamp_s=now,
                              q=[.1,.2,.3,.4,.5], aperture=.6, reason=reason,
                              commissioning=info)), flush=True)
        next_feedback = now+.01
        if case == 'silence' and started: silenced = True
sys.exit(7 if case == 'exit_error' else 0)
'''


@pytest.fixture
def helper():
    path = Path(__file__).parents[1]/'tools/commission_hold.py'
    spec = importlib.util.spec_from_file_location('hold_diagnostic_test', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def options(**changes):
    values = dict(config=Path('config/robot.local.json'), robot_python=Path(sys.executable),
                  execute=True, pose_confirmed=True, enable_torque=False,
                  exercise='command-expiry', hold_seconds=1., duration=8.)
    values.update(changes)
    return SimpleNamespace(**values)


@pytest.mark.parametrize('changes', [dict(execute=False), dict(pose_confirmed=False),
    dict(duration=float('nan')), dict(duration=31), dict(duration=2),
    dict(hold_seconds=float('inf')), dict(hold_seconds=0), dict(exercise='move')])
def test_explicit_prerequisites_and_duration_checked_before_worker_construction(helper, changes):
    with pytest.raises(ValueError): helper.worker_command(options(**changes))


def test_command_is_restricted_and_interpreter_resolves_from_caller_cwd(helper, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    command = helper.worker_command(options(robot_python=Path('sdk/bin/python')))
    assert command[0] == str(tmp_path/'sdk/bin/python')
    assert '--commission-hold' in command and '--pose-confirmed' in command
    assert '--enable-torque' not in command
    assert '--enable-torque' in helper.worker_command(options(enable_torque=True))


def setup_fake(tmp_path, case):
    script = tmp_path/'fake_worker.py'; script.write_text(FAKE_WORKER)
    session = tmp_path/'evidence'; session.mkdir()
    return [sys.executable, str(script), case], session


@pytest.mark.parametrize('exercise', ['command-expiry', 'software-stop', 'release', 'controller-exit'])
def test_one_bounded_sequence_records_feedback_and_never_changes_anchor(helper, tmp_path, exercise):
    command, session = setup_fake(tmp_path, 'normal')
    result = helper.supervise(command, session, exercise=exercise, hold_seconds=.12, duration=3.)
    assert result['outcome'] in ('protocol_transition_observed', 'worker_exited_after_controller_eof')
    assert result['worker_returncode'] == 0 and result['feedback_frames'] > 0
    assert not result['motion_stop_validated'] and not result['acceptance_changed']
    rows = [json.loads(line) for line in (session/'events.jsonl').read_text().splitlines()]
    packets = [row['packet'] for row in rows if row['kind'] == 'command']
    assert packets[0]['mode'] == 'release'
    assert sum(packet['mode'] == 'release' for packet in packets) == (2 if exercise == 'release' else 1)
    for packet in packets:
        if packet['mode'] == 'active':
            assert packet['q'] == [.1, .2, .3, .4, .5] and packet['aperture'] == .6
    assert [packet['sequence'] for packet in packets] == list(range(len(packets)))


def test_feedback_silence_stops_renewal_before_hold_duration(helper, tmp_path):
    command, session = setup_fake(tmp_path, 'silence')
    with pytest.raises(RuntimeError, match='Feedback stopped'):
        helper.supervise(command, session, exercise='command-expiry', hold_seconds=.5, duration=3.)
    summary = json.loads((session/'summary.json').read_text())
    assert summary['outcome'] == 'failed' and summary['worker_returncode'] == 0
    rows = [json.loads(line) for line in (session/'events.jsonl').read_text().splitlines()]
    active = [row for row in rows if row['kind'] == 'command' and row['packet']['mode'] == 'active']
    assert active[-1]['timestamp_s']-active[0]['timestamp_s'] < .2


def test_noncommissioning_worker_is_rejected_before_first_command(helper, tmp_path):
    command, session = setup_fake(tmp_path, 'wrong_mode')
    with pytest.raises(ValueError, match='restricted hold'):
        helper.supervise(command, session, exercise='command-expiry', hold_seconds=.12, duration=3.)
    rows = [json.loads(line) for line in (session/'events.jsonl').read_text().splitlines()]
    assert not any(row['kind'] == 'command' for row in rows)


def test_observed_transition_does_not_hide_failed_worker_cleanup(helper, tmp_path):
    command, session = setup_fake(tmp_path, 'exit_error')
    with pytest.raises(RuntimeError, match='did not exit normally'):
        helper.supervise(command, session, exercise='software-stop', hold_seconds=.12, duration=3.)
    summary = json.loads((session/'summary.json').read_text())
    assert summary['outcome'] == 'failed' and summary['worker_returncode'] == 7
    assert summary['observed_reason'] == 'emergency_stop'


def test_operator_x_requests_stop_without_starting_an_active_phase(helper, tmp_path):
    command, session = setup_fake(tmp_path, 'normal')
    read_fd, write_fd = os.pipe()
    os.write(write_fd, b'X\n'); os.close(write_fd)
    with os.fdopen(read_fd) as operator_input:
        result = helper.supervise(command, session, exercise='command-expiry', hold_seconds=.12,
                                  duration=3., operator_input=operator_input)
    assert result['outcome'] == 'operator_stopped' and result['operator_stop_requested']
    rows = [json.loads(line) for line in (session/'events.jsonl').read_text().splitlines()]
    assert [row['packet']['mode'] for row in rows if row['kind'] == 'command'] == ['estop']


def test_clean_exit_with_another_reason_does_not_pass_eof_exercise(helper, tmp_path):
    command, session = setup_fake(tmp_path, 'exit_wrong_reason')
    with pytest.raises(RuntimeError, match='without confirming controller EOF'):
        helper.supervise(command, session, exercise='controller-exit', hold_seconds=.12, duration=3.)
    summary = json.loads((session/'summary.json').read_text())
    assert summary['outcome'] == 'failed' and summary['worker_returncode'] == 0
