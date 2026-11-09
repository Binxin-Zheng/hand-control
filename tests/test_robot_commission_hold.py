"""Bounded hold commissioning through the real worker, with fake ports only."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import select
import subprocess
import sys
import time

import pytest

from test_robot_worker_startup import worker, robot_config, install_fake_bus


@pytest.mark.parametrize('duration', [None, 0, -1, 30.01, float('nan'), float('inf'), True])
def test_hold_duration_is_explicit_and_bounded_before_port_construction(worker, monkeypatch, duration):
    monkeypatch.setattr(worker, 'RawFeetechPort', lambda _: pytest.fail('Port constructed'))
    with pytest.raises(ValueError, match='duration'):
        worker.run(robot_config(worker), False, commission_hold=True, duration_s=duration, pose_confirmed=True)


def test_hold_has_explicit_pose_numeric_prerequisites_but_does_not_edit_acceptance(worker):
    config = robot_config(worker)
    config['acceptance'] = {'software_stop': True}
    before = deepcopy(config)
    with pytest.raises(ValueError, match='pose-confirmed'):
        worker.validate_commission_hold(config, 1., False)
    worker.validate_commission_hold(config, 1., True)
    assert config == before
    with pytest.raises(ValueError, match='Robot setup incomplete'):
        worker.validate_acceptance(config)
    config['expected_servo_calibration']['elbow_flex']['range_max'] += 1
    with pytest.raises(ValueError, match='does not match'):
        worker.validate_commission_hold(config, 1., True)


def test_actual_startup_hold_raw_is_anchor_and_active_writes_never_touch_gripper(worker, monkeypatch):
    config = robot_config(worker)
    events = install_fake_bus(worker, monkeypatch, config['expected_servo_calibration'], torque=1)
    bus_type = sys.modules['lerobot.motors.feetech'].FeetechMotorsBus
    original = bus_type.sync_read
    def moved_since_initial_read(self, register, motors=None, **kwargs):
        values = original(self, register, motors, **kwargs)
        if motors is not None:
            values = dict.fromkeys(worker.policy.ARM_NAMES, 2005)
        return values
    monkeypatch.setattr(bus_type, 'sync_read', moved_since_initial_read)
    port = worker.RawFeetechPort(config)
    try:
        port.open_and_arm(deadline_s=time.monotonic()+2)
        assert port.startup_arm_raw == dict.fromkeys(worker.policy.ARM_NAMES, 2005)
        restriction = worker.CommissionHold(port.calibration, port.startup_arm_raw, port.startup_aperture, 0., 1.)
        assert restriction.anchor_q == [.01]*5 and restriction.anchor_aperture == .5
        restriction.accept_active(restriction.anchor_q)
        port.send_fixed_arm_anchor(restriction.anchor_raw)
        with pytest.raises(ValueError, match='exact startup'):
            port.send_fixed_arm_anchor(dict.fromkeys(worker.policy.ARM_NAMES, 2006))
        port.stop()
        assert [event[2] for event in events if isinstance(event, tuple)] == [port.startup_arm_raw]*3
    finally:
        port.close()
    assert events[-1] == 'disconnect'


@pytest.mark.parametrize('bad', ['missing', 'extra', 'nan', 'fractional', 'bool', 'range'])
def test_stop_never_writes_partial_or_invalid_current_raw_feedback(worker, monkeypatch, bad):
    config = robot_config(worker)
    events = install_fake_bus(worker, monkeypatch, config['expected_servo_calibration'])
    port = worker.RawFeetechPort(config)
    port.open_and_arm()
    raw = dict(port.startup_arm_raw)
    if bad == 'missing': raw.pop('wrist_roll')
    elif bad == 'extra': raw['gripper'] = 1500
    else: raw['wrist_roll'] = {'nan': float('nan'), 'fractional': 2000.1, 'bool': True, 'range': 4096}[bad]
    monkeypatch.setattr(port.bus, 'sync_read', lambda *a, **kw: raw)
    writes_before = list(events)
    try:
        with pytest.raises(ValueError): port.stop()
        assert events == writes_before
    finally:
        port.close()


@pytest.mark.parametrize('delay_phase', ['torque_read', 'startup_hold'])
def test_commission_startup_refuses_stale_goal_and_never_enables_torque(worker, monkeypatch, delay_phase):
    config = robot_config(worker)
    events = install_fake_bus(worker, monkeypatch, config['expected_servo_calibration'], torque=0)
    clock = [1.]
    monkeypatch.setattr(worker.time, 'monotonic', lambda: clock[0])
    bus_type = sys.modules['lerobot.motors.feetech'].FeetechMotorsBus
    original_read, original_sync = bus_type.read, bus_type.sync_read
    def read(self, register, *a, **kw):
        if register == 'Torque_Enable': clock[0] += .02 if delay_phase == 'torque_read' else .005
        return original_read(self, register, *a, **kw)
    def sync(self, register, motors=None, **kw):
        if motors is not None and delay_phase == 'startup_hold': clock[0] += .08
        return original_sync(self, register, motors, **kw)
    monkeypatch.setattr(bus_type, 'read', read)
    monkeypatch.setattr(bus_type, 'sync_read', sync)
    port = worker.RawFeetechPort(config)
    try:
        with pytest.raises(ValueError, match='stale, future, or read'):
            port.open_and_arm(True, deadline_s=5.)
        assert 'enable_torque' not in events
        writes = [event[2] for event in events if isinstance(event, tuple)]
        assert all(set(values) == set(worker.policy.ARM_NAMES) for values in writes)
        assert len(writes) == (1 if delay_phase == 'startup_hold' else 0)
    finally:
        port.close()


def test_commission_stop_refuses_slow_current_read_even_with_valid_raw(worker, monkeypatch):
    config = robot_config(worker)
    events = install_fake_bus(worker, monkeypatch, config['expected_servo_calibration'])
    port = worker.RawFeetechPort(config)
    port.open_and_arm(deadline_s=time.monotonic()+2)
    clock = [1.]
    monkeypatch.setattr(worker.time, 'monotonic', lambda: clock[0])
    def slow_read(*a, **kw):
        clock[0] += .101
        return dict(port.startup_arm_raw)
    monkeypatch.setattr(port.bus, 'sync_read', slow_read)
    before = list(events)
    try:
        with pytest.raises(ValueError, match='read exceeded'): port.stop()
        assert events == before
    finally:
        port.close()


@pytest.mark.parametrize('stamp,start', [(8., 9.99), (float('nan'), 9.99), (10.006, 9.99), (10., 9.89)])
def test_feedback_freshness_includes_read_start_age(worker, stamp, start):
    with pytest.raises(ValueError, match='stale, future, or read'):
        worker.validate_commission_feedback(dict(timestamp_s=stamp, q=[0.]*5, aperture=.5), 10., start, .1)


def test_fixed_anchor_policy_cannot_restart_after_phase_finishes(worker):
    calibration = worker.policy.Calibration(robot_config(worker)['joints'])
    hold = worker.CommissionHold(calibration, dict.fromkeys(worker.policy.ARM_NAMES, 2000), .5, 0., 2.)
    hold.finish('released')  # Initial release is required and does not end the unused phase.
    assert not hold.active_finished
    with pytest.raises(ValueError, match='equal the startup'):
        hold.accept_active([.001]*5)
    hold.accept_active([0.]*5)
    hold.finish('command_expired')
    hold.finish('released')
    assert hold.feedback(1.)['finish_reason'] == 'command_expired'
    with pytest.raises(ValueError, match='active phase ended'):
        hold.accept_active([0.]*5)


FAKE_PORT_RUNNER = r'''
import importlib.util,json,sys,time
spec=importlib.util.spec_from_file_location('worker',sys.argv[1])
w=importlib.util.module_from_spec(spec);spec.loader.exec_module(w)
config=json.loads(sys.argv[3]); case=sys.argv[4]
class FakePort:
    def __init__(self,config):
        self.calibration=w.policy.Calibration(config['joints'])
        self.bus=self;self.is_connected=True;self.safe_to_hold=True;self.sent=False
        self.startup_arm_raw=dict.fromkeys(w.policy.ARM_NAMES,2000)
        self.startup_aperture=.5
    def event(self,name,value=None):
        with open(sys.argv[2],'a') as f:f.write(json.dumps([name,time.monotonic(),value])+'\n')
    def open_and_arm(self,enable,*,deadline_s=None):
        assert deadline_s is not None
        self.event('open',enable)
    def read(self):
        if case=='slow' and self.sent:time.sleep(.11)
        stamp=time.monotonic()-(1 if case=='stale' and self.sent else 0)
        return dict(timestamp_s=stamp,q=[0.]*5,aperture=.5)
    def send(self,*args):raise AssertionError('Commissioning used unrestricted send')
    def send_fixed_arm_anchor(self,anchor):
        assert anchor==self.startup_arm_raw
        self.sent=True;self.event('send_fixed',anchor)
    def stop(self):self.event('stop');return dict(self.startup_arm_raw)
    def close(self):self.event('close')
w.RawFeetechPort=FakePort
w.run(config,False,commission_hold=True,duration_s=float(sys.argv[5]),pose_confirmed=True)
'''


def fake_command(worker, tmp_path, case='normal', duration=2.):
    runner = tmp_path/'real_loop_fake_port.py'; runner.write_text(FAKE_PORT_RUNNER)
    events = tmp_path/'port-events.jsonl'
    return [sys.executable, str(runner), str(Path(worker.__file__)), str(events),
            json.dumps(robot_config(worker)), case, str(duration)], events


def receive(process, predicate, timeout=2.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if select.select([process.stdout], [], [], max(0., deadline-time.monotonic()))[0]:
            line = process.stdout.readline()
            assert line, process.stderr.read()
            row = json.loads(line)
            if predicate(row): return row
    pytest.fail('Worker feedback deadline exceeded')


def send(process, seq, mode, q=None):
    process.stdin.write(json.dumps(dict(sequence=seq, deadline_s=time.monotonic()+.08,
                                      mode=mode, q=[0.]*5 if q is None else q, aperture=.5))+'\n')
    process.stdin.flush()


@pytest.mark.parametrize('transition', ['release', 'stop', 'estop', 'expiry', 'bad_target', 'malformed', 'backlog'])
def test_real_loop_terminal_phase_cannot_return_to_old_anchor(worker, tmp_path, transition):
    command, events = fake_command(worker, tmp_path)
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        receive(process, lambda row: row['reason']=='release_required')
        send(process, 0, 'release'); receive(process, lambda row: row['reason']=='released')
        send(process, 1, 'active'); receive(process, lambda row: row['reason']=='active')
        if transition in ('release', 'stop', 'estop'): send(process, 2, transition)
        elif transition == 'bad_target': send(process, 2, 'active', [.001]*5)
        elif transition == 'malformed': process.stdin.write('{bad\n'); process.stdin.flush()
        elif transition == 'backlog': process.stdin.write('{}\n{}\n'); process.stdin.flush()
        terminal = receive(process, lambda row: row['commissioning']['active_finished'])
        original_reason = terminal['commissioning']['finish_reason']
        if transition == 'expiry': assert original_reason == 'command_expired'
        if transition == 'bad_target': assert 'equal the startup' in original_reason
        if transition == 'backlog': assert original_reason == 'command_backlog'
        # Even explicit reset + release cannot reuse this invocation's old anchor.
        send(process, 3, 'reset'); receive(process, lambda row: row['reason']=='reset_requires_release')
        send(process, 4, 'release'); receive(process, lambda row: row['reason']=='released')
        send(process, 5, 'active')
        rejected = receive(process, lambda row: 'active phase ended' in row['reason'])
        assert rejected['commissioning']['finish_reason'] == original_reason
        process.stdin.close(); process.wait(timeout=2)
        assert process.returncode == 0, process.stderr.read()
        rows = [json.loads(line) for line in events.read_text().splitlines()]
        assert sum(row[0]=='send_fixed' for row in rows) == 1
        assert rows[-1][0] == 'close'
    finally:
        if process.poll() is None: process.kill(); process.wait()
        process.stdout.close(); process.stderr.close()


@pytest.mark.parametrize('case', ['stale', 'slow'])
def test_real_loop_bad_read_ends_phase_and_no_further_active_writes(worker, tmp_path, case):
    command, events = fake_command(worker, tmp_path, case)
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        receive(process, lambda row: row['reason']=='release_required')
        send(process, 0, 'release'); receive(process, lambda row: row['reason']=='released')
        send(process, 1, 'active')
        row = receive(process, lambda row: row['commissioning']['active_finished'])
        assert 'stale, future, or read' in row['commissioning']['finish_reason']
        process.wait(timeout=2)
        assert process.returncode == 0, process.stderr.read()
        assert sum(json.loads(line)[0]=='send_fixed' for line in events.read_text().splitlines()) == 1
    finally:
        if process.poll() is None: process.kill(); process.wait()
        process.stdin.close(); process.stdout.close(); process.stderr.close()


def test_real_loop_duration_is_bounded_without_controller_input(worker, tmp_path):
    command, events = fake_command(worker, tmp_path, duration=.08)
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        row = receive(process, lambda row: row['commissioning']['active_finished'])
        assert row['commissioning']['finish_reason']=='commission_duration_elapsed'
        process.wait(timeout=2)
        assert process.returncode == 0, process.stderr.read()
        assert not any(json.loads(line)[0]=='send_fixed' for line in events.read_text().splitlines())
    finally:
        if process.poll() is None: process.kill(); process.wait()
        process.stdin.close(); process.stdout.close(); process.stderr.close()


@pytest.mark.parametrize('exercise', ['command-expiry', 'software-stop', 'release', 'controller-exit'])
def test_supervisor_and_real_restricted_worker_loop_agree_on_protocol(worker, tmp_path, exercise):
    path = Path(worker.__file__).with_name('commission_hold.py')
    spec = importlib.util.spec_from_file_location('real_hold_supervisor_test', path)
    helper = importlib.util.module_from_spec(spec); spec.loader.exec_module(helper)
    command, events = fake_command(worker, tmp_path)
    session = tmp_path/'session'; session.mkdir()
    result = helper.supervise(command, session, exercise=exercise, hold_seconds=.12, duration=3.)
    assert result['worker_returncode'] == 0
    assert result['outcome'] in ('protocol_transition_observed', 'worker_exited_after_controller_eof')
    rows = [json.loads(line) for line in events.read_text().splitlines()]
    sent = [row[2] for row in rows if row[0]=='send_fixed']
    assert sent and all(raw==dict.fromkeys(worker.policy.ARM_NAMES, 2000) for raw in sent)
    assert rows[-1][0] == 'close'


def test_installed_sdk_encoder_byte_decode_is_integer_without_a_port():
    """The stricter hold validation accepts real SDK raw types; no I/O methods."""
    scs = pytest.importorskip('scservo_sdk')
    reader = scs.GroupSyncRead(None, None, 56, 2)
    reader.data_dict[1] = [0xd5, 0x07]
    raw = reader.getData(1, 56, 2)
    assert type(raw) is int and raw == 2005
    assert reader.port is None and reader.ph is None
