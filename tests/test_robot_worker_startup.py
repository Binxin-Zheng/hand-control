"""Firmware drift and SDK timer regressions, with no serial/device access."""
from copy import deepcopy
from dataclasses import dataclass
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest


@pytest.fixture
def worker():
    path = Path(__file__).parents[1] / 'tools/robot_worker.py'
    spec = importlib.util.spec_from_file_location('worker_startup_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def robot_config(worker):
    joints = {name: dict(id=i+1, zero_raw=2000, raw_per_unit=500,
                        lower=-1, upper=1, raw_min=1000, raw_max=3000)
              for i, name in enumerate(worker.policy.ARM_NAMES)}
    joints['gripper'] = dict(id=6, zero_raw=1000, raw_per_unit=1000,
                            lower=0, upper=1, raw_min=1000, raw_max=2000)
    expected = {name: dict(id=item['id'], drive_mode=0, homing_offset=100,
                          range_min=item['raw_min'], range_max=item['raw_max'])
                for name, item in joints.items()}
    return dict(port='/dev/must_never_be_opened', joints=joints,
                expected_servo_calibration=expected,
                gripper_model_calibration=dict(zero_raw=1000, raw_per_rad=1000, angle_range_rad=[0, 1]))


class PacketTimer:
    """The installed LeRobot overwrite behavior, independent of its dependencies."""
    packet_timeout = 0
    tx_time_per_byte = .01

    def getCurrentTime(self):
        return 0.

    def setPacketTimeout(self, length):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = self.tx_time_per_byte * (length+3) + 50

    def setPacketTimeoutMillis(self, milliseconds):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = milliseconds


def test_packet_timeout_survives_sdk_overwrites_and_uses_monotonic_time(worker, monkeypatch):
    clock = [30.]
    monkeypatch.setattr(worker.time, 'monotonic', lambda: clock[0])
    port = PacketTimer()
    port.setPacketTimeoutMillis(20)
    port.setPacketTimeout(48)  # Six two-byte sync-read replies undo the old fix.
    assert port.packet_timeout > 50
    worker.configure_packet_timeout(port)
    for action in (lambda: port.setPacketTimeoutMillis(1000),
                   lambda: port.setPacketTimeout(48), lambda: port.setPacketTimeout(6)):
        clock[0] += .1
        action()
        assert port.packet_timeout == 20
        assert port.packet_start_time == pytest.approx(clock[0]*1000)
    port.setPacketTimeoutMillis(5)
    assert port.packet_timeout == 5


@dataclass
class ServoCalibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int


def install_fake_bus(worker, monkeypatch, actual, torque=1):
    events = []

    class Bus:
        def __init__(self, **kwargs):
            self.port_handler = PacketTimer()
            self.port_handler.ser = SimpleNamespace(timeout=None, write_timeout=None, fileno=lambda: 99)
            self.is_connected = False

        def connect(self):
            # Exercise the SDK's connection-time overwrite after the wrapper installs.
            self.port_handler.setPacketTimeoutMillis(1000)
            self.port_handler.setPacketTimeout(48)
            assert self.port_handler.packet_timeout == 20
            self.is_connected = True
            events.append('connect')

        def read(self, register, name, **kwargs):
            assert kwargs['normalize'] is False
            return 0 if register == 'Operating_Mode' else torque

        def read_calibration(self):
            events.append('read_calibration')
            return {name: ServoCalibration(**item) for name, item in actual.items()}

        def sync_read(self, register, motors=None, **kwargs):
            assert register == 'Present_Position' and kwargs['normalize'] is False
            names = actual if motors is None else motors
            return {name: 1500 if name == 'gripper' else 2000 for name in names}

        def sync_write(self, register, values, **kwargs):
            assert kwargs['normalize'] is False
            events.append(('write', register, values))

        def enable_torque(self):
            events.append('enable_torque')

        def disconnect(self, disable_torque):
            assert disable_torque is False
            self.is_connected = False
            events.append('disconnect')

    motors = ModuleType('lerobot.motors')
    motors.Motor = lambda *args: args
    motors.MotorNormMode = SimpleNamespace(DEGREES='degrees')
    feetech = ModuleType('lerobot.motors.feetech')
    feetech.FeetechMotorsBus = Bus
    monkeypatch.setitem(sys.modules, 'lerobot', ModuleType('lerobot'))
    monkeypatch.setitem(sys.modules, 'lerobot.motors', motors)
    monkeypatch.setitem(sys.modules, 'lerobot.motors.feetech', feetech)
    monkeypatch.setattr(worker, 'open', lambda *args: SimpleNamespace(close=lambda: None), raising=False)
    monkeypatch.setattr(worker.fcntl, 'flock', lambda *args: None)
    monkeypatch.setattr(worker.fcntl, 'ioctl', lambda *args: None)
    return events


@pytest.mark.parametrize('field', ['homing_offset', 'range_min', 'range_max', 'id', 'drive_mode'])
def test_changed_firmware_blocks_every_motion_write(worker, monkeypatch, field):
    config = robot_config(worker)
    actual = deepcopy(config['expected_servo_calibration'])
    actual['elbow_flex'][field] += 1
    events = install_fake_bus(worker, monkeypatch, actual, torque=0)
    port = worker.RawFeetechPort(config)
    try:
        with pytest.raises(RuntimeError, match='Servo calibration changed.*elbow_flex'):
            port.open_and_arm(enable_torque=True)
        assert events == ['connect', 'read_calibration']
        assert not port.safe_to_hold  # Cleanup must not write an untrusted mapped hold.
    finally:
        port.close()


def test_matching_firmware_is_checked_before_startup_hold_and_explicit_torque(worker, monkeypatch):
    config = robot_config(worker)
    events = install_fake_bus(worker, monkeypatch, config['expected_servo_calibration'], torque=0)
    port = worker.RawFeetechPort(config)
    try:
        port.open_and_arm(enable_torque=True)
        assert events[:2] == ['connect', 'read_calibration']
        assert events[2] == ('write', 'Goal_Position', dict.fromkeys(worker.policy.ARM_NAMES, 2000))
        assert events[3] == ('write', 'Goal_Position', {'gripper': 1500})
        assert events[4] == 'enable_torque'
        assert port.bus.port_handler.ser.timeout == 0
        assert port.bus.port_handler.ser.write_timeout == .02
    finally:
        port.close()


def test_torque_off_still_requires_explicit_flag_before_any_goal_write(worker, monkeypatch):
    config = robot_config(worker)
    events = install_fake_bus(worker, monkeypatch, config['expected_servo_calibration'], torque=0)
    port = worker.RawFeetechPort(config)
    try:
        with pytest.raises(RuntimeError, match='explicit --enable-torque'):
            port.open_and_arm()
        assert events == ['connect', 'read_calibration']
        assert not port.safe_to_hold
    finally:
        port.close()


def test_startup_preserves_existing_gripper_goal_when_torque_is_on(worker, monkeypatch):
    config = robot_config(worker)
    events = install_fake_bus(worker, monkeypatch, config['expected_servo_calibration'], torque=1)
    port = worker.RawFeetechPort(config)
    try:
        port.open_and_arm()
        assert events == ['connect', 'read_calibration',
                          ('write', 'Goal_Position', dict.fromkeys(worker.policy.ARM_NAMES, 2000))]
    finally:
        port.close()


def test_missing_firmware_pin_rejects_before_sdk_import_or_connection(worker):
    config = robot_config(worker)
    config.pop('expected_servo_calibration')
    port = worker.RawFeetechPort(config)
    with pytest.raises(ValueError, match='Pin expected_servo_calibration'):
        port.open_and_arm()
    assert port.bus is None and port.lock is None


@pytest.mark.parametrize('field,value', [('range_min', 999), ('id', 12), ('homing_offset', 2048), ('drive_mode', 1)])
def test_firmware_pin_must_match_accepted_numeric_joint_mapping(worker, field, value):
    config = robot_config(worker)
    config['expected_servo_calibration']['shoulder_pan'][field] = value
    with pytest.raises(ValueError, match='does not match joint mapping'):
        worker.expected_servo_calibration(config)


def test_actual_installed_sdk_sync_read_timer_without_serial(worker, monkeypatch):
    """Run in the isolated SDK environment too; constructing handlers opens nothing."""
    scs = pytest.importorskip('scservo_sdk')
    feetech = pytest.importorskip('lerobot.motors.feetech.feetech')
    handler = scs.PortHandler('/dev/must_never_be_opened')
    handler.tx_time_per_byte = .01
    handler.setPacketTimeout = feetech.patch_setPacketTimeout.__get__(handler, scs.PortHandler)
    protocol = scs.PacketHandler(0)
    monkeypatch.setattr(protocol, 'txPacket', lambda port, packet: scs.COMM_SUCCESS)
    protocol.syncReadTx(handler, 56, 2, [1, 2, 3, 4, 5, 6], 6)
    assert handler.packet_timeout > 50
    clock = [100.]
    monkeypatch.setattr(worker.time, 'monotonic', lambda: clock[0])
    worker.configure_packet_timeout(handler)
    protocol.syncReadTx(handler, 56, 2, [1, 2, 3, 4, 5, 6], 6)
    assert handler.packet_timeout == 20
    clock[0] += .019
    assert not handler.isPacketTimeout()
    clock[0] += .002
    assert handler.isPacketTimeout()
    assert not handler.is_open and handler.ser is None
