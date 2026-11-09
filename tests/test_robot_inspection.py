"""Read-only inspector must never invoke register writes or torque-off cleanup."""
from dataclasses import make_dataclass
import importlib.util
import json
from pathlib import Path
import sys
import types

import pytest


def test_inspection_reads_and_disconnects_without_torque_changes(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('inspection', Path(__file__).parents[1]/'tools/inspect_robot.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    fields = ['id', 'drive_mode', 'homing_offset', 'range_min', 'range_max']
    record = make_dataclass('Record', fields)
    names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
    expected = {n: dict(zip(fields, [i+1, 0, 100, 500, 3500])) for i, n in enumerate(names)}
    path = tmp_path/'calibration.json'; path.write_text(json.dumps(expected))
    events = []
    class FakeBus:
        def __init__(self, **kwargs):
            self.is_connected = False
            self.port_handler = types.SimpleNamespace(ser=types.SimpleNamespace(fileno=lambda: 0))
        def connect(self):
            # Exercise the inspector's register-write guards, not a fake pass.
            for method in (self.write, self.sync_write, self._write):
                with pytest.raises(RuntimeError, match='forbidden'): method('Torque_Enable', 1)
            self.is_connected = True
        def set_timeout(self, **kwargs): pass
        def read_calibration(self): return {n: record(**v) for n, v in expected.items()}
        def sync_read(self, register, **kwargs):
            assert register == 'Present_Position' and kwargs['normalize'] is False
            return dict.fromkeys(names, 2000)
        def read(self, register, name, **kwargs):
            assert register in ('Operating_Mode', 'Torque_Enable')
            assert kwargs['normalize'] is False
            return 0
        def disconnect(self, disable_torque=True):
            assert disable_torque is False
            events.append('closed_without_torque_change')
    motors = types.ModuleType('lerobot.motors')
    motors.Motor = lambda *args: args
    motors.MotorNormMode = types.SimpleNamespace(DEGREES='degrees')
    feetech = types.ModuleType('lerobot.motors.feetech'); feetech.FeetechMotorsBus = FakeBus
    monkeypatch.setitem(sys.modules, 'lerobot', types.ModuleType('lerobot'))
    monkeypatch.setitem(sys.modules, 'lerobot.motors', motors)
    monkeypatch.setitem(sys.modules, 'lerobot.motors.feetech', feetech)
    monkeypatch.setattr(module.fcntl, 'ioctl', lambda *args: None)
    result = module.inspect(str(tmp_path/'not_a_serial_device'), path)
    assert result['calibration_matches'] is True
    assert result['mode'] == dict.fromkeys(names, 0)
    assert events == ['closed_without_torque_change']
