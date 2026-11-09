"""Timing diagnostics preserve collision rejection and exception behavior."""
import importlib.util
from pathlib import Path

import pytest


@pytest.fixture
def runtime_entry():
    path = Path(__file__).parents[1]/'tools/run_teleop.py'
    spec = importlib.util.spec_from_file_location('control_diagnostics_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_overlay_refreshes_at_ten_hz_and_immediately_on_state_or_reason(runtime_entry):
    due = runtime_entry.overlay_refresh_due
    assert due('IDLE: released', None, 0., 0.)
    assert not due('IDLE: released', 'IDLE: released', .02, .1)
    assert not due('IDLE: released', 'IDLE: released', .099, .1)
    assert due('IDLE: released', 'IDLE: released', .1, .1)
    assert due('ACTIVE: tracking', 'IDLE: released', .02, .1)
    assert due('PAUSED: unavailable', 'PAUSED: timing', .02, .1)


def test_timed_guard_preserves_fail_closed_behavior_and_error(monkeypatch):
    path = Path(__file__).parents[1]/'tools/run_teleop.py'
    spec = importlib.util.spec_from_file_location('control_diagnostics_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    samples = iter([1., 1.012, 2., 2.023])
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(samples))

    class Guard:
        last_collision = {'reason': 'collision_clearance'}
        def check_segment(self, *args, **kwargs):
            if args == ('raise',):
                raise RuntimeError('collision engine failed')
            assert args == ('original arguments',)
            assert kwargs == {'original': True}
            return False

    guard = module.TimedCollisionGuard(Guard())
    assert guard.check_segment('original arguments', original=True) is False
    assert guard.last_collision == {'reason': 'collision_clearance'}
    assert guard.duration_s == pytest.approx(.012)
    with pytest.raises(RuntimeError, match='collision engine failed'):
        guard.check_segment('raise')
    assert guard.duration_s == pytest.approx(.035)
