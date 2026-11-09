"""Diagnostic recording must not delay delivery of an already observed frame."""
import importlib.util
import io
import json
from pathlib import Path
import sys

import pytest

from anyteleop_so101.runtime.transport import FileObservationReceiver
from conftest import obs


def load_worker():
    path = Path(__file__).parents[1]/'tools/vision_rgbd.py'
    spec = importlib.util.spec_from_file_location('vision_delivery_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_slow_recording_receives_already_published_fresh_observation(tmp_path, monkeypatch):
    worker = load_worker()
    clock = [100.05]
    monkeypatch.setattr(worker.time, 'monotonic', lambda: clock[0])
    packet = {'schema': 'teleop.vision.v1', 'observation': obs(100., 7).as_dict()}
    destination = tmp_path/'latest.json'
    receiver = FileObservationReceiver(destination)

    class SlowLog(io.StringIO):
        def write(self, value):
            assert receiver.poll().sequence == 7
            clock[0] += .02
            return super().write(value)

    log = SlowLog()
    def slow_archive():
        assert receiver.poll().sequence == 7
        assert json.loads(log.getvalue()) == packet
        clock[0] += .2

    timings = worker.publish_then_record(packet, destination, log, record_frame=slow_archive)
    assert timings['frame_log_s'] == pytest.approx(.02)
    assert timings['image_record_s'] == pytest.approx(.2)
    # Freshness remains based on capture time, never the completion of recording.
    assert receiver.poll() is None
    assert receiver.error == 'Stale/future vision input'


def test_failed_publication_still_preserves_acquired_attempt(tmp_path, monkeypatch):
    worker = load_worker()
    import anyteleop_so101.runtime.transport as transport
    def unavailable(*args):
        raise OSError('disk unavailable')
    monkeypatch.setattr(transport, 'publish_file', unavailable)
    packet = {'schema': 'teleop.vision.v1', 'observation': obs(100., 7).as_dict()}
    log = io.StringIO()
    archived = []
    with pytest.raises(OSError, match='disk unavailable'):
        worker.publish_then_record(packet, tmp_path/'latest.json', log,
                                   record_frame=lambda: archived.append(7))
    assert json.loads(log.getvalue()) == packet
    assert archived == [7]


def test_disconnected_pipeline_cleanup_preserves_original_error_and_closes_rest(capsys):
    worker = load_worker()
    closed = []
    def stopped_pipeline():
        raise RuntimeError('stop() cannot be called before start()')
    with pytest.raises(RuntimeError, match='Device disconnected'):
        try:
            raise RuntimeError('Device disconnected')
        finally:
            worker.close_resources([('camera', stopped_pipeline),
                                    ('detector', lambda: closed.append('detector')),
                                    ('preview', lambda: closed.append('preview'))],
                                   original_error=sys.exc_info()[1])
    assert closed == ['detector', 'preview']
    assert 'Cleanup warning (camera)' in capsys.readouterr().err


def test_cleanup_failure_on_normal_exit_is_not_reported_as_success():
    worker = load_worker()
    closed = []
    def failed(): raise RuntimeError('stop failed')
    with pytest.raises(RuntimeError, match='Vision cleanup failed: camera'):
        worker.close_resources([('camera', failed), ('detector', lambda: closed.append(True))])
    assert closed == [True]
