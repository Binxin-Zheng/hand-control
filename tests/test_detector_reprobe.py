"""Instrumentation transparency without importing MediaPipe or using devices."""
import importlib.util
import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


spec = importlib.util.spec_from_file_location(
    'detector_reprobe_test', Path(__file__).parents[1]/'tools/reprobe_recorded_detector.py')
reprobe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reprobe)


def test_process_tap_preserves_input_return_identity_and_single_call():
    image, result = object(), object()
    calls = []
    processor = SimpleNamespace(process=lambda item: (calls.append(item), result)[1], close=lambda: calls.append('closed'))
    tap = reprobe.ProcessTap(processor)
    assert tap.process(image) is result
    assert tap.last_result is result
    assert calls == [image] and tap.calls == 1
    tap.close()
    assert calls == [image, 'closed']


def test_process_tap_propagates_exception_without_reusing_previous_result():
    def fail(_):
        raise RuntimeError('original failure')
    tap = reprobe.ProcessTap(SimpleNamespace(process=fail))
    tap.last_result = object()
    with pytest.raises(RuntimeError, match='original failure'):
        tap.process(object())
    assert tap.calls == 1 and tap.last_result is None


@pytest.mark.parametrize('max_hands', [1, 2])
def test_only_explicit_two_candidate_variant_replaces_unprocessed_original_graph(max_hands):
    events = []
    original = SimpleNamespace(close=lambda: events.append('closed original'))
    replacement = object()
    detector = SimpleNamespace(hand_detector=original)
    def construct(**kwargs):
        events.append(('original constructor', kwargs))
        return detector
    def hands(**kwargs):
        events.append(('replacement graph', kwargs))
        return replacement
    module = SimpleNamespace(SingleHandDetector=construct,
        mp=SimpleNamespace(solutions=SimpleNamespace(hands=SimpleNamespace(Hands=hands))))
    assert reprobe.create_detector(module, 'Right', max_hands) is detector
    assert events[0] == ('original constructor', dict(hand_type='Right', selfie=False))
    if max_hands == 1:
        assert len(events) == 1 and detector.hand_detector is original
    else:
        assert events[1] == 'closed original'
        assert events[2] == ('replacement graph', dict(static_image_mode=False,
            max_num_hands=2, min_detection_confidence=.8, min_tracking_confidence=.8))
        assert detector.hand_detector is replacement


def raw_result(label='Right'):
    points = [SimpleNamespace(x=i/100, y=i/200, z=-i/300, visibility=0., presence=0.) for i in range(21)]
    return SimpleNamespace(multi_hand_landmarks=[SimpleNamespace(landmark=points)],
        multi_hand_world_landmarks=[SimpleNamespace(landmark=points)],
        multi_handedness=[SimpleNamespace(classification=[SimpleNamespace(index=0,label=label,score=.9)])])


def test_serialization_preserves_all_raw_keypoints_even_when_wrapper_filters_hand():
    result = raw_result('Right')
    raw = reprobe.serialize_raw_results(result)
    assert len(raw['image_landmarks'][0]) == len(raw['world_landmarks'][0]) == 21
    assert raw['image_landmarks'][0][20]['x'] == .2
    assert raw['handedness'][0][0] == dict(index=0,label='Right',score=.9)
    assert reprobe.selection_reason(raw, 'Left', 0) == 'handedness_filtered'
    assert len(result.multi_hand_landmarks[0].landmark) == 21
    assert reprobe.selection_reason(raw, 'Right', 1) == 'upstream_selected'
    assert reprobe.selection_reason(raw, 'Right', 0) == 'selection_inconsistent'


def test_no_raw_hand_is_distinct_from_wrong_handedness():
    raw = reprobe.serialize_raw_results(SimpleNamespace(multi_hand_landmarks=None,
        multi_hand_world_landmarks=None,multi_handedness=None))
    assert reprobe.selection_reason(raw, 'Left', 0) == 'mediapipe_no_hand'
    assert reprobe.selection_reason(raw, 'Left', 1) == 'selection_inconsistent'


@pytest.mark.parametrize('broken', [SimpleNamespace(), raw_result()])
def test_diagnostic_failure_is_serializable_and_does_not_hide_original_attempt(broken):
    if hasattr(broken, 'multi_hand_landmarks'):
        broken.multi_hand_landmarks[0].landmark[0].x = float('nan')
    captured = reprobe.capture_raw_after_error(broken)
    assert 'raw_capture_error' in captured
    assert 'raw_mediapipe' not in captured
    json.dumps(captured, allow_nan=False)


def test_probe_comparison_reports_any_selected_or_numerical_difference():
    identity = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
    saved = dict(detected=True,pixels=[[0.,0.]]*21,local_points_mano=[[0.,0.,0.]]*21,
                 model_wrist_rotation=identity)
    changed = dict(saved,pixels=[[.00001,0.]]*21)
    comparison = reprobe.compare_probe(saved, changed)
    assert comparison['selected_match'] and not comparison['exact_probe_match']
    assert comparison['pixels']['max_abs'] == .00001
    assert comparison['local_points_mano']['max_abs'] == 0
    assert not reprobe.compare_probe(saved,dict(detected=False))['selected_match']


def test_detector_pin_matches_recorded_upstream_source_without_importing_it():
    path = Path('/home/bx/repos/dex-retargeting/example/vector_retargeting/single_hand_detector.py')
    if not path.is_file():
        pytest.skip('Upstream checkout not present')
    assert reprobe.detector_blob(path.read_bytes()) == reprobe.DETECTOR_BLOB


@pytest.mark.parametrize('label', ['Left', 'Right', None])
def test_original_detect_returns_are_identical_with_tap_without_model_inference(label):
    """Execute the unchanged class bodies with a fake inference result only."""
    import numpy as np
    path = Path('/home/bx/repos/dex-retargeting/example/vector_retargeting/single_hand_detector.py')
    if not path.is_file():
        pytest.skip('Upstream checkout not present')
    tree = ast.parse(path.read_text())
    # Omitting import statements and postponing type annotations keeps this pure
    # test independent of MediaPipe; method/constant ASTs remain unchanged.
    tree.body = ast.parse('from __future__ import annotations').body + [
        node for node in tree.body if not isinstance(node, (ast.Import, ast.ImportFrom))]
    namespace = {'np': np}
    exec(compile(ast.fix_missing_locations(tree), str(path), 'exec'), namespace)
    cls = namespace['SingleHandDetector']
    detector = cls.__new__(cls)
    detector.operator2mano = namespace['OPERATOR2MANO_RIGHT']
    detector.detected_hand_type = 'Left'
    result = raw_result(label)
    # Non-collinear wrist/index/middle coordinates exercise the original SVD.
    result.multi_hand_world_landmarks[0].landmark[5].x += .02
    result.multi_hand_world_landmarks[0].landmark[9].z -= .03
    result.multi_handedness[0].ListFields = lambda: [(None, [SimpleNamespace(label=label)])]
    if label is None:
        result.multi_hand_landmarks = None
    calls = []
    processor = SimpleNamespace(process=lambda image: (calls.append(image), result)[1])
    rgb = object()
    detector.hand_detector = processor
    baseline = detector.detect(rgb)
    tap = reprobe.ProcessTap(processor)
    detector.hand_detector = tap
    observed = detector.detect(rgb)
    assert tap.calls == 1 and calls == [rgb, rgb]
    assert observed[0] == baseline[0]
    assert observed[2] is baseline[2]
    if label == 'Left':
        np.testing.assert_array_equal(observed[1], baseline[1])
        np.testing.assert_array_equal(observed[3], baseline[3])
        assert observed[1].shape == (21, 3)
    else:
        assert baseline == observed == (0, None, None, None)


@pytest.mark.parametrize('corrupt_sequence', [2, 3])
def test_complete_attempt_log_retains_corrupt_archive_and_never_skips_temporal_gap(
        tmp_path, monkeypatch, corrupt_sequence):
    import numpy as np
    upstream = Path('/home/bx/repos/dex-retargeting/example/vector_retargeting/single_hand_detector.py')
    if not upstream.is_file():
        pytest.skip('Upstream checkout not present')
    detector_path = tmp_path/'upstream/example/vector_retargeting/single_hand_detector.py'
    detector_path.parent.mkdir(parents=True)
    detector_path.write_bytes(upstream.read_bytes())
    recording = tmp_path/'camera'
    recording.mkdir()
    rows = [dict(sequence=0, error='capture clock invalid')]
    bgr = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    for sequence in range(1, 4):
        probe = dict(schema='upstream.probe.v2', sequence=sequence, detected=False,
            image_width=2, image_height=2, pixels=None, local_points_mano=None,
            model_wrist_rotation=None, operator2mano=np.eye(3).tolist())
        rows.append(dict(observation=dict(sequence=sequence), probe=probe,
            geometry=dict(selected_hand='Right', detector_blob=reprobe.DETECTOR_BLOB)))
        path = recording/f'{sequence:06d}.npz'
        if sequence == corrupt_sequence:
            path.write_bytes(b'incomplete archive')
        else:
            np.savez(path, color_bgr=bgr)
    (recording/'frames.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
    calls = []
    raw = SimpleNamespace(multi_hand_landmarks=None, multi_hand_world_landmarks=None,
                          multi_handedness=None)
    def process(image):
        np.testing.assert_array_equal(image, bgr[:, :, ::-1])
        calls.append(image)
        return raw
    class FakeDetector:
        def __init__(self, *, hand_type, selfie):
            assert hand_type == 'Right' and selfie is False
            self.operator2mano = np.eye(3)
            self.detected_hand_type = 'Left'
            self.hand_detector = SimpleNamespace(process=process, close=lambda: None)
        def detect(self, image):
            self.hand_detector.process(image)
            return 0, None, None, None
    class Loader:
        def create_module(self, spec):
            return None
        def exec_module(self, module):
            module.SingleHandDetector = FakeDetector
    loader_spec = importlib.util.spec_from_loader('fake_upstream_reprobe', Loader())
    monkeypatch.setattr(reprobe.importlib.util, 'spec_from_file_location', lambda *args: loader_spec)
    monkeypatch.setattr(reprobe, 'package_versions', lambda: {})
    monkeypatch.setitem(sys.modules, 'cv2', SimpleNamespace(__version__='fake', COLOR_BGR2RGB=1,
        cvtColor=lambda image, code: image[:, :, ::-1].copy()))
    out = tmp_path/'reprobe.jsonl'
    if corrupt_sequence == 2:
        with pytest.raises(RuntimeError, match='not complete'):
            reprobe.run(recording, out, tmp_path/'upstream')
    else:
        reprobe.run(recording, out, tmp_path/'upstream')
    actual = [json.loads(line) for line in out.read_text().splitlines()]
    summary = json.loads(out.with_suffix('.summary.json').read_text())
    assert [row['sequence'] for row in actual] == list(range(4))
    assert actual[0]['status'] == 'capture_error_no_original_inference'
    assert actual[corrupt_sequence]['status'] == 'archive_unreadable'
    assert actual[corrupt_sequence]['recorded_observation'] == dict(sequence=corrupt_sequence)
    assert len(calls) == corrupt_sequence-1 == summary['total_process_calls']
    assert summary['counts']['attempts'] == 4
    assert summary['counts']['archive_errors'] == summary['counts']['capture_errors'] == 1
    assert summary['counts']['exact_probe_match'] == corrupt_sequence-1
    if corrupt_sequence == 2:
        assert actual[3]['status'] == 'not_reprocessed_after_gap'
        assert summary['continuity'] == 'stopped_after_input_or_inference_gap'
    else:
        assert summary['continuity'] == 'terminal_unreadable_archive'


def test_truncated_recording_prefix_cannot_start_fresh_temporal_graph(tmp_path):
    (tmp_path/'frames.jsonl').write_text(json.dumps(dict(sequence=100,error='capture error'))+'\n')
    with pytest.raises(ValueError, match='must begin at zero'):
        reprobe.run(tmp_path, tmp_path/'out.jsonl', tmp_path/'upstream')
    assert not (tmp_path/'out.jsonl').exists()
