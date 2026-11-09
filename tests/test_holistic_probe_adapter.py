"""Pure adapter/configuration tests: no MediaPipe graph, image inference or devices."""
import ast
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


spec = importlib.util.spec_from_file_location('holistic_probe_adapter_test',
    Path(__file__).parents[1]/'tools/holistic_probe_adapter.py')
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


def point(x, y=0., z=0., **fields):
    values = dict(x=x, y=y, z=z, visibility=0., presence=0., **fields)
    return SimpleNamespace(**values, HasField=lambda field: field in fields)


def landmarks(count):
    points = [point(i*.005, (i % 4)*.007, 0.) for i in range(count)]
    if count == 21:
        points[0] = point(0.)
        points[5] = point(.03, .05)
        points[9] = point(0., .07)
    return SimpleNamespace(landmark=points)


def result(*sides, body=True):
    values = dict.fromkeys(adapter.OUTPUTS)
    if body:
        values['pose_landmarks'] = landmarks(33)
        values['pose_world_landmarks'] = landmarks(33)
    for side in sides:
        streams = adapter.HAND_STREAMS[side]
        values[streams['image_landmarks']] = landmarks(21)
        values[streams['world_landmarks']] = landmarks(21)
        values[streams['presence_score']] = .95
        values[streams['presence']] = True
        # Deliberately contradictory raw label; anatomical branch selects.
        values[streams['handedness']] = SimpleNamespace(classification=[
            SimpleNamespace(index=1, label='Right', score=.99)])
    return SimpleNamespace(**values)


@pytest.fixture
def upstream_math():
    path = Path(__file__).parents[2]/'dex-retargeting/example/vector_retargeting/single_hand_detector.py'
    if not path.is_file():
        pytest.skip('Pinned upstream checkout is unavailable; no download performed')
    source = path.read_bytes()
    assert adapter.detector_blob(source) == adapter.DETECTOR_BLOB
    tree = ast.parse(source)
    # Compile only the original constants/class with deferred annotations. No
    # imports, constructor calls or substitute math, and no MediaPipe dependency.
    keep = [node for node in tree.body if isinstance(node, ast.ClassDef) or
            isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and
            t.id.startswith('OPERATOR2MANO_') for t in node.targets)]
    prefix = ast.parse('from __future__ import annotations').body
    module = ast.Module(body=prefix+keep, type_ignores=[])
    scope = {'np': np}
    exec(compile(module, str(path), 'exec'), scope)
    return SimpleNamespace(**scope)


def test_all_four_gates_are_point_eight_and_other_settings_preserve_legacy_defaults():
    config = adapter.graph_options()
    assert len(config['calculator_params']) == 4
    assert set(config['calculator_params'].values()) == {.8}
    for index in (1, 2):
        assert any(f'handlandmarksfromposecpu_{index}__handlandmarkcpu__ThresholdingCalculator'
                   in name for name in config['calculator_params'])
    assert config['side_inputs'] == dict(model_complexity=1, smooth_landmarks=True,
        enable_segmentation=False, smooth_segmentation=True,
        refine_face_landmarks=False, use_prev_landmarks=True)
    assert len(config['outputs']) == len(set(config['outputs'])) == 19
    assert not any('handvisibility' in key for key in config['calculator_params'])
    config['calculator_params'].clear()
    assert len(adapter.graph_options()['calculator_params']) == 4


def test_all_landmarks_raw_labels_absent_fields_and_false_flags_are_retained():
    output = result('left', 'right')
    right = adapter.HAND_STREAMS['right']
    setattr(output, right['presence'], False)
    setattr(output, right['presence_score'], .79)
    raw = adapter.capture_raw(output)
    assert len(raw['pose_landmarks']) == len(raw['pose_world_landmarks']) == 33
    for side in ('left', 'right'):
        assert len(raw['hands'][side]['image_landmarks']) == 21
        assert len(raw['hands'][side]['world_landmarks']) == 21
        assert raw['hands'][side]['handedness'][0]['label'] == 'Right'
    assert raw['hands']['right']['presence'] is False
    assert raw['hands']['right']['presence_score'] == .79
    assert raw['hands']['right']['world_landmarks'][0]['presence'] is None
    assert raw['hands']['right']['tracking_roi'] is None
    json.dumps(raw, allow_nan=False)


def test_body_right_uses_original_all_21_math_without_raw_left_inversion(upstream_math):
    output = result('left', 'right')
    raw = adapter.capture_raw(output)
    actual = adapter.adapt_body_right(output, raw, upstream_math, (480,640,3), 17)
    assert actual['probe']['detected'] and actual['probe']['sequence'] == 17
    expected_world = upstream_math.SingleHandDetector.parse_keypoint_3d(
        getattr(output, adapter.HAND_STREAMS['right']['world_landmarks']))
    centered = expected_world-expected_world[0]
    rotation = upstream_math.SingleHandDetector.estimate_frame_from_hand_points(centered)
    np.testing.assert_array_equal(actual['probe']['local_points_mano'],
                                  centered @ rotation @ upstream_math.OPERATOR2MANO_RIGHT)
    np.testing.assert_array_equal(actual['probe']['pixels'],
        upstream_math.SingleHandDetector.parse_keypoint_2d(output.right_hand_landmarks,(480,640)))
    assert actual['probe']['local_points_mano'][0] == [0.,0.,0.]
    assert len(actual['probe']['local_points_mano']) == 21
    assert not actual['diagnostics']['global_translation_available']
    assert not actual['diagnostics']['persistent_person_identity']


@pytest.mark.parametrize('missing', ['body', 'right', 'world', 'image', 'presence_false', 'presence_none'])
def test_missing_current_outputs_never_select_left_or_reuse_previous(missing, upstream_math):
    output = result('left', 'right', body=missing != 'body')
    if missing == 'right':
        output = result('left')
    elif missing in ('world', 'image', 'presence_false', 'presence_none'):
        field = {'world':'world_landmarks', 'image':'image_landmarks',
                 'presence_false':'presence', 'presence_none':'presence'}[missing]
        setattr(output, adapter.HAND_STREAMS['right'][field], False if missing == 'presence_false' else None)
    actual = adapter.adapt_body_right(output, adapter.capture_raw(output), upstream_math,(480,640,3),3)
    assert not actual['probe']['detected']
    assert actual['probe']['pixels'] is None and actual['probe']['local_points_mano'] is None
    assert actual['raw']['hands']['left']['image_landmarks'] is not None


def test_presence_flag_below_configured_threshold_is_not_silently_accepted(upstream_math):
    output = result('right')
    setattr(output, adapter.HAND_STREAMS['right']['presence_score'], .79)
    with pytest.raises(ValueError, match='contradicts'):
        adapter.adapt_body_right(output, adapter.capture_raw(output), upstream_math,(480,640,3),0)


def test_malformed_or_nonfinite_raw_outputs_raise():
    output = result('right')
    output.right_hand_landmarks.landmark.pop()
    with pytest.raises(ValueError, match='21'):
        adapter.capture_raw(output)
    output = result('right')
    output.pose_landmarks.landmark[0].x = float('nan')
    with pytest.raises(ValueError):
        adapter.capture_raw(output)


def test_one_process_call_receives_identical_rgb_and_no_cached_hand(upstream_math):
    frames = [result('right'), result()]
    received = []
    class FakeGraph:
        def process(self, input_data):
            received.append(input_data['image'])
            return frames.pop(0)
    probe = adapter.HolisticProbe.__new__(adapter.HolisticProbe)
    probe._graph, probe._upstream, probe.calls = FakeGraph(), upstream_math, 0
    rgb = np.zeros((3,4,3), dtype=np.uint8)
    first, second = probe.process(rgb, 0), probe.process(rgb, 1)
    assert len(received) == probe.calls == 2 and all(image is rgb for image in received)
    assert first['probe']['detected'] and not second['probe']['detected']
    assert second['raw']['hands']['right']['world_landmarks'] is None
    assert first['diagnostics']['inference_elapsed_s'] >= 0
    with pytest.raises(ValueError, match='uint8'):
        probe.process(rgb.astype(float), 2)
    assert probe.calls == 2


def test_constructor_rejects_wrong_hand_or_unpinned_source_before_inference(tmp_path):
    with pytest.raises(ValueError, match='Right'):
        adapter.HolisticProbe(tmp_path, hand='Left')
    source = tmp_path/'example/vector_retargeting/single_hand_detector.py'
    source.parent.mkdir(parents=True)
    source.write_text('raise AssertionError("must not import")')
    with pytest.raises(ValueError, match='pinned'):
        adapter.HolisticProbe(tmp_path)


def test_roi_preserves_normalized_position_extent_and_rotation():
    fields = dict(x_center=.2, y_center=.7, width=.4, height=.3, rotation=-1.2)
    roi = SimpleNamespace(**fields, HasField=lambda field: field in fields)
    assert adapter.serialize_roi(roi) == fields
