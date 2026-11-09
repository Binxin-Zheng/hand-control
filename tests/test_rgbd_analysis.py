"""Audit statistics use logged clocks/evidence without loading a camera SDK."""
import importlib.util
import json
import math
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    'rgbd_analysis', Path(__file__).parents[1]/'tools/analyze_rgbd_recording.py')
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def observation(x=0., *, evidence='rgbd_fused', track='right', frame='camera'):
    return dict(position_evidence=evidence, track_id=track, frame_id=frame,
                pose=dict(position_m=[x, 0., .5], quaternion_xyzw=[0., 0., 0., 1.]))


def add(stats, sequence, timestamp, obs=None, *, detected=True, geometry='g', reason='valid'):
    return stats.add(sequence=sequence, timestamp_s=timestamp, detected=detected,
        observation=obs, diagnostics=dict(reason=reason, position_reference='wrist_surface'),
        elapsed_s=.001 if timestamp is not None else None, geometry_sha256=geometry)


def test_denominators_include_capture_loss_and_predictions_are_not_observations():
    stats = analysis.MethodSummary()
    stats.add(sequence=0, timestamp_s=None, detected=False)
    add(stats, 1, 1., detected=False, reason='upstream_not_selected')
    add(stats, 2, 1.1, observation(evidence='predicted'), reason='prediction')
    add(stats, 3, 1.2, observation(evidence='monocular_model'), reason='model_only')
    add(stats, 4, 1.4, observation())
    result = stats.as_dict()
    assert result['attempts'] == 5
    assert result['selected_hand_frames'] == 3
    assert result['observed_position_frames'] == 1
    assert result['pose_present_frames'] == 3
    assert result['availability']['all_attempts'] == dict(numerator=1, denominator=5, fraction=.2)
    assert result['availability']['selected_hand']['fraction'] == pytest.approx(1/3)
    assert result['longest_invalid_frame_streak'] == 4
    assert result['longest_invalid_sample_span_s'] == pytest.approx(.2)
    assert result['missing_timestamps'] == 1
    assert result['counts']['capture_error'] == 1
    assert result['fusion_elapsed_s']['count'] == 4


def test_motion_uses_actual_elapsed_time_and_unavailable_frames_break_pairs():
    stats = analysis.MethodSummary()
    add(stats, 0, 1., observation(0.))
    first = add(stats, 1, 1.2, observation(.02))
    second = add(stats, 2, 1.7, observation(.07))
    add(stats, 3, 1.8, reason='depth_holes')
    assert add(stats, 4, 2., observation(.2)) is None
    assert first['dt_s'] == pytest.approx(.2)
    assert first['speed_m_s'] == pytest.approx(.1)
    assert second['dt_s'] == pytest.approx(.5)
    assert second['speed_m_s'] == pytest.approx(.1)
    result = stats.as_dict()
    assert result['longest_valid_frame_streak'] == 3
    assert result['longest_valid_sample_span_s'] == pytest.approx(.7)
    assert result['adjacent_observed_motion']['step_m']['count'] == 2
    assert result['adjacent_observed_motion']['step_m']['max'] == pytest.approx(.05)


def test_reference_identity_clock_and_sequence_discontinuities_do_not_create_motion():
    stats = analysis.MethodSummary()
    add(stats, 0, 1., observation())
    assert add(stats, 1, 1.1, observation(.1, track='other')) is None
    assert add(stats, 2, 1.1, observation(.2, track='other')) is None
    assert add(stats, 4, 1.4, observation(.3, track='other')) is None
    assert add(stats, 5, 1.5, observation(.4, track='other'), geometry='new') is None
    result = stats.as_dict()
    assert result['nonincreasing_timestamps'] == 1
    rejected = result['adjacent_observed_motion']['rejected_pairs']
    assert rejected == dict(identity_or_frame_change=1, missing_or_nonincreasing_timestamp=1,
                            nonconsecutive_recorded_sequence=1, geometry_or_reference_change=1)
    assert result['adjacent_observed_motion']['speed_m_s']['count'] == 0


def test_missing_capture_timestamps_do_not_invent_outage_duration():
    stats = analysis.MethodSummary()
    add(stats, 0, 1., reason='depth_holes')
    stats.add(sequence=1, timestamp_s=None, detected=False)
    add(stats, 2, 10., reason='depth_holes')
    add(stats, 3, 10.2, reason='depth_holes')
    result = stats.as_dict()
    assert result['longest_invalid_frame_streak'] == 4
    assert result['longest_invalid_sample_span_s'] == pytest.approx(.2)


def test_empty_summary_has_json_nulls_and_method_validation_precedes_sdk_import():
    result = analysis.MethodSummary().as_dict()
    assert result['availability']['all_attempts']['fraction'] is None
    assert result['fusion_elapsed_s']['p95'] is None
    json.dumps(result, allow_nan=False)
    assert analysis.DEFAULT_METHODS == ('model_translation', 'surface_wrist')
    assert 'wrist_surface_fit' in analysis.POSITION_METHODS
    with pytest.raises(ValueError, match='distinct supported'):
        analysis.run(Path('/missing'), Path('/missing'), methods=['surface_wrist', 'surface_wrist'])


def test_incomplete_archive_retains_attempt_detection_and_original_evidence(tmp_path, monkeypatch):
    import sys
    import types
    import numpy as np
    # The unreadable-depth branch must not call a camera SDK or image operation.
    monkeypatch.setitem(sys.modules, 'cv2', types.SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'pyrealsense2', types.SimpleNamespace())
    source_observation = dict(observation(), sequence=0, timestamp_s=1.)
    row = dict(observation=source_observation, geometry={'detector_blob': 'recorded_blob'},
               probe=dict(detected=True, model_wrist_rotation=[[1,0,0],[0,1,0],[0,0,1]]),
               diagnostics={'reason': 'recorded_observed'}, geometry_sha256='g')
    (tmp_path/'frames.jsonl').write_text(json.dumps(row)+'\n')
    np.savez(tmp_path/'000000.npz', color_bgr=np.zeros((1, 1, 3)))
    summary = analysis.run(tmp_path, tmp_path/'result.jsonl', methods=['wrist_surface_fit'])
    assert summary['attempts'] == summary['selected_hand_frames'] == 1
    assert summary['unreadable_depth_archives'] == 1
    method = summary['methods']['wrist_surface_fit']
    assert method['observed_position_frames'] == 0
    assert method['counts'] == {'recording_depth_unreadable': 1}
    assert method['availability']['selected_hand']['denominator'] == 1
    result = json.loads((tmp_path/'result.jsonl').read_text())
    assert result['recorded_observation'] == source_observation
    assert result['recorded_diagnostics'] == row['diagnostics']
    assert result['methods']['wrist_surface_fit']['observation'] is None
    assert result['input_frame_sha256'] == analysis.file_sha256(tmp_path/'000000.npz')


def test_corrupt_or_missing_depth_archive_returns_explicit_error(tmp_path):
    missing = tmp_path/'missing.npz'
    assert analysis.read_recorded_depth(missing) == (None, None, {
        'type': 'FileNotFoundError', 'message': str(FileNotFoundError(2, 'No such file or directory', str(missing)))})
    corrupt = tmp_path/'corrupt.npz'
    corrupt.write_bytes(b'PK\x03\x04broken')
    depth, digest, error = analysis.read_recorded_depth(corrupt)
    assert depth is None
    assert digest == analysis.file_sha256(corrupt)
    assert error['type'] == 'BadZipFile'


def test_recording_hash_is_content_based(tmp_path):
    path = tmp_path/'sample'
    path.write_bytes(b'abc')
    assert analysis.file_sha256(path) == 'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad'


def test_orientation_guard_uses_logged_delta_and_breaks_on_tracking_loss():
    previous = dict(sequence=0, timestamp_s=1., track_id='a', frame_id='camera',
                    geometry_sha256='g', rotation=[[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    angle = .6
    current = dict(previous, sequence=1, timestamp_s=1.05,
        rotation=[[math.cos(angle), -math.sin(angle), 0.],
                  [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]])
    pair = analysis.model_orientation_pair(previous, current)
    assert pair['angle_rad'] == pytest.approx(.6)
    assert pair['guard_limit_rad'] == pytest.approx(.52)
    assert pair['guard_exceeded']
    current['timestamp_s'] = 1.1
    assert not analysis.model_orientation_pair(previous, current)['guard_exceeded']
    assert analysis.model_orientation_pair(None, current) is None
    current['track_id'] = 'b'
    assert analysis.model_orientation_pair(previous, current) is None


def test_orientation_failures_on_usable_position_pairs_have_explicit_denominator():
    stats = analysis.MethodSummary()
    add(stats, 0, 1., observation())
    stats.add(sequence=1, timestamp_s=1.1, detected=True, observation=observation(.01),
        diagnostics=dict(reason='valid', position_reference='wrist_surface'), geometry_sha256='g',
        model_orientation=dict(angle_rad=1., guard_exceeded=True))
    stats.add(sequence=2, timestamp_s=1.2, detected=True, observation=None,
        diagnostics=dict(reason='depth_holes'), geometry_sha256='g',
        model_orientation=dict(angle_rad=2., guard_exceeded=True))
    result = stats.as_dict()['model_orientation_on_observed_pairs']
    assert result['pairs'] == 1
    assert result['guard_exceeded_pairs'] == 1
    assert result['angle_rad']['max'] == 1.
