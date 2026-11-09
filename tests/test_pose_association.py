import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('pose_association_test',
    Path(__file__).parents[1]/'tools/analyze_pose_association.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def body():
    points = [dict(x=.5, y=.5, z=0., visibility=.9, presence=None) for _ in range(33)]
    points[14] = dict(points[14], x=.2, y=.6)
    points[16] = dict(points[16], x=.2, y=.3)
    points[15] = dict(points[15], x=.8, y=.3)
    return points


def hands(*wrists):
    return dict(image_landmarks=[[dict(x=x,y=y,z=0.) for _ in range(21)] for x,y in wrists],
        handedness=[[dict(label='Right', score=.99, index=1)] for _ in wrists])


def test_unmirrored_wrist_geometry_does_not_silently_use_raw_handedness_as_identity():
    result = audit.associate(body(), hands((.8,.3),(.2,.3)), 640,480)
    assert result['right_wrist_nearest_indices'] == [1]
    first, second = result['candidates']
    assert first['nearer_body_wrist'] == 'left'
    assert second['nearer_body_wrist'] == 'right'
    assert second['handedness'][0]['label'] == 'Right'
    assert second['right_wrist_distance_px'] == 0
    assert first['right_wrist_distance_px'] == pytest.approx(384)
    assert first['right_wrist_distance_forearm_units'] == pytest.approx(384/144)


def test_missing_body_preserves_candidate_count_and_raw_scores_without_invented_geometry():
    result = audit.associate(None, hands((.8,.3)), 640,480)
    assert result['raw_candidate_count'] == 1 and not result['body_available']
    assert result['raw_handedness'][0][0]['score'] == .99
    assert result['candidates'] == [] and result['right_wrist_nearest_indices'] == []


def test_degenerate_body_forearm_and_duplicate_candidates_remain_explicit():
    pose = body(); pose[14] = dict(pose[16])
    result = audit.associate(pose, hands((.2,.3),(.2,.3)), 640,480)
    assert result['right_forearm_projected_length_px'] == 0
    assert all(candidate['right_wrist_distance_forearm_units'] is None for candidate in result['candidates'])
    assert result['right_wrist_nearest_indices'] == [0,1]
    assert result['right_wrist_ranking_margin_px'] == 0
    json.dumps(result, allow_nan=False)


def test_polyline_distance_uses_full_digit_including_degenerate_segments():
    chain = np.array([[0.,0.],[0.,0.],[10.,0.],[10.,10.]])
    assert audit.distance_to_polyline(np.array([7.,3.]),chain) == 3
    assert audit.distance_to_polyline(np.array([10.,6.]),chain) == 0


def test_all_33_landmarks_and_unavailable_presence_are_preserved():
    points = [SimpleNamespace(x=i,y=0,z=0,visibility=.9,presence=0,
                              HasField=lambda name: False) for i in range(33)]
    result = audit.serialize_landmarks(SimpleNamespace(landmark=points))
    assert len(result) == 33 and result[-1]['x'] == 32
    assert result[-1]['presence'] is None and result[-1]['visibility'] == .9
    with pytest.raises(ValueError, match='33'):
        audit.serialize_landmarks(SimpleNamespace(landmark=points[:-1]))


def test_summary_distinguishes_body_absence_from_candidate_absence():
    a = audit.associate(None, hands((.8,.3)),640,480)
    row = dict(status='processed',pose_landmarks=None,inference_elapsed_s=.02,
               association=dict(baseline=a,max2=a))
    summary = audit.summarize([row,dict(status='archive_unreadable')])
    assert summary['counts']['attempts'] == 2
    assert summary['counts']['body_unavailable'] == 1
    assert summary['baseline']['candidate_unavailable'] == 0
    assert summary['baseline']['body_and_candidate_available'] == 0
