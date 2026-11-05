import numpy as np
import pytest
from anyteleop_so101.observation_adapter import adapt_probe


def probe():
    points = np.zeros((21, 3))
    points[[5, 9, 13, 17]] = [[-.03, .04, 0], [0, .06, 0], [.02, .05, 0], [.04, .03, 0]]
    return dict(schema='upstream.probe.v2', sequence=4, detected=True,
                local_points_mano=points.tolist(), model_wrist_rotation=np.eye(3).tolist(),
                operator2mano=np.eye(3).tolist())


def test_rgb_never_becomes_global_pose():
    obs, diagnostic = adapt_probe(probe(), timestamp_s=10, track_id='right:1')
    assert obs.pose is None and obs.position_evidence == 'unavailable'
    assert diagnostic['reason'] == 'global_translation_unavailable'


def test_fused_translation_preserves_all_points_and_orientation():
    row = probe()
    depth = np.array(row['local_points_mano']) + [.1, -.1, .6]
    obs, diagnostic = adapt_probe(row, timestamp_s=10, track_id='right:1',
                                 depth_points_c=depth, visible=np.ones(21, bool))
    np.testing.assert_allclose(obs.pose.p, [.1, -.1, .6])
    np.testing.assert_allclose(obs.pose.R, np.eye(3))
    assert obs.position_evidence == 'rgbd_fused'
    assert len(row['local_points_mano']) == 21
    assert diagnostic['global_position_available']


def test_invalid_depth_is_unavailable_not_last_good_pose():
    obs, diagnostic = adapt_probe(probe(), timestamp_s=10, track_id='right:1',
                                 depth_points_c=np.zeros((21, 3)), visible=np.ones(21, bool))
    assert obs.pose is None
    assert diagnostic['reason'] == 'insufficient_palm_depth'


def test_no_hand_does_not_use_depth():
    row = probe(); row['detected'] = False
    obs, _ = adapt_probe(row, timestamp_s=10, track_id='right:1')
    assert obs.pose is None and obs.aperture is None


def test_depth_requires_visibility():
    with pytest.raises(ValueError, match='together'):
        adapt_probe(probe(), timestamp_s=10, track_id='right:1', depth_points_c=np.zeros((21, 3)))
