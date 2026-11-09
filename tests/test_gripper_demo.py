import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from anyteleop_so101.geometry import Pose
from anyteleop_so101.mapping import RelativeMapper

spec=importlib.util.spec_from_file_location('gripper_demo',Path(__file__).parents[1]/'tools/gripper_test.py')
demo=importlib.util.module_from_spec(spec);spec.loader.exec_module(demo)


def packet(angle, timestamp=1., detected=True):
    return dict(schema='teleop.gripper_preview.v1', simulation_only=True,
                timestamp_s=timestamp, angle_rad=angle, hand='Right', detected=detected)


def test_direct_follow_requires_no_calibration_or_wrist_xyz():
    assert demo.preview_target(packet(.7), 1.01, -.17, 1.75) == pytest.approx(.7)
    assert demo.preview_target(packet(2.), 1.01, -.17, 1.75) == pytest.approx(1.75)
    assert demo.preview_target(packet(0.), 1.01, -.17, 1.75) == pytest.approx(0.)


def test_missing_and_stale_preview_input_is_not_observed_motion():
    assert demo.preview_target(packet(None, detected=False), 1.01, -.17, 1.75) is None
    assert demo.preview_target(packet(.7), 1.2, -.17, 1.75) is None
    assert demo.preview_target(packet(float('nan')), 1.01, -.17, 1.75) is None
    assert demo.preview_target(packet(.7), 1.01, -.17, 1.75, 'Left') is None
    # Simulation-only reacquisition follows directly, without a calibration latch.
    assert demo.preview_target(packet(.8, 1.3), 1.31, -.17, 1.75) == pytest.approx(.8)
    with pytest.raises(ValueError): demo.preview_target({'schema':'teleop.vision.v1'},1.,-.17,1.75)


def test_mirror_first_person_mapping_is_proper_rotation():
    config=json.loads((Path(__file__).parents[1]/'config/runtime.sim.json').read_text())
    mapper=RelativeMapper(config['R_base_camera'],1.)
    assert np.linalg.det(mapper.C)==pytest.approx(1.)
    mapper.engage(Pose([0,0,.6],np.eye(3)),Pose([0,0,0],np.eye(3)))
    for camera_delta,robot_delta in [([0,0,-.1],[.1,0,0]),([0,-.1,0],[0,0,.1]),([-.1,0,0],[0,-.1,0])]:
        target=mapper.target(Pose(np.array([0,0,.6])+camera_delta,np.eye(3)))
        np.testing.assert_allclose(target.p,robot_delta,atol=1e-12)
        np.testing.assert_allclose(target.R,np.eye(3))


def test_palm_proximity_endpoints_and_rigid_scale_invariance():
    from anyteleop_so101.gesture import thumb_palm_opening
    from conftest import hand_points
    from scipy.spatial.transform import Rotation
    points=hand_points();points[4]=points[[0,5,9]].mean(axis=0)
    assert thumb_palm_opening(points)[0]==0
    points[4,2]+=.1
    assert thumb_palm_opening(points)[0]==1
    rotation=Rotation.from_euler('xyz',[.3,-.9,.5]).as_matrix()
    np.testing.assert_allclose(thumb_palm_opening(points),thumb_palm_opening(2*points@rotation.T+[1,2,3]))
    for opening,target in [(0,0),(1,1.74533),(.5,.872665)]:
        value=packet(.7);value.update(mapping='thumb_palm_proximity',opening=opening)
        assert demo.preview_target(value,1.01,-.174533,1.74533)==pytest.approx(target)


def test_fixed_jaw_model_signed_angle_and_palm_normal_rejection():
    from anyteleop_so101.gesture import thumb_jaw_opening
    from conftest import hand_points
    from scipy.spatial.transform import Rotation
    for angle,expected in [(-.4,0),(0,0),(np.pi/4,.5),(np.pi/2,1)]:
        points=hand_points(angle)
        opening,measured=thumb_jaw_opening(points)
        assert opening==pytest.approx(expected)
        assert measured==pytest.approx(angle)
        # Folding toward palm normal must not masquerade as outward jaw rotation.
        points[4,2]+=.03
        assert thumb_jaw_opening(points)[0]==pytest.approx(expected)
        rotation=Rotation.from_euler('xyz',[.5,-1.,.8]).as_matrix()
        assert thumb_jaw_opening(points@rotation.T*1.8+[1,2,3])[0]==pytest.approx(expected)
        packet_value=packet(.1);packet_value.update(mapping='thumb_fixed_jaw',opening=opening)
        assert demo.preview_target(packet_value,1.01,-.174533,1.74533)==pytest.approx(expected*1.74533)


def test_dex_preview_accepts_actual_negative_closed_joint_angle():
    value=packet(-.174533);value.update(mapping='dex_palm_thumb',opening=0.)
    assert demo.preview_target(value,1.01,-.174533,1.74533)==pytest.approx(-.174533)
    value['timestamp_s']=.5
    assert demo.preview_target(value,1.01,-.174533,1.74533) is None


def test_palm_binding_does_not_depend_on_finger_tip_motion():
    from anyteleop_so101.dex_gripper import palm_thumb_vector
    from conftest import hand_points
    from scipy.spatial.transform import Rotation
    p=hand_points(.5)
    target=palm_thumb_vector(p)
    p[[8,12,16,20]] += [.02,-.04,.03]
    np.testing.assert_allclose(palm_thumb_vector(p),target)
    R=Rotation.from_euler('xyz',[1.,-.8,2.]).as_matrix()
    np.testing.assert_allclose(palm_thumb_vector(2*p@R.T+[1,2,3]),target,atol=1e-12)
