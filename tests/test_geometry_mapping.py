import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from anyteleop_so101.geometry import Pose,proper_rotation,rotation_error
from anyteleop_so101.mapping import RelativeMapper,GripperPickup

@pytest.mark.parametrize('angle',[0,.5,1.5,3.14])
def test_pose_roundtrip(angle):
    p=Pose([1,2,3],Rotation.from_rotvec([angle,0,0]).as_matrix())
    back=p.compose(p.inverse()); assert np.linalg.norm(back.p)<1e-12
    assert np.linalg.norm(rotation_error(back.R,np.eye(3)))<1e-12
    data=p.as_dict(); restored=Pose.from_xyzw(data['position_m'],data['quaternion_xyzw'])
    assert np.allclose(restored.R,p.R)

@pytest.mark.parametrize('bad',[np.diag([-1,1,1]),np.eye(3)*2,np.full((3,3),np.nan)])
def test_bad_rotation_rejected(bad):
    with pytest.raises(ValueError): proper_rotation(bad)

@pytest.mark.parametrize('scale',[.5,1.,2.])
def test_engage_never_jumps(scale):
    mapper=RelativeMapper(Rotation.from_euler('z',1).as_matrix(),scale)
    h=Pose([1,2,3],Rotation.from_euler('x',.8).as_matrix())
    r=Pose([.1,.2,.3],Rotation.from_euler('y',.3).as_matrix())
    mapper.engage(h,r); target=mapper.target(h)
    assert np.allclose(target.p,r.p); assert np.allclose(target.R,r.R)

def test_delta_is_not_rotated_by_live_hand():
    m=RelativeMapper(np.eye(3)); m.engage(Pose.identity(),Pose.identity())
    r=Rotation.from_euler('z',np.pi/2).as_matrix()
    target=m.target(Pose([.01,.02,0],r))
    assert np.allclose(target.p,[.01,.02,0]); assert np.allclose(target.R,r)

def test_coordinate_conjugation():
    C=Rotation.from_euler('y',.6).as_matrix(); H=Rotation.from_euler('z',.3).as_matrix()
    m=RelativeMapper(C); m.engage(Pose.identity(),Pose.identity())
    t=m.target(Pose([.01,0,0],H))
    assert np.allclose(t.p,C@[.01,0,0]); assert np.allclose(t.R,C@H@C.T)

def test_reclutch_uses_new_measured_pose():
    m=RelativeMapper(np.eye(3)); m.engage(Pose.identity(),Pose.identity()); m.reset()
    now=Pose([.2,.1,0],Rotation.from_euler('x',.3).as_matrix())
    h=Pose([.8,0,0],np.eye(3)); m.engage(h,now)
    assert np.allclose(m.target(h).p,now.p); assert np.allclose(m.target(h).R,now.R)

def test_gripper_pickup_and_rate_limit():
    g=GripperPickup(max_rate=1); g.engage(.1)
    assert g.update(.9,.01)==.1
    g.update(.1,.01); assert g.picked_up
    assert g.update(.9,.01)==pytest.approx(.11)
    assert g.update(None,.01)==pytest.approx(.11)

def test_gripper_pickup_crossing():
    g=GripperPickup(tolerance=.01); g.engage(.5)
    g.update(.3,.01); g.update(.7,.01); assert g.picked_up

def test_mapping_requires_engage():
    with pytest.raises(RuntimeError): RelativeMapper(np.eye(3)).target(Pose.identity())

def test_no_fake_quaternion_normalization():
    with pytest.raises(ValueError): Pose.from_xyzw([0,0,0],[0,0,0,2])
