import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from anyteleop_so101.perception import *
from conftest import hand_points

@pytest.mark.parametrize('O',[
    np.array([[0,0,-1],[-1,0,0],[0,1,0]]),
    np.array([[0,0,-1],[1,0,0],[0,-1,0]])
])
def test_decode_upstream_inverse(O):
    p=hand_points(); R=Rotation.from_euler('xyz',[.2,.3,.4]).as_matrix()
    local=p@R@O
    restored,r=decode_dex_local(local,R,O)
    assert np.allclose(restored,p); assert np.allclose(r,R)

def test_model_wrist_not_global_position():
    p=hand_points(); t=np.array([.1,.05,.7]); d=p+t
    got,quality=wrist_translation_from_depth(p,d,np.ones(21,dtype=bool))
    assert np.allclose(got,t); assert quality['position_evidence']=='rgbd_fused'
    assert np.allclose(p[0],0)

def test_depth_outlier_rejected():
    p=hand_points(); t=np.array([.1,.05,.7]); d=p+t; d[5]+=[.1,.2,.5]
    got,quality=wrist_translation_from_depth(p,d,np.ones(21,dtype=bool))
    assert np.allclose(got,t); assert 5 not in quality['inlier_indices']

def test_missing_palm_depth_rejected():
    with pytest.raises(ValueError): wrist_translation_from_depth(hand_points(),hand_points()+[0,0,.7],np.zeros(21,dtype=bool))

def test_collinear_depth_geometry_rejected():
    p=np.zeros((21,3)); p[:,0]=np.arange(21)*.01
    with pytest.raises(ValueError): wrist_translation_from_depth(p,p+[0,0,.7],np.ones(21,dtype=bool))

@pytest.mark.parametrize('angle',[.3,.7,1.2])
def test_aperture_rotation_invariant(angle):
    p=hand_points(angle); R=Rotation.from_euler('xyz',[.8,-1.3,.4]).as_matrix()
    assert aperture_angle(p)==pytest.approx(angle)
    assert aperture_angle(p@R.T+[.1,.3,.7])==pytest.approx(angle)

def test_thumb_does_not_move_position_anchor():
    a,b=hand_points(.3),hand_points(1.3); t=np.array([.1,.2,.8])
    ta,_=wrist_translation_from_depth(a,a+t,np.ones(21,dtype=bool))
    tb,_=wrist_translation_from_depth(b,b+t,np.ones(21,dtype=bool))
    assert np.allclose(ta,tb)

def test_rigid_alignment_and_not_pnp():
    a=hand_points()[PALM]; R=Rotation.from_euler('xyz',[.6,.2,.1]).as_matrix(); t=[.1,.2,.7]
    pose,residual=rigid_fit_3d(a,a@R.T+t)
    assert np.allclose(pose.R,R); assert np.allclose(pose.p,t); assert residual<1e-12

def test_pinhole_backprojection():
    K=RectifiedIntrinsics(500,500,320,240,640,480)
    assert np.allclose(K.deproject([370,240],.5),[.05,0,.5])

@pytest.mark.parametrize('uv,z',[([-1,240],.5),([640,240],.5),([320,240],0),([320,240],float('nan'))])
def test_invalid_deprojection(uv,z):
    with pytest.raises(ValueError): RectifiedIntrinsics(500,500,320,240,640,480).deproject(uv,z)

def test_depth_patch_holes_edges_mixed_depth():
    d=np.full((20,20),.7); m=np.ones_like(d,dtype=bool)
    assert depth_patch_median(d,[10,10],m)==pytest.approx(.7)
    assert depth_patch_median(d,[0,0],m) is None
    d[8:13,8:13]=0; assert depth_patch_median(d,[10,10],m) is None
    d[:]=.7; d[:,10:]=1.; assert depth_patch_median(d,[10,10],m) is None

def test_aperture_calibration_validation():
    assert normalize_aperture(.75,.5,1.)==.5
    with pytest.raises(ValueError): normalize_aperture(.5,.5,.51)

@pytest.mark.parametrize('uv',[[-.1,5],[5,-.1],[20,5],[5,20]])
def test_subpixel_outside_image_never_clipped(uv):
    depth=np.full((20,20),.7)
    assert depth_patch_median(depth,uv,np.ones_like(depth,dtype=bool),radius=0) is None
