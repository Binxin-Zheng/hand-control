import numpy as np
import pytest
from anyteleop_so101.runtime.rgbd import fuse_frame
from anyteleop_so101.perception import RectifiedIntrinsics
from conftest import hand_points


def frame():
    points=hand_points(); z=.6
    k=RectifiedIntrinsics(500,500,320,240,640,480)
    pixels=points[:,:2]*500/z+[320,240]
    row=dict(schema='upstream.probe.v2',sequence=0,detected=True,image_width=640,image_height=480,
             pixels=pixels.tolist(),local_points_mano=points.tolist(),
             model_wrist_rotation=np.eye(3).tolist(),operator2mano=np.eye(3).tolist())
    return row,k


def test_registered_depth_fuses_correct_global_wrist():
    row,k=frame()
    obs,diag=fuse_frame(row,np.full((480,640),.6),np.ones((480,640),bool),k.deproject,
                       timestamp_s=1.,track_id='camera1:right',aperture_calibration=(.1,1.2))
    np.testing.assert_allclose(obs.pose.p,[0,0,.6],atol=1e-12)
    assert obs.position_evidence=='rgbd_fused' and obs.aperture is not None
    assert diag['depth_source']=='registered_skin_surface'


def test_depth_holes_reject_pose_without_losing_valid_finger_channel():
    row,k=frame()
    obs,diag=fuse_frame(row,np.zeros((480,640)),np.ones((480,640),bool),k.deproject,
                       timestamp_s=1.,track_id='camera1:right',aperture_calibration=(.1,1.2))
    assert obs.pose is None and obs.aperture is not None
    assert not diag['depth_valid_indices']


def test_unregistered_resolution_is_rejected():
    row,k=frame()
    with pytest.raises(ValueError):
        fuse_frame(row,np.ones((240,320)),np.ones((240,320),bool),k.deproject,
                   timestamp_s=1.,track_id='camera1:right',aperture_calibration=(.1,1.2))
