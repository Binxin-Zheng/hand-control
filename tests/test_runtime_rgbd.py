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


def test_uncalibrated_preview_reports_angle_and_metric_position():
    from anyteleop_so101.runtime.rgbd import status_lines
    row,k=frame()
    obs,diag=fuse_frame(row,np.full((480,640),.6),np.ones((480,640),bool),k.deproject,
                       timestamp_s=1.,track_id='camera1:right',aperture_calibration=None)
    assert diag['aperture_angle_rad']==pytest.approx(.7)
    assert obs.aperture is None
    text='\n'.join(status_lines(obs,diag,detected=True,hand='Right',now_s=1.02))
    assert 'Z=+0.600' in text and '20 ms' in text and '0.700 rad' in text
    assert 'held (uncalibrated/unavailable)' in text


def test_preview_does_not_display_fake_xyz_on_depth_failure():
    from anyteleop_so101.runtime.rgbd import status_lines
    row,k=frame()
    obs,diag=fuse_frame(row,np.zeros((480,640)),np.ones((480,640),bool),k.deproject,
                       timestamp_s=1.,track_id='camera1:right',aperture_calibration=None)
    text='\n'.join(status_lines(obs,diag,detected=True,hand='Right',now_s=1.02))
    assert 'Wrist XYZ (m): unavailable' in text
    assert 'insufficient_palm_depth' in text


def test_surface_mode_uses_current_depth_not_model_translation():
    row,k=frame()
    # Deliberately inconsistent model shape, while observed planar depth is valid.
    model=np.asarray(row['local_points_mano']); model[:,2]=np.linspace(0,.2,21)
    row['local_points_mano']=model.tolist()
    obs,diag=fuse_frame(row,np.full((480,640),.6),np.ones((480,640),bool),k.deproject,
                       timestamp_s=1.,track_id='right',aperture_calibration=None,
                       position_method='surface_wrist')
    np.testing.assert_allclose(obs.pose.p,k.deproject(row['pixels'][0],.6))
    assert diag['position_reference']=='wrist_skin_surface'
    assert diag['baseline_reason']=='inconsistent_palm_depth'
    assert obs.position_evidence=='rgbd_fused'


def test_surface_mode_rejects_missing_wrist_even_with_other_palm_depth():
    row,k=frame(); depth=np.full((480,640),.6)
    x,y=np.rint(row['pixels'][0]).astype(int);depth[y-3:y+4,x-3:x+4]=0
    obs,diag=fuse_frame(row,depth,np.ones((480,640),bool),k.deproject,
                       timestamp_s=1.,track_id='right',aperture_calibration=None,
                       position_method='surface_wrist')
    assert obs.pose is None and obs.position_evidence=='unavailable'
    assert diag['reason']=='insufficient_wrist_surface_depth'
