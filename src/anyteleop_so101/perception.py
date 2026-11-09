"""Pure numerical helpers, NOT a new detector and NOT a validated RGB-D tracker.
The camera worker must provide exposure-aligned pixels, correct distortion handling,
identity association and visibility checks. An available depth is not proof that a
hidden anatomical joint is visible. See docs/03_ALGORITHMS.md.
"""
from dataclasses import dataclass
import numpy as np
from .geometry import Pose, finite_array, proper_rotation, unit

PALM = np.array([0, 5, 9, 13, 17])


@dataclass(frozen=True)
class RectifiedIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def __post_init__(self):
        if not np.all(np.isfinite([self.fx,self.fy,self.cx,self.cy])) or min(self.fx,self.fy,self.width,self.height) <= 0:
            raise ValueError("Invalid intrinsics")

    def deproject(self, uv, z_m):
        uv = finite_array(uv, (2,), "rectified pixel")
        if not (0 <= uv[0] < self.width and 0 <= uv[1] < self.height):
            raise ValueError("Pixel outside image; do not clip it")
        if not np.isfinite(z_m) or z_m <= 0:
            raise ValueError("Invalid depth")
        return np.array([(uv[0]-self.cx)*z_m/self.fx, (uv[1]-self.cy)*z_m/self.fy, z_m])


def depth_patch_median(depth_m, uv, visible_hand_mask, radius=2, min_fraction=.8,
                       max_spread_m=.015, diagnostics=None):
    """Reject borders, holes and mixed-depth patches. Caller supplies a visibility mask.
    Image must be in meters, registered to the SAME pixel grid as uv. This is not
    a substitute for camera-driver alignment or anatomical occlusion reasoning.
    """
    d = np.asarray(depth_m, dtype=float); mask = np.asarray(visible_hand_mask, dtype=bool)
    if d.ndim != 2 or mask.shape != d.shape or radius < 0:
        raise ValueError("Depth/mask shape or radius invalid")
    if not (0 < min_fraction <= 1) or max_spread_m <= 0:
        raise ValueError("Invalid patch thresholds")
    uv = finite_array(uv, (2,), "pixel")
    h,w = d.shape
    if not (0 <= uv[0] < w and 0 <= uv[1] < h):
        if diagnostics is not None: diagnostics['reason']='outside_image'
        return None
    x,y = np.rint(uv).astype(int)
    if x-radius < 0 or y-radius < 0 or x+radius >= w or y+radius >= h:
        if diagnostics is not None: diagnostics['reason']='image_border'
        return None
    patch=d[y-radius:y+radius+1,x-radius:x+radius+1]
    m=mask[y-radius:y+radius+1,x-radius:x+radius+1]
    valid=m & np.isfinite(patch) & (patch>0)
    if diagnostics is not None:
        diagnostics.update(depth_fraction=float((np.isfinite(patch)&(patch>0)).mean()),
                           mask_fraction=float(m.mean()), valid_fraction=float(valid.mean()))
    if valid.mean() < min_fraction:
        if diagnostics is not None:
            diagnostics['reason']='depth_holes' if diagnostics['depth_fraction'] < min_fraction else 'surface_mask_support'
        return None
    values=patch[valid]
    spread=float(np.quantile(values,.9)-np.quantile(values,.1))
    if diagnostics is not None: diagnostics['spread_m']=spread
    if spread > max_spread_m:
        if diagnostics is not None: diagnostics['reason']='mixed_depth_patch'
        return None
    if diagnostics is not None: diagnostics['reason']='valid'
    return float(np.median(values))


def decode_dex_local(joint_pos_mano, mediapipe_wrist_rot, operator2mano):
    """Undo the audited example's row-vector transforms, preserving all 21 points.
    Upstream: joint_pos = (world-world[0]) @ wrist_rot @ operator2mano.
    Result is model-estimated CAMERA-ORIENTED relative geometry, not camera position.
    Pass detector.operator2mano, do not guess left/right or mirror conventions.
    """
    local = finite_array(joint_pos_mano, (21,3), "dex points")
    R = proper_rotation(mediapipe_wrist_rot); O = proper_rotation(operator2mano)
    relative = local @ O.T @ R.T
    if np.linalg.norm(relative[0]) > 1e-5:
        raise ValueError("Expected wrist-rooted upstream coordinates")
    return relative, R


def wrist_translation_from_depth(relative_model, points_c, valid, residual_limit_m=.012,
                                 minimum_inliers=3):
    """Robust RGB-D translation anchor while KEEPING upstream RGB-model orientation.
    For each usable palm point: t_i = depth_point_i - model_relative_point_i.
    Median + residual rejection. Surface/joint bias and model scale error remain;
    numerical consistency is NOT a guarantee of anatomical accuracy.
    """
    relative = finite_array(relative_model,(21,3),"relative model")
    pts=np.asarray(points_c,dtype=float); valid=np.asarray(valid,dtype=bool)
    if pts.shape != (21,3) or valid.shape != (21,):
        raise ValueError("Expected 21 points and mask")
    if minimum_inliers < 3 or residual_limit_m <= 0:
        raise ValueError("Invalid fusion settings")
    idx=PALM[valid[PALM] & np.isfinite(pts[PALM]).all(axis=1) & (pts[PALM,2]>0)]
    if len(idx) < minimum_inliers:
        raise ValueError("insufficient_palm_depth")
    offsets=pts[idx]-relative[idx]
    t=np.median(offsets,axis=0)
    good=np.linalg.norm(offsets-t,axis=1) <= residual_limit_m
    idx=idx[good]
    if len(idx) < minimum_inliers:
        raise ValueError("inconsistent_palm_depth")
    # Need a non-collinear palm patch, not three points on the same edge.
    s=np.linalg.svd(relative[idx]-relative[idx].mean(axis=0),compute_uv=False)
    if s[1] < .003:
        raise ValueError("degenerate_palm_coverage")
    t=np.median(pts[idx]-relative[idx],axis=0)
    residual=float(np.sqrt(np.mean(np.sum((pts[idx]-relative[idx]-t)**2,axis=1))))
    return t, {"inlier_indices":idx.tolist(),"residual_m":residual,
               "position_evidence":"rgbd_fused","orientation_evidence":"rgb_model"}


def rigid_fit_3d(source, target):
    """Kabsch 3D-to-3D alignment. Deliberately NOT called PnP. No outlier rejection."""
    a=np.asarray(source,dtype=float); b=np.asarray(target,dtype=float)
    if a.ndim != 2 or a.shape != b.shape or a.shape[1] != 3 or len(a)<3 or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError("Need matching finite Nx3 points")
    ac=a-a.mean(0); bc=b-b.mean(0)
    if np.linalg.svd(ac,compute_uv=False)[1]<1e-8 or np.linalg.svd(bc,compute_uv=False)[1]<1e-8:
        raise ValueError("Collinear geometry")
    U,_,Vt=np.linalg.svd(ac.T@bc)
    D=np.eye(3); D[2,2]=np.linalg.det(Vt.T@U.T)
    R=Vt.T@D@U.T; t=b.mean(0)-R@a.mean(0)
    rms=float(np.sqrt(np.mean(np.sum((a@R.T+t-b)**2,axis=1))))
    return Pose(t,R),rms


def aperture_angle(points):
    """Thumb 2->4 vs four proximal-to-distal finger rays; rigid-rotation invariant.
    Four fingers should be held together. Thumb out-of-plane motion is a genuine
    ambiguity of this chosen gesture, not an extra gripper DOF.
    """
    p=finite_array(points,(21,3),"hand points")
    fingers=np.array([unit(p[b]-p[a]) for a,b in [(5,8),(9,12),(13,16),(17,20)]])
    fixed=unit(fingers.mean(0)); thumb=unit(p[4]-p[2])
    return float(np.arccos(np.clip(fixed@thumb,-1.,1.)))


def normalize_aperture(angle, closed, opened):
    if not np.isfinite([angle,closed,opened]).all() or opened-closed < .1:
        raise ValueError("Need distinct calibrated open and closed angles, radians")
    return float(np.clip((angle-closed)/(opened-closed),0.,1.))
