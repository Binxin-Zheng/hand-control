"""RGB-D fusion boundary. Depth surfaces are not anatomical visibility proof."""
import numpy as np
from dataclasses import replace
from itertools import combinations
from ..geometry import Pose
from ..perception import PALM, aperture_angle, depth_patch_median
from ..observation_adapter import adapt_probe
from ..surface_position import wrist_surface_fit


def surface_wrist(points, valid):
    """Observed skin point at wrist pixel with noncollinear palm-depth support.

    This is deliberately a different control reference from an anatomical wrist
    inferred by translating the model skeleton. No historical depth is used.
    Keep the baseline's 12 mm residual and 3 mm coverage thresholds.
    """
    indices = PALM[np.asarray(valid, dtype=bool)[PALM]]
    if 0 not in indices or len(indices) < 3:
        raise ValueError('insufficient_wrist_surface_depth')
    best = None
    for subset in combinations(indices, 3):
        if 0 not in subset:
            continue
        patch = points[list(subset)]
        _, singular, vectors = np.linalg.svd(patch-patch.mean(axis=0))
        if singular[1] < .003:
            continue
        residuals = np.abs((points[indices]-patch.mean(axis=0))@vectors[2])
        inliers = indices[residuals <= .012]
        score = (len(inliers), -float(np.sum(residuals[residuals <= .012]**2)))
        if best is None or score > best[0]:
            best = (score, inliers, residuals[residuals <= .012])
    if best is None:
        raise ValueError('degenerate_surface_coverage')
    return points[0].copy(), dict(inlier_indices=best[1].tolist(),
                                  residual_m=float(np.sqrt(np.mean(best[2]**2))))


def fuse_frame(row, depth_m, visibility_mask, deproject, *, timestamp_s, track_id,
               aperture_calibration, frame_id='camera_color_optical', position_method='model_translation',
               wrist_support_mask=None):
    if position_method not in ('model_translation', 'surface_wrist', 'wrist_surface_fit'):
        raise ValueError('Unknown position method')
    depth = np.asarray(depth_m, dtype=float)
    expected = (row['image_height'], row['image_width'])
    if depth.shape != expected or np.shape(visibility_mask) != expected:
        raise ValueError('RGB/depth/visibility must share the registered pixel grid')
    points = np.zeros((21, 3)); valid = np.zeros(21, dtype=bool)
    patches = {}
    if row['detected']:
        pixels = np.asarray(row['pixels'])
        if pixels.shape != (21, 2) or not np.isfinite(pixels).all():
            raise ValueError('Expected all 21 finite pixel coordinates')
        for index in PALM:
            patch = {}; patches[str(index)] = patch
            z = depth_patch_median(depth, pixels[index], visibility_mask, diagnostics=patch)
            if z is not None:
                point = np.asarray(deproject(pixels[index].tolist(), z), dtype=float)
                if point.shape == (3,) and np.isfinite(point).all():
                    points[index] = point; valid[index] = True
    observation, diagnostics = adapt_probe(row, timestamp_s=timestamp_s, track_id=track_id,
        frame_id=frame_id, depth_points_c=points, visible=valid, aperture_calibration=aperture_calibration)
    if position_method == 'surface_wrist' and row['detected']:
        baseline_reason = diagnostics['reason']
        diagnostics = dict(baseline_reason=baseline_reason, global_position_available=False)
        observation = replace(observation, pose=None, position_evidence='unavailable', orientation_evidence='unavailable')
        try:
            position, support = surface_wrist(points, valid)
        except ValueError as exc:
            diagnostics['reason'] = str(exc)
        else:
            observation = replace(observation, pose=Pose(position, row['model_wrist_rotation']),
                                  position_evidence='rgbd_fused', orientation_evidence='rgb_model')
            diagnostics.update(support, reason='surface_wrist_observed', global_position_available=True)
    if position_method == 'wrist_surface_fit' and row['detected']:
        baseline_reason = diagnostics['reason']
        position, support = wrist_surface_fit(row['pixels'], depth, deproject,
            support_mask=visibility_mask if wrist_support_mask is None else wrist_support_mask)
        diagnostics = dict(support, baseline_reason=baseline_reason,
                           global_position_available=position is not None)
        observation = replace(observation,
            pose=None if position is None else Pose(position, row['model_wrist_rotation']),
            position_evidence='unavailable' if position is None else 'rgbd_fused',
            orientation_evidence='unavailable' if position is None else 'rgb_model')
    diagnostics.update(position_method=position_method,
                       position_reference={'surface_wrist': 'wrist_skin_surface',
                                           'wrist_surface_fit': 'wrist_surface_ray',
                                           'model_translation': 'model_wrist_translation'}[position_method])
    angle = None
    if row['detected']:
        try:
            angle = aperture_angle(row['local_points_mano'])
        except ValueError:
            pass  # Invalid finger geometry must never be displayed as an opening measurement.
    diagnostics.update(aperture_angle_rad=angle, depth_valid_indices=np.flatnonzero(valid).tolist(),
                       depth_source='registered_skin_surface', depth_points_c=points.tolist(), depth_patches=patches)
    return observation, diagnostics


def status_lines(observation, diagnostics, *, detected, hand, now_s):
    """Human-readable diagnostics; never substitute cached position for missing depth."""
    pose = observation.pose
    tracking = f"{hand}: {'detected' if detected else 'not selected'} | {diagnostics['reason']}"
    surface_fit = diagnostics.get('position_method') == 'wrist_surface_fit'
    label = 'Wrist surface XYZ (m)' if surface_fit else 'Wrist XYZ (m)'
    position = (label+': unavailable' if pose is None else
                label+': ' + '  '.join(f'{axis}={value:+.3f}' for axis, value in zip('XYZ', pose.p)))
    count = len(diagnostics.get('depth_valid_indices', []))
    rms = diagnostics.get('residual_m')
    depth = ((f"Surface support: {diagnostics.get('surface_inlier_count', 0)} patches | fit RMS: "
              if surface_fit else f"Palm depth: {count}/5 | fusion RMS: ")
             + ('n/a' if rms is None else f'{1000*rms:.1f} mm'))
    rejected = ([] if surface_fit else sorted({p['reason'] for p in diagnostics.get('depth_patches', {}).values() if p['reason'] != 'valid'}))
    if rejected: depth += ' | '+', '.join(rejected)
    age = f"Capture age: {1000*(now_s-observation.timestamp_s):.0f} ms (control limit: 150 ms)"
    angle = diagnostics.get('aperture_angle_rad')
    fingers = 'Finger angle: ' + ('unavailable' if angle is None else f'{angle:.3f} rad')
    fingers += ' | gripper: ' + ('held (uncalibrated/unavailable)' if observation.aperture is None else f'{observation.aperture:.2f}')
    return [tracking, position, depth, age, fingers]
