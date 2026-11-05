"""Explicit upstream-to-core boundary, without cameras, predictions or motor I/O.

The caller supplies exposure-aligned, deprojected depth and a visibility mask.
A plain RGB recording deliberately produces no global pose. File presentation
clock must be explicitly rebased by the replay caller, never sent to live control.
"""
from .geometry import Pose
from .perception import (aperture_angle, decode_dex_local, normalize_aperture,
                         wrist_translation_from_depth)
from .protocol import HandObservation


def adapt_probe(row, *, timestamp_s, track_id, frame_id='camera_color_optical',
                depth_points_c=None, visible=None, aperture_calibration=None):
    if row.get('schema') != 'upstream.probe.v2':
        raise ValueError('Expected explicit upstream.probe.v2 adapter input')
    pose = None
    aperture = None
    diagnostics = {'reason': 'upstream_not_selected', 'global_position_available': False}
    if (depth_points_c is None) != (visible is None):
        raise ValueError('Depth points and anatomical visibility must be supplied together')
    if row['detected']:
        relative, rotation = decode_dex_local(row['local_points_mano'],
                                              row['model_wrist_rotation'], row['operator2mano'])
        diagnostics['reason'] = 'global_translation_unavailable'
        if aperture_calibration is not None:
            aperture = normalize_aperture(aperture_angle(relative), *aperture_calibration)
        if depth_points_c is not None:
            try:
                translation, fusion = wrist_translation_from_depth(relative, depth_points_c, visible)
            except ValueError as exc:
                diagnostics['reason'] = str(exc)
            else:
                pose = Pose(translation, rotation)
                diagnostics.update(fusion)
                diagnostics.update(reason='rgbd_fused', global_position_available=True)
    observation = HandObservation(
        sequence=row['sequence'], timestamp_s=timestamp_s, track_id=track_id,
        frame_id=frame_id, pose=pose, aperture=aperture,
        position_evidence='rgbd_fused' if pose is not None else 'unavailable',
        orientation_evidence='rgb_model' if pose is not None else 'unavailable',
        aperture_evidence='rgb_model' if aperture is not None else 'unavailable',
    )
    return observation, diagnostics
