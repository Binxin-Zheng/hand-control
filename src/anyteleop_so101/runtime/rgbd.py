"""RGB-D fusion boundary. Depth surfaces are not anatomical visibility proof."""
import numpy as np
from ..perception import PALM, depth_patch_median
from ..observation_adapter import adapt_probe


def fuse_frame(row, depth_m, visibility_mask, deproject, *, timestamp_s, track_id,
               aperture_calibration, frame_id='camera_color_optical'):
    depth = np.asarray(depth_m, dtype=float)
    expected = (row['image_height'], row['image_width'])
    if depth.shape != expected or np.shape(visibility_mask) != expected:
        raise ValueError('RGB/depth/visibility must share the registered pixel grid')
    points = np.zeros((21, 3)); valid = np.zeros(21, dtype=bool)
    if row['detected']:
        pixels = np.asarray(row['pixels'])
        if pixels.shape != (21, 2) or not np.isfinite(pixels).all():
            raise ValueError('Expected all 21 finite pixel coordinates')
        for index in PALM:
            z = depth_patch_median(depth, pixels[index], visibility_mask)
            if z is not None:
                point = np.asarray(deproject(pixels[index].tolist(), z), dtype=float)
                if point.shape == (3,) and np.isfinite(point).all():
                    points[index] = point; valid[index] = True
    observation, diagnostics = adapt_probe(row, timestamp_s=timestamp_s, track_id=track_id,
        frame_id=frame_id, depth_points_c=points, visible=valid, aperture_calibration=aperture_calibration)
    diagnostics.update(depth_valid_indices=np.flatnonzero(valid).tolist(),
                       depth_source='registered_skin_surface', depth_points_c=points.tolist())
    return observation, diagnostics
