"""Current-frame RGB-D wrist-surface positioning; no detector or temporal filling.

The reference is the intersection of the detected wrist ray and a locally fitted
visible surface. It is NOT an anatomical wrist joint. We use depth-connected
patches surrounding the ray, rather than subtracting bone-model coordinates from
skin depth. An unsupported ray is rejected, including when nearby depth exists.
"""
import numpy as np
from scipy.spatial import ConvexHull, QhullError

from .geometry import finite_array


SURFACE_FIT_VERSION = 3
# Explicit desktop-demo measurement envelope. In particular, a saturated Z16
# value near 65 m is not a plausible hand observation. Out-of-range frames are
# retained as failures; this is not a depth clipping/filling operation.
SURFACE_DEPTH_RANGE_M = (.05, 2.)
MAX_WRIST_DEPTH_GAP_PX = 4.
MAX_WRIST_PATCH_DISTANCE_PX = 6.


def wrist_surface_support_mask(pixels, image_shape):
    """Geometric association region with at most a 24px half-width.

    This bounded square around landmark 0 is not hand segmentation, visibility,
    or evidence of a wrist surface. Depth and support checks still decide whether
    any current surface is observed inside it.
    """
    uv = finite_array(pixels, (21, 2), 'all image landmarks')
    if len(image_shape) != 2 or any(int(n) != n or n <= 0 for n in image_shape):
        raise ValueError('Image shape must contain positive height and width')
    height, width = (int(n) for n in image_shape)
    mask = np.zeros((height, width), dtype=bool)
    x, y = uv[0]
    if 0 <= x < width and 0 <= y < height:
        x0, y0 = np.ceil(uv[0]-24).astype(int)
        x1, y1 = np.floor(uv[0]+24).astype(int)
        mask[max(0, y0):min(height, y1+1), max(0, x0):min(width, x1+1)] = True
    return mask


def _surrounds(pixels, reference):
    try:
        hull = ConvexHull(pixels)
    except QhullError:
        return False
    return bool(np.all(hull.equations[:, :2] @ reference + hull.equations[:, 2] <= 1e-8))


def _patch_samples(depth, support_mask, reference, radius):
    """Vectorized original 5x5, 80%-valid, 15mm-spread patch criteria."""
    height, width = depth.shape
    x0, y0 = np.ceil(reference-radius).astype(int)
    x1, y1 = np.floor(reference+radius).astype(int)
    xs = np.arange(max(2, x0+2), min(width-2, x1-1), 4)
    ys = np.arange(max(2, y0+2), min(height-2, y1-1), 4)
    xx, yy = np.meshgrid(xs, ys)
    pixels = np.column_stack((xx.ravel(), yy.ravel()))
    if not len(pixels):
        return np.empty((0, 3)), 0
    offsets = np.arange(-2, 3)
    patches = depth[pixels[:, 1, None, None]+offsets[None, :, None],
                    pixels[:, 0, None, None]+offsets[None, None, :]].reshape(-1, 25)
    valid = (np.isfinite(patches) & (patches >= SURFACE_DEPTH_RANGE_M[0])
             & (patches <= SURFACE_DEPTH_RANGE_M[1]))
    valid &= support_mask[pixels[:, 1, None, None]+offsets[None, :, None],
                          pixels[:, 0, None, None]+offsets[None, None, :]].reshape(-1, 25)
    accepted = valid.mean(axis=1) >= .8
    selected = patches[accepted].copy()
    selected[~valid[accepted]] = np.nan
    if not len(selected):
        return np.empty((0, 3)), len(pixels)
    quantiles = np.nanquantile(selected, [.1, .5, .9], axis=1)
    coherent = quantiles[2]-quantiles[0] <= .015
    return np.column_stack((pixels[accepted][coherent], quantiles[1, coherent])), len(pixels)


def _components(samples):
    """Separate neighboring skin and background surfaces before fitting."""
    delta = np.abs(samples[:, None, :]-samples[None, :, :])
    adjacent = ((delta[:, :, 0] <= 4.01) & (delta[:, :, 1] <= 4.01)
                & (delta[:, :, 2] <= .015))
    seen = np.zeros(len(samples), dtype=bool)
    result = []
    for seed in range(len(samples)):
        if seen[seed]:
            continue
        seen[seed] = True
        pending, indices = [seed], []
        while pending:
            index = pending.pop()
            indices.append(index)
            neighbors = np.flatnonzero(adjacent[index] & ~seen)
            seen[neighbors] = True
            pending.extend(neighbors.tolist())
        if len(indices) >= 6:
            result.append(samples[indices])
    return result


def _fit_surface(samples, reference, ray, deproject):
    points = np.asarray([deproject(pixel[:2].tolist(), float(pixel[2])) for pixel in samples])
    if points.shape != (len(samples), 3) or not np.isfinite(points).all() or np.any(points[:, 2] <= 0):
        return None, {'reason': 'invalid_surface_deprojection'}
    # Robust orthogonal fitting. All samples belong to one spatially connected
    # depth surface. Reweighting does not fabricate or extend measured support.
    weights = np.ones(len(points))
    for _ in range(5):
        center = np.average(points, axis=0, weights=weights)
        _, _, vectors = np.linalg.svd((points-center)*np.sqrt(weights[:, None]), full_matrices=False)
        normal = vectors[-1]
        residual = (points-center) @ normal
        weights = np.minimum(1., .004/np.maximum(np.abs(residual), 1e-12))
    inliers = np.abs(residual) <= .012
    diagnostics = {'surface_patch_count': len(samples), 'surface_inlier_count': int(inliers.sum())}
    if inliers.mean() < .8 or inliers.sum() < 6:
        return None, dict(diagnostics, reason='nonplanar_wrist_surface')
    # Recheck support AFTER robust rejection; a plane must not extrapolate from
    # only one side of a missing wrist region.
    if not _surrounds(samples[inliers, :2], reference):
        return None, dict(diagnostics, reason='unsupported_wrist_ray')
    points = points[inliers]
    center = points.mean(axis=0)
    _, singular, vectors = np.linalg.svd(points-center, full_matrices=False)
    coverage = float(singular[1]/np.sqrt(len(points)))
    if coverage < .003:
        return None, dict(diagnostics, reason='degenerate_wrist_surface', coverage_m=coverage)
    normal = vectors[-1]
    incidence = float(abs(normal @ ray)/np.linalg.norm(ray))
    if incidence < .2:
        return None, dict(diagnostics, reason='grazing_wrist_surface', incidence=incidence)
    distance = float((normal @ center)/(normal @ ray))
    if not np.isfinite(distance) or distance <= 0:
        return None, dict(diagnostics, reason='invalid_surface_intersection')
    position = ray*distance
    residual = (points-center) @ normal
    rms = float(np.sqrt(np.mean(residual**2)))
    # Locality guards against a spurious steep plane extending through the ray.
    if position[2] < points[:, 2].min()-.012 or position[2] > points[:, 2].max()+.012 or np.max(abs(residual)) > .012:
        return None, dict(diagnostics, reason='inconsistent_wrist_surface')
    return position, dict(diagnostics, reason='wrist_surface_fit_observed',
                          residual_m=rms, coverage_m=coverage, incidence=incidence,
                          surface_normal=normal.tolist(), support_pixels=samples[inliers, :2].tolist())


def _local_depth_support(depth, support_mask, reference):
    """Current near-ray values used only to veto gaps or conflicting planes."""
    height, width = depth.shape
    x0, y0 = np.ceil(reference-MAX_WRIST_DEPTH_GAP_PX).astype(int)
    x1, y1 = np.floor(reference+MAX_WRIST_DEPTH_GAP_PX).astype(int)
    xx, yy = np.meshgrid(np.arange(max(0, x0), min(width, x1+1)),
                         np.arange(max(0, y0), min(height, y1+1)))
    pixels = np.column_stack((xx.ravel(), yy.ravel()))
    z = depth[pixels[:, 1], pixels[:, 0]]
    supported = (np.linalg.norm(pixels-reference, axis=1) <= MAX_WRIST_DEPTH_GAP_PX)
    supported &= support_mask[pixels[:, 1], pixels[:, 0]]
    supported &= np.isfinite(z) & (z >= SURFACE_DEPTH_RANGE_M[0]) & (z <= SURFACE_DEPTH_RANGE_M[1])
    return np.column_stack((pixels[supported], z[supported]))


def _check_local_support(position, details, local_samples, reference, deproject):
    if not len(local_samples):
        return dict(reason='wrist_depth_gap_too_large', local_depth_count=0)
    support_pixels = np.asarray(details['support_pixels'])
    nearest_patch = float(np.min(np.linalg.norm(support_pixels-reference, axis=1)))
    points = np.asarray([deproject(sample[:2].tolist(), float(sample[2])) for sample in local_samples])
    if points.shape != (len(local_samples), 3) or not np.isfinite(points).all() or np.any(points[:, 2] <= 0):
        return dict(reason='invalid_local_deprojection')
    residual = np.abs((points-position) @ np.asarray(details['surface_normal']))
    diagnostics = dict(local_depth_count=len(local_samples),
                       nearest_support_patch_px=nearest_patch,
                       max_local_depth_residual_m=float(residual.max()))
    # Contradicting near-ray depth cannot be discarded as a robust-fit outlier.
    # This conservative veto does not promote isolated depth to positive evidence;
    # the coherent connected patches still supply that evidence.
    if residual.max() > .012:
        return dict(diagnostics, reason='contradictory_wrist_depth')
    # A lone valid pixel inside a larger hole must not satisfy the near-ray gap
    # condition. At least one near-ray value must itself belong to a coherent
    # accepted 5x5 patch; its depth already passed the plane consistency veto.
    covered = np.any(np.all(np.abs(local_samples[:, None, :2]-support_pixels[None, :, :]) <= 2,
                            axis=2), axis=1)
    diagnostics['local_depth_in_support_patch_count'] = int(covered.sum())
    if not covered.any():
        return dict(diagnostics, reason='wrist_depth_gap_too_large')
    if nearest_patch > MAX_WRIST_PATCH_DISTANCE_PX:
        return dict(diagnostics, reason='wrist_patch_gap_too_large')
    return diagnostics


def wrist_surface_fit(pixels, depth_m, deproject, *, support_mask=None):
    """Return a measured-surface estimate and diagnostics, or None on rejection.

    The SDK owns deprojection/distortion. All 21 upstream landmarks are retained;
    landmark 0 defines the fixed semantic ray and 5/17 size the local search area.
    Patch quality and plane inliers retain 80% support and 15/12mm thresholds.
    Three bounded radii search for surrounded, noncollinear *current* support.
    No whole-hand model dimensions, previous position, or predicted depth enters.
    A supplied support mask participates in each 5x5 patch's 80% support test.
    None is for standalone callers that already supply an associated depth region.
    A small surrounded depth hole may be interpolated, but unbounded missing depth
    or a contradictory current foreground sample cannot be filled by a background
    ring. The reference remains a visible surface, never an occluded bone joint.
    """
    uv = finite_array(pixels, (21, 2), 'all image landmarks')
    depth = np.asarray(depth_m, dtype=float)
    if depth.ndim != 2:
        raise ValueError('Depth must be a registered 2D image')
    mask = np.ones(depth.shape, dtype=bool) if support_mask is None else np.asarray(support_mask, dtype=bool)
    if mask.shape != depth.shape:
        raise ValueError('Support mask must match the registered depth image')
    reference = uv[0]
    metadata = dict(surface_fit_version=SURFACE_FIT_VERSION,
                    surface_depth_range_m=list(SURFACE_DEPTH_RANGE_M),
                    position_reference='wrist_surface_ray',
                    surface_source='current_registered_depth',
                    max_wrist_depth_gap_px=MAX_WRIST_DEPTH_GAP_PX,
                    max_wrist_patch_distance_px=MAX_WRIST_PATCH_DISTANCE_PX,
                    reference_pixel=reference.tolist())
    h, w = depth.shape
    if not (0 <= reference[0] < w and 0 <= reference[1] < h):
        return None, dict(metadata, reason='wrist_outside_image')
    ray = finite_array(deproject(reference.tolist(), 1.), (3,), 'wrist ray')
    if ray[2] <= 0:
        return None, dict(metadata, reason='invalid_wrist_ray')
    # Use the smallest well-supported neighborhood to limit curvature bias.
    scaled_radius = int(np.clip(np.linalg.norm(uv[5]-uv[17])*.25, 9, 24))
    radii = sorted(set([9, scaled_radius, 24]))
    local_samples = _local_depth_support(depth, mask, reference)
    attempts = []
    for radius in radii:
        samples, tested = _patch_samples(depth, mask, reference, radius)
        info = dict(radius_px=radius, tested_patches=tested, valid_patches=len(samples))
        if len(samples) < 6:
            attempts.append(dict(info, reason='insufficient_wrist_surface_support'))
            continue
        candidates = [s for s in _components(samples) if _surrounds(s[:, :2], reference)]
        if len(candidates) != 1:
            attempts.append(dict(info, reason='ambiguous_wrist_surface' if candidates else 'unsupported_wrist_ray'))
            continue
        position, details = _fit_surface(candidates[0], reference, ray, deproject)
        if position is not None:
            local_details = _check_local_support(position, details, local_samples, reference, deproject)
            details.update(local_details)
            if 'reason' in local_details:
                position = None
        attempts.append(dict(info, **details))
        if position is not None:
            return position, dict(metadata, **attempts[-1], surface_attempts=attempts)
    return None, dict(metadata, reason=attempts[-1]['reason'], surface_attempts=attempts)
