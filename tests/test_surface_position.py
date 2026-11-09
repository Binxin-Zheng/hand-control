import numpy as np
import pytest

from anyteleop_so101.perception import RectifiedIntrinsics
from anyteleop_so101.surface_position import wrist_surface_fit, wrist_surface_support_mask
from anyteleop_so101.runtime.rgbd import fuse_frame


K = RectifiedIntrinsics(400, 400, 320, 240, 640, 480)


def surface(reference=(0., 0., .5), normal=(0., 0., 1.)):
    reference = np.asarray(reference)
    normal = np.asarray(normal, dtype=float)
    yy, xx = np.mgrid[:480, :640]
    rays = np.stack(((xx-320)/400, (yy-240)/400, np.ones_like(xx)), axis=-1)
    depth = (normal @ reference)/(rays @ normal)
    pixels = np.tile(reference[:2]/reference[2]*400+[320, 240], (21, 1))
    pixels[5] += [40, 40]
    pixels[17] += [-40, 40]
    return pixels, depth


@pytest.mark.parametrize('axis', range(3))
@pytest.mark.parametrize('sign', [-1, 1])
def test_recovers_six_metric_displacement_directions(axis, sign):
    p0 = np.array([.02, -.03, .5])
    p1 = p0.copy(); p1[axis] += sign*.02
    estimates = []
    for reference in [p0, p1]:
        pixels, depth = surface(reference, (.2, -.1, 1.))
        position, info = wrist_surface_fit(pixels, depth, K.deproject)
        assert info['surface_source'] == 'current_registered_depth'
        np.testing.assert_allclose(position, reference, atol=2e-5)
        estimates.append(position)
    np.testing.assert_allclose(estimates[1]-estimates[0], p1-p0, atol=2e-5)


def test_surface_rotation_about_fixed_wrist_does_not_translate_reference():
    reference = [.02, .01, .55]
    for normal in [(0, 0, 1), (.6, -.2, 1), (-.5, .4, 1)]:
        pixels, depth = surface(reference, normal)
        position, _ = wrist_surface_fit(pixels, depth, K.deproject)
        np.testing.assert_allclose(position, reference, atol=3e-5)


def test_current_surrounding_depth_supports_a_small_wrist_hole():
    pixels, depth = surface()
    depth[237:244, 317:324] = 0
    position, info = wrist_surface_fit(pixels, depth, K.deproject)
    np.testing.assert_allclose(position, [0, 0, .5], atol=1e-10)
    assert info['surface_inlier_count'] >= 6
    assert info['coverage_m'] >= .003


def test_nearby_one_sided_depth_cannot_fill_unobserved_wrist():
    pixels, depth = surface()
    depth[:, 318:] = 0
    position, info = wrist_surface_fit(pixels, depth, K.deproject)
    assert position is None
    assert info['reason'] == 'unsupported_wrist_ray'


def test_depth_discontinuity_through_reference_is_rejected():
    pixels, depth = surface()
    depth[:, 320:] = .8
    position, _ = wrist_surface_fit(pixels, depth, K.deproject)
    assert position is None


@pytest.mark.parametrize('bad_depth', [0., np.nan, 65.535])
def test_missing_or_impossible_depth_never_reuses_previous_position(bad_depth):
    pixels, depth = surface()
    assert wrist_surface_fit(pixels, depth, K.deproject)[0] is not None
    position, info = wrist_surface_fit(pixels, np.full_like(depth, bad_depth), K.deproject)
    assert position is None
    assert info['reason'] == 'insufficient_wrist_surface_support'


def test_disconnected_background_does_not_move_foreground_wrist():
    pixels, depth = surface()
    depth[:225] = .9
    depth[255:] = .9
    position, _ = wrist_surface_fit(pixels, depth, K.deproject)
    np.testing.assert_allclose(position, [0, 0, .5], atol=1e-10)


@pytest.mark.parametrize('foreground_radius', [2, 3, 4, 5, 6, 7])
def test_background_ring_cannot_overrule_current_foreground_at_wrist(foreground_radius):
    pixels, depth = surface(reference=(0, 0, .9))
    yy, xx = np.indices(depth.shape)
    depth[(xx-320)**2+(yy-240)**2 <= foreground_radius**2] = .5
    position, info = wrist_surface_fit(pixels, depth, K.deproject)
    assert position is None
    assert any(attempt['reason'] in ('contradictory_wrist_depth', 'wrist_patch_gap_too_large')
               for attempt in info['surface_attempts'])


def test_near_center_foreground_still_vetoes_background_when_center_is_missing():
    pixels, depth = surface(reference=(0, 0, .9))
    depth[237:244, 317:324] = .5
    depth[240, 320] = 0
    position, info = wrist_surface_fit(pixels, depth, K.deproject)
    assert position is None
    assert any(attempt['reason'] == 'contradictory_wrist_depth' for attempt in info['surface_attempts'])


def test_surrounded_large_depth_hole_is_not_observed():
    pixels, depth = surface()
    depth[232:249, 312:329] = 0
    position, info = wrist_surface_fit(pixels, depth, K.deproject)
    assert position is None
    assert info['reason'] == 'wrist_depth_gap_too_large'


def test_isolated_center_depth_cannot_bridge_large_unsupported_hole():
    pixels, depth = surface()
    depth[232:249, 312:329] = 0
    depth[240, 320] = .5
    position, info = wrist_surface_fit(pixels, depth, K.deproject)
    assert position is None
    assert info['reason'] == 'wrist_depth_gap_too_large'


def test_isolated_center_cannot_bridge_hole_between_coherent_nearby_patches():
    pixels, depth = surface()
    yy, xx = np.indices(depth.shape)
    depth[(xx-320)**2+(yy-240)**2 <= 4**2] = 0
    depth[240, 320] = .5
    position, info = wrist_surface_fit(pixels, depth, K.deproject)
    assert position is None
    assert info['reason'] == 'wrist_depth_gap_too_large'
    assert any(attempt.get('local_depth_in_support_patch_count') == 0
               for attempt in info['surface_attempts'])


def test_mask_participates_in_each_patch_support_fraction():
    pixels, depth = surface()
    mask = np.ones(depth.shape, dtype=bool)
    mask[:, ::2] = False  # No 5x5 patch has the required 80% associated pixels.
    position, info = wrist_surface_fit(pixels, depth, K.deproject, support_mask=mask)
    assert position is None
    assert info['reason'] == 'insufficient_wrist_surface_support'


def test_excluded_foreground_cannot_be_filled_from_distant_background_ring():
    pixels, depth = surface(reference=(0, 0, .9))
    mask = np.ones(depth.shape, dtype=bool)
    mask[232:249, 312:329] = False
    position, info = wrist_surface_fit(pixels, depth, K.deproject, support_mask=mask)
    assert position is None
    assert info['reason'] == 'wrist_depth_gap_too_large'


def test_geometric_wrist_mask_is_bounded_and_does_not_claim_visibility():
    pixels, depth = surface()
    mask = wrist_surface_support_mask(pixels, depth.shape)
    assert mask.dtype == bool
    assert mask.sum() == 49*49
    assert mask[216:265, 296:345].all()
    np.testing.assert_allclose(wrist_surface_fit(pixels, depth, K.deproject, support_mask=mask)[0],
                               [0, 0, .5], atol=1e-10)
    pixels[0] = [-1, 240]
    assert not wrist_surface_support_mask(pixels, depth.shape).any()


def test_support_mask_shape_is_explicit():
    pixels, depth = surface()
    with pytest.raises(ValueError, match='Support mask'):
        wrist_surface_fit(pixels, depth, K.deproject, support_mask=np.ones((1, 1), bool))


def test_runtime_requires_explicit_new_association_to_override_old_visibility_mask():
    from test_runtime_rgbd import frame
    row, k = frame()
    depth = np.full((480, 640), .6)
    excluded = np.zeros(depth.shape, dtype=bool)
    options = dict(timestamp_s=1., track_id='right', aperture_calibration=None,
                   position_method='wrist_surface_fit')
    observation, _ = fuse_frame(row, depth, excluded, k.deproject, **options)
    assert observation.position_evidence == 'unavailable'
    assert observation.pose is None
    observation, _ = fuse_frame(row, depth, excluded, k.deproject,
        wrist_support_mask=wrist_surface_support_mask(row['pixels'], depth.shape), **options)
    assert observation.position_evidence == 'rgbd_fused'
    np.testing.assert_allclose(observation.pose.p, k.deproject(row['pixels'][0], .6), atol=1e-10)


def test_adapter_keeps_surface_reference_and_model_orientation_explicit():
    from test_runtime_rgbd import frame
    row, k = frame()
    # Model dimensions cannot move this measured surface estimate.
    model = np.asarray(row['local_points_mano']); model[:, 2] = np.linspace(0, .2, 21)
    row['local_points_mano'] = model.tolist()
    obs, info = fuse_frame(row, np.full((480, 640), .6), np.ones((480, 640), bool), k.deproject,
                          timestamp_s=1., track_id='right', aperture_calibration=None,
                          position_method='wrist_surface_fit')
    np.testing.assert_allclose(obs.pose.p, k.deproject(row['pixels'][0], .6), atol=1e-10)
    assert obs.position_evidence == 'rgbd_fused'
    assert obs.orientation_evidence == 'rgb_model'
    assert info['position_reference'] == 'wrist_surface_ray'
    assert info['baseline_reason'] == 'inconsistent_palm_depth'
