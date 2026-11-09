"""Native diagnostic units, calibration and missing data without any device."""
from types import SimpleNamespace

import numpy as np
import pytest

from anyteleop_so101.runtime.stereo_diagnostics import capture_stereo_diagnostics


RS = SimpleNamespace(format=SimpleNamespace(z16='z16'),
    frame_metadata_value=SimpleNamespace(actual_exposure='exposure', gain_level='gain', actual_fps='fps'))


class Profile:
    def __init__(self, format, index=0):
        self._format, self._index = format, index
        self.intrinsics = SimpleNamespace(width=3, height=2, fx=200., fy=201., ppx=1., ppy=.5,
                                         model='none', coeffs=[0.]*5)

    def as_video_stream_profile(self): return self
    def fps(self): return 30
    def format(self): return self._format
    def stream_index(self): return self._index

    def get_extrinsics_to(self, other):
        return SimpleNamespace(rotation=[0., 1., 0., -1., 0., 0., 0., 0., 1.],
                               translation=[.1, 0., 0.])


class Frame:
    def __init__(self, format, *, timestamp=1000., domain='global', index=0):
        self.profile = Profile(format, index)
        self.timestamp, self.domain = timestamp, domain
        self.data = np.arange(6, dtype=np.uint16 if format == 'z16' else np.uint8).reshape(2, 3)

    def get_frame_number(self): return 42
    def get_timestamp(self): return self.timestamp
    def get_frame_timestamp_domain(self): return self.domain
    def supports_frame_metadata(self, key): return key != 'gain'
    def get_frame_metadata(self, key): return {'exposure': 33000, 'fps': 30000}[key]
    def get_data(self): return self.data


def frameset(depth, ir1, ir2):
    return SimpleNamespace(get_depth_frame=lambda: depth,
                           get_infrared_frame=lambda index: {1: ir1, 2: ir2}[index])


def test_native_arrays_are_unmirrored_copies_with_explicit_units_and_calibration():
    depth, ir1, ir2, color = Frame('z16'), Frame('y8', index=1), Frame('y8', index=2), Frame('bgr8')
    arrays, meta = capture_stereo_diagnostics(frameset(depth, ir1, ir2), color, .001, RS)
    assert meta['all_streams_present']
    np.testing.assert_array_equal(arrays['native_depth_z16'], [[0, 1, 2], [3, 4, 5]])
    np.testing.assert_array_equal(arrays['infrared_1'], ir1.data)
    depth.data[:] = 99
    assert arrays['native_depth_z16'][0, 0] == 0  # SDK pooled buffers cannot mutate the recording.
    assert meta['depth_scale_m_per_unit'] == .001
    info = meta['streams']['native_depth_z16']
    assert info['timestamp_s'] == 1.
    assert info['metadata']['actual_exposure']['value_s'] == .033
    assert info['metadata']['actual_fps']['value_hz'] == 30.
    assert info['metadata']['gain_level'] == {'available': False}
    transform = info['extrinsics_to_color']
    rotation = np.asarray(transform['rotation_column_major']).reshape((3, 3), order='F')
    np.testing.assert_allclose(rotation @ [1., 0., 0.]+transform['translation_m'], [.1, 1., 0.])


def test_missing_ir_and_different_clock_domain_do_not_fabricate_synchronized_images():
    depth, ir1, color = Frame('z16'), Frame('y8', timestamp=2000., domain='hardware'), Frame('bgr8')
    arrays, meta = capture_stereo_diagnostics(frameset(depth, ir1, None), color, .001, RS)
    assert not meta['all_streams_present']
    assert 'infrared_2' not in arrays
    assert meta['streams']['infrared_2']['reason'] == 'stream_missing'
    assert meta['streams']['infrared_1']['skew_to_native_depth_s'] is None
    assert meta['streams']['infrared_1']['timestamp_s'] == 2.
    assert meta['streams']['infrared_1']['timestamp_domain'] == 'hardware'


def test_disparity_data_is_not_mislabeled_native_z16():
    with pytest.raises(ValueError, match='must be Z16'):
        capture_stereo_diagnostics(frameset(Frame('disparity16'), None, None), Frame('bgr8'), .001, RS)
