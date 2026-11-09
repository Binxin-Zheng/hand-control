"""Read-only RealSense recording helpers; no device creation or configuration.

Native depth and both infrared images diagnose stereo versus color-registration
holes. They never replace a hand observation or fill missing depth.
"""
import math


def video_geometry(profile):
    intrinsics = profile.as_video_stream_profile().intrinsics
    return dict(width=intrinsics.width, height=intrinsics.height,
                fx=intrinsics.fx, fy=intrinsics.fy, ppx=intrinsics.ppx, ppy=intrinsics.ppy,
                distortion=str(intrinsics.model), coeffs=list(intrinsics.coeffs),
                fps=profile.fps(), format=str(profile.format()), stream_index=profile.stream_index())


def extrinsics_to_color(profile, color_profile):
    extrinsics = profile.get_extrinsics_to(color_profile)
    return dict(rotation_column_major=list(extrinsics.rotation),
                translation_m=list(extrinsics.translation),
                convention='p_color = R_color_source @ p_source + t_color_source')


def frame_description(frame, rs):
    if not frame:
        return dict(available=False, reason='stream_missing')
    description = dict(available=True, frame_number=frame.get_frame_number(),
                       timestamp_s=frame.get_timestamp()/1000,
                       timestamp_domain=str(frame.get_frame_timestamp_domain()), metadata={})
    # Actual FRAME metadata, not non-streaming sensor option readback. SDK units:
    # https://github.com/realsenseai/librealsense/blob/master/include/librealsense2/h/rs_frame.h
    for name, unit in (('actual_exposure', 'microseconds'), ('gain_level', 'SDK gain units'),
                       ('actual_fps', 'frames/s times 1000'), ('auto_exposure', 'zero means off'),
                       ('frame_laser_power', 'SDK laser-power units'),
                       ('frame_emitter_mode', 'SDK emitter-mode enum')):
        key = getattr(rs.frame_metadata_value, name, None)
        if key is None:
            description['metadata'][name] = dict(available=False)
            continue
        try:
            if not frame.supports_frame_metadata(key):
                description['metadata'][name] = dict(available=False)
                continue
            value = frame.get_frame_metadata(key)
            item = dict(available=True, sdk_raw_value=int(value), sdk_raw_unit=unit)
            if name == 'actual_exposure':
                item['value_s'] = value/1_000_000
            elif name == 'actual_fps':
                item['value_hz'] = value/1000
            description['metadata'][name] = item
        except RuntimeError as error:
            description['metadata'][name] = dict(available=False, error=str(error))
    return description


def capture_stereo_diagnostics(frames, color, depth_scale, rs):
    """Copy native arrays and their calibration from this same frameset only."""
    import numpy as np
    if not math.isfinite(depth_scale) or depth_scale <= 0:
        raise ValueError('Invalid native depth scale')
    streams = {'native_depth_z16': frames.get_depth_frame(),
               'infrared_1': frames.get_infrared_frame(1),
               'infrared_2': frames.get_infrared_frame(2)}
    arrays = {}
    metadata = dict(schema='teleop.stereo_diagnostics.v1', depth_scale_m_per_unit=depth_scale,
                    color=frame_description(color, rs), streams={},
                    meaning='Diagnostic source images only; not additional observed hand positions')
    if color:
        metadata['color']['geometry'] = video_geometry(color.profile)
    for name, frame in streams.items():
        description = frame_description(frame, rs)
        if frame:
            if name == 'native_depth_z16' and frame.profile.format() != rs.format.z16:
                raise ValueError('Native diagnostic depth must be Z16; do not label disparity as metric depth')
            arrays[name] = np.asanyarray(frame.get_data()).copy()
            description['geometry'] = video_geometry(frame.profile)
            if color:
                description['extrinsics_to_color'] = extrinsics_to_color(frame.profile, color.profile)
        metadata['streams'][name] = description
    depth = metadata['streams']['native_depth_z16']
    # Preserve skew and clock domains rather than claiming all images coincide.
    for description in metadata['streams'].values():
        comparable = (depth['available'] and description['available']
                      and depth['timestamp_domain'] == description['timestamp_domain'])
        description['skew_to_native_depth_s'] = (
            description['timestamp_s']-depth['timestamp_s'] if comparable else None)
    metadata['all_streams_present'] = all(item['available'] for item in metadata['streams'].values())
    return arrays, metadata
