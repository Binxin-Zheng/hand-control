"""Opt-in RealSense RGB-D worker; unchanged audited dex detector, no motor imports.

Live device needs global/system capture timestamps. SDK deprojection preserves
intrinsics/distortion. Bag replay is SIMULATION ONLY and rebases capture time.
All acquired frames and failures are recorded. Source frame-number gaps remain
visible in logs; processing throughput does not guarantee lossless sensor capture.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import socket
import sys
import time
import uuid
from dataclasses import replace
import xml.etree.ElementTree as ET


def publish_then_record(packet, destination, log, *, record_frame=None, sock=None, port=None):
    """Deliver a fresh observation before potentially slow diagnostic recording."""
    from anyteleop_so101.runtime.transport import publish_file
    started = time.monotonic()
    encoded = json.dumps(packet, allow_nan=False)
    try:
        publish_file(destination, packet)
        if sock is not None:
            sock.sendto(encoded.encode(), ('127.0.0.1', port))
    finally:
        # Preserve this acquired attempt even if publication fails.
        published = time.monotonic()
        log.write(encoded+'\n'); log.flush()
        logged = time.monotonic()
        if record_frame is not None:
            record_frame()
    return dict(publication_s=published-started, frame_log_s=logged-published,
                image_record_s=time.monotonic()-logged)


def close_resources(actions, original_error=None):
    """Close every resource, preserving the acquisition error that caused exit."""
    errors = []
    for name, close in actions:
        try:
            close()
        except Exception as error:
            errors.append((name, error))
    if errors:
        if original_error is not None:
            for name, error in errors:
                print(f'Cleanup warning ({name}): {error}', file=sys.stderr, flush=True)
        else:
            raise RuntimeError('Vision cleanup failed: '+', '.join(name for name, _ in errors)) from errors[0][1]


def run(args, should_stop=lambda: False):
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    record_stereo = getattr(args, 'record_stereo', False)
    if record_stereo and not args.record_images:
        raise ValueError('Stereo diagnostics require raw image recording')
    from anyteleop_so101.runtime.rgbd import fuse_frame, status_lines
    from anyteleop_so101.surface_position import SURFACE_FIT_VERSION, wrist_surface_support_mask
    from anyteleop_so101 import surface_position
    surface_code_hash = hashlib.sha256(Path(surface_position.__file__).read_bytes()).hexdigest()
    gripper_limits = None
    if args.closed_rad is None:
        from anyteleop_so101.runtime.model import validate_model
        model_path = validate_model(args.gripper_urdf)
        limit = ET.parse(model_path).getroot().find("joint[@name='gripper']/limit")
        gripper_limits = (float(limit.get('lower')), float(limit.get('upper')))
        if gripper_limits[1] <= gripper_limits[0]:
            raise ValueError('Invalid model gripper limits')
    path = args.detector_repo / 'example/vector_retargeting/single_hand_detector.py'
    data = path.read_bytes()
    blob = hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
    if blob != '4867ad7d2e5ea3984002c1730b937d06b1fe97ee':
        raise ValueError('Detector differs from audited baseline; review and validate before changing the pin')
    spec = importlib.util.spec_from_file_location('upstream_detector', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    detector = module.SingleHandDetector(hand_type=args.hand, selfie=False)
    pipeline = rs.pipeline(); config = rs.config()
    if args.bag:
        rs.config.enable_device_from_file(config, str(args.bag), repeat_playback=False)
    else:
        if args.serial: config.enable_device(args.serial)
        depth_width, depth_height = map(int, args.depth_size.split('x'))
        config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        if record_stereo:
            for index in (1, 2):
                config.enable_stream(rs.stream.infrared, index, depth_width, depth_height, rs.format.y8, 30)
    sock = None if args.port is None else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    started = False
    args.out.mkdir(parents=True, exist_ok=True)
    identity = f'{args.hand}:{uuid.uuid4()}'
    frame_id = 'camera_color_optical'
    epoch = 0; was_detected = False
    clock_offset = time.monotonic()-time.time()
    first_sensor_time = None; replay_start = None
    previous_frame = None
    last_status_time = 0.
    clock_ready = False
    try:
        if not args.bag:
            # Set timestamp mode before starting, rather than switching a running stream.
            resolved_device = config.resolve(rs.pipeline_wrapper(pipeline)).get_device()
            for sensor in resolved_device.query_sensors():
                if sensor.supports(rs.option.global_time_enabled):
                    sensor.set_option(rs.option.global_time_enabled, 1)
        profile = pipeline.start(config); started = True
        clock_sync_deadline = time.monotonic()+3.
        device = profile.get_device()
        serial = device.get_info(rs.camera_info.serial_number)
        if args.bag:
            device.as_playback().set_real_time(False)
        print(f"Camera: {device.get_info(rs.camera_info.name)} | serial: {serial}", flush=True)
        depth_scale = device.first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        with (args.out/'frames.jsonl').open('w') as log, (args.out/'timing.jsonl').open('w') as timing_log:
            sequence = 0
            while not should_stop() and (args.frames is None or sequence < args.frames):
                cycle_started = time.monotonic()
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=1000)
                except RuntimeError:
                    if args.bag and device.as_playback().current_status() == rs.playback_status.stopped:
                        break
                    raise  # Consumer ages out and pauses if camera disconnects.
                acquired = time.monotonic()
                native_frames = frames  # Retain original depth/IR before color registration.
                frames = align.process(frames)
                aligned = time.monotonic()
                color, depth = frames.get_color_frame(), frames.get_depth_frame()
                if not color or not depth:
                    log.write(json.dumps({'sequence': sequence, 'error': 'missing_color_or_depth'})+'\n')
                    sequence += 1
                    continue
                capture = color.get_timestamp()/1000
                depth_capture = depth.get_timestamp()/1000
                domain = color.get_frame_timestamp_domain()
                clock_error = None
                if domain != depth.get_frame_timestamp_domain() or abs(capture-depth_capture) > .02:
                    clock_error = 'RGB/depth capture clocks or timestamps are not aligned'
                elif not args.bag and domain not in (rs.timestamp_domain.global_time, rs.timestamp_domain.system_time):
                    clock_error = 'Capture clock is not host-synchronized; verify RealSense global time'
                if clock_error:
                    log.write(json.dumps({'sequence': sequence, 'error': clock_error,
                        'source_frame_number': color.get_frame_number(), 'capture_domain': str(domain)})+'\n')
                    log.flush()
                    if not args.bag and not clock_ready and time.monotonic() < clock_sync_deadline:
                        sequence += 1
                        continue  # Initial synchronization only; never auto-recover a running stream.
                    raise ValueError(clock_error)
                clock_ready = True
                if args.bag:
                    if first_sensor_time is None:
                        first_sensor_time, replay_start = capture, time.monotonic()
                    timestamp = replay_start + capture-first_sensor_time
                    time.sleep(max(0, timestamp-time.monotonic()))
                else:
                    if domain not in (rs.timestamp_domain.global_time, rs.timestamp_domain.system_time):
                        raise ValueError('Capture clock is not host-synchronized; verify RealSense global time')
                    if abs((time.monotonic()-time.time())-clock_offset) > .005:
                        raise ValueError('Host wall clock changed; restart requires explicit re-engagement')
                    timestamp = capture+clock_offset
                image = np.asanyarray(color.get_data())
                if color.profile.format() == rs.format.rgb8:
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                elif color.profile.format() != rs.format.bgr8:
                    raise ValueError('Expected RGB8 or BGR8 color stream')
                depth_m = np.asanyarray(depth.get_data()).astype(float)*depth_scale
                inference_started = time.monotonic()
                n, local, pixels, rotation = detector.detect(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
                inference_finished = time.monotonic()
                if n and not was_detected: epoch += 1
                was_detected = bool(n)
                uv = None if not n else detector.parse_keypoint_2d(pixels, image.shape[:2])
                row = dict(schema='upstream.probe.v2', sequence=sequence, detected=bool(n),
                    image_width=image.shape[1], image_height=image.shape[0],
                    local_points_mano=None if not n else local.tolist(),
                    model_wrist_rotation=None if not n else rotation.tolist(),
                    operator2mano=detector.operator2mano.tolist(), pixels=None if not n else uv.tolist())
                # Surface-support mask only. No claim that hidden anatomical joints are visible.
                mask = np.zeros(image.shape[:2], dtype=np.uint8)
                if n:
                    hull = cv2.convexHull(uv[[0, 5, 9, 13, 17]].astype(np.int32))
                    cv2.fillConvexPoly(mask, hull, 1)
                    mask = cv2.dilate(mask, np.ones((7, 7), np.uint8))
                intrinsics = depth.profile.as_video_stream_profile().intrinsics
                observation, diagnostics = fuse_frame(row, depth_m, mask.astype(bool),
                    lambda pixel, z: rs.rs2_deproject_pixel_to_point(intrinsics, pixel, z),
                    timestamp_s=timestamp, track_id=f'{identity}:{epoch}', frame_id=frame_id,
                    aperture_calibration=None if args.closed_rad is None else (args.closed_rad, args.open_rad),
                    position_method=args.position_method,
                    wrist_support_mask=None if not n or args.position_method != 'wrist_surface_fit' else
                        wrist_surface_support_mask(uv, depth_m.shape))
                if gripper_limits is not None and diagnostics['aperture_angle_rad'] is not None:
                    lower, upper = gripper_limits
                    joint_angle = float(np.clip(diagnostics['aperture_angle_rad'], lower, upper))
                    observation = replace(observation, aperture=(joint_angle-lower)/(upper-lower), aperture_evidence='rgb_model')
                diagnostics['gripper_mapping'] = 'direct_joint_angle' if gripper_limits is not None else 'calibrated_range'
                fusion_finished = time.monotonic()
                geometry = dict(serial=serial, width=intrinsics.width, height=intrinsics.height,
                    fx=intrinsics.fx, fy=intrinsics.fy, ppx=intrinsics.ppx, ppy=intrinsics.ppy,
                    distortion=str(intrinsics.model), coeffs=list(intrinsics.coeffs), depth_scale=depth_scale,
                    detector_blob=blob, selected_hand=args.hand,
                    aperture_calibration=None if args.closed_rad is None else [args.closed_rad, args.open_rad],
                    visibility_method='palm_hull_surface_assumption')
                if args.position_method != 'model_translation':
                    geometry['position_method'] = args.position_method
                if args.position_method == 'wrist_surface_fit':
                    geometry['surface_fit_version'] = SURFACE_FIT_VERSION
                    geometry['surface_estimator_sha256'] = surface_code_hash
                    geometry['position_reference'] = 'wrist_surface_ray'
                    geometry['wrist_surface_association'] = 'landmark_wrist_roi_not_segmentation'
                if gripper_limits is not None:
                    geometry['gripper_mapping'] = dict(method='direct_joint_angle', joint_limits_rad=gripper_limits)
                geometry['depth_stream'] = dict(width=profile.get_stream(rs.stream.depth).as_video_stream_profile().width(),
                                                height=profile.get_stream(rs.stream.depth).as_video_stream_profile().height())
                if record_stereo:
                    geometry['diagnostic_streams'] = ['native_depth_z16', 'infrared_1', 'infrared_2']
                geometry_hash = hashlib.sha256(json.dumps(geometry, sort_keys=True).encode()).hexdigest()
                acceptance = {} if args.geometry_acceptance is None else json.loads(args.geometry_acceptance.read_text())
                validated = bool(not args.bag and acceptance.get('geometry_sha256') == geometry_hash
                                 and acceptance.get('depth_visibility_and_clock_validated') is True)
                number = color.get_frame_number()
                packet = dict(schema='teleop.vision.v1', observation=observation.as_dict(),
                    geometry_validated=validated, geometry_sha256=geometry_hash, geometry=geometry,
                    diagnostics=diagnostics, probe=row, capture_domain=str(domain),
                    source_frame_number=number,
                    source_frame_gap=None if previous_frame is None else number-previous_frame-1,
                    source='bag_replay' if args.bag else 'live_rgbd')
                previous_frame = number
                ready = time.monotonic()
                timings = dict(sequence=sequence, timestamp_s=timestamp,
                    capture_age_at_publish_start_s=ready-timestamp,
                    acquire_wait_s=acquired-cycle_started, alignment_s=aligned-acquired,
                    inference_s=inference_finished-inference_started,
                    fusion_and_adaptation_s=fusion_finished-inference_finished,
                    packet_preparation_s=ready-fusion_finished)
                def record_frame():
                    # Atomic installation leaves an interrupted archive visibly missing,
                    # rather than presenting a partial ZIP as a successful raw frame.
                    destination = args.out/f'{sequence:06d}.npz'
                    temporary = destination.with_suffix('.npz.tmp')
                    try:
                        values = dict(color_bgr=image, depth_m=depth_m, timestamp_s=timestamp)
                        if record_stereo:
                            from anyteleop_so101.runtime.stereo_diagnostics import capture_stereo_diagnostics
                            native_arrays, native_metadata = capture_stereo_diagnostics(
                                native_frames, color, depth_scale, rs)
                            native_metadata.update(sequence=sequence, color_capture_monotonic_s=timestamp)
                            values.update(native_arrays)
                            values['stereo_metadata_json'] = np.asarray(json.dumps(native_metadata, allow_nan=False))
                        with temporary.open('wb') as raw:
                            np.savez_compressed(raw, **values)
                        temporary.replace(destination)
                    finally:
                        temporary.unlink(missing_ok=True)
                try:
                    timings.update(publish_then_record(packet, args.vision_file, log,
                        record_frame=record_frame if args.record_images else None, sock=sock, port=args.port))
                except BaseException as error:
                    timings['delivery_or_recording_error'] = type(error).__name__+': '+str(error)
                    timing_log.write(json.dumps(timings, allow_nan=False)+'\n'); timing_log.flush()
                    raise
                now = time.monotonic()
                lines = status_lines(observation, diagnostics, detected=bool(n), hand=args.hand, now_s=now)
                if now-last_status_time >= 1:
                    print(' | '.join(lines), flush=True)
                    last_status_time = now
                quit_requested = False
                if args.display:
                    preview = image.copy()
                    if n: detector.draw_skeleton_on_image(preview, pixels)
                    # Fixed 0..1.5 m color scale for diagnostic display only; no depth clipping in fusion.
                    depth_u8 = np.clip(np.nan_to_num(depth_m)*255/1.5, 0, 255).astype(np.uint8)
                    depth_view = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
                    depth_view[~np.isfinite(depth_m) | (depth_m <= 0)] = 0
                    if args.mirror_preview:
                        preview = cv2.flip(preview, 1)
                        depth_view = cv2.flip(depth_view, 1)
                    view = np.hstack([preview, depth_view])
                    panel = np.zeros((140, view.shape[1], 3), dtype=np.uint8)
                    for index, line in enumerate(lines):
                        cv2.putText(panel, line, (12, 24+index*25), cv2.FONT_HERSHEY_SIMPLEX,
                                    .55, (220, 240, 220), 1, cv2.LINE_AA)
                    cv2.imshow('RGB-D hand + aligned depth'+(' (mirror display only)' if args.mirror_preview else ' (unmirrored)'), np.vstack([view, panel]))
                    quit_requested = cv2.waitKey(1) & 0xff in (27, ord('q'))
                timings['preview_and_status_s'] = time.monotonic()-now
                timings['cycle_s'] = time.monotonic()-cycle_started
                timing_log.write(json.dumps(timings, allow_nan=False)+'\n'); timing_log.flush()
                sequence += 1
                if quit_requested: break
    finally:
        actions = [('camera', pipeline.stop)] if started else []
        actions.append(('detector', detector.hand_detector.close))
        if sock is not None: actions.append(('socket', sock.close))
        if args.display: actions.append(('preview', cv2.destroyAllWindows))
        close_resources(actions, original_error=sys.exc_info()[1])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--detector-repo', type=Path, required=True)
    p.add_argument('--serial'); p.add_argument('--bag', type=Path)
    p.add_argument('--depth-size', choices=['640x480', '640x360', '480x270', '424x240'], default='640x480',
                   help='Native depth profile; lower resolutions permit nearer D455 depth. Color/detector remain 640x480.')
    p.add_argument('--hand', choices=['Right', 'Left'], default='Right')
    p.add_argument('--position-method', choices=['model_translation', 'surface_wrist', 'wrist_surface_fit'], default='model_translation',
                   help='Explicit alternative surface reference; original model-translation baseline remains default')
    p.add_argument('--closed-rad', type=float); p.add_argument('--open-rad', type=float)
    p.add_argument('--port', type=int, help='Optional localhost UDP output')
    p.add_argument('--vision-file', type=Path, default=Path('outputs/runtime/vision.json'))
    p.add_argument('--out', type=Path, default=Path('outputs/rgbd'))
    p.add_argument('--frames', type=int); p.add_argument('--display', action='store_true')
    p.add_argument('--record-images', action='store_true')
    p.add_argument('--record-stereo', action='store_true',
                   help='Also record native depth, both IR views, calibration and actual frame metadata; implies --record-images')
    p.add_argument('--mirror-preview', action='store_true', help='Mirror display only; inference and geometry remain unmirrored')
    p.add_argument('--aperture-calibration', type=Path, help='Optional legacy closed/open angles; default follows joint angle without calibration')
    p.add_argument('--gripper-urdf', type=Path, default=Path('assets/so101/so101_new_calib.urdf'),
                   help='Default automatic angle mapping uses this model gripper joint; no hand calibration needed')
    p.add_argument('--geometry-acceptance', type=Path)
    args = p.parse_args()
    if args.record_stereo:
        args.record_images = True
    import math
    if args.aperture_calibration:
        if args.closed_rad is not None or args.open_rad is not None:
            p.error('Choose saved calibration or explicit angles, not both')
        calibration = json.loads(args.aperture_calibration.read_text())
        if calibration.get('hand') != args.hand:
            p.error('Saved calibration is for a different selected hand')
        args.closed_rad, args.open_rad = calibration['closed_rad'], calibration['open_rad']
    if (args.closed_rad is None) != (args.open_rad is None):
        p.error('Supply both optional angles or neither (default: automatic joint-angle mapping)')
    if args.closed_rad is not None and (not all(math.isfinite(x) for x in (args.closed_rad, args.open_rad)) or args.open_rad-args.closed_rad < .1):
        p.error('Provide measured closed/open angles separated by at least 0.1 radians')
    # Finish the acquired frame and its archive before ordinary launcher shutdown.
    # The launcher's bounded TERM/KILL fallback still handles an unresponsive SDK.
    import signal
    stopping = False
    def request_stop(signum, frame):
        global stopping
        stopping = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        run(args, should_stop=lambda: stopping)
    except KeyboardInterrupt:
        pass  # run() closes the camera and detector in its finally block.
