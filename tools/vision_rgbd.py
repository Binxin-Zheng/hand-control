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
import time
import uuid


def run(args):
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    from anyteleop_so101.runtime.rgbd import fuse_frame
    from anyteleop_so101.runtime.transport import publish_file
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
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    sock = None if args.port is None else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    started = False
    args.out.mkdir(parents=True, exist_ok=True)
    identity = f'{args.hand}:{uuid.uuid4()}'
    frame_id = 'camera_color_optical'
    epoch = 0; was_detected = False
    clock_offset = time.monotonic()-time.time()
    first_sensor_time = None; replay_start = None
    previous_frame = None
    try:
        profile = pipeline.start(config); started = True
        device = profile.get_device()
        serial = device.get_info(rs.camera_info.serial_number)
        if args.bag:
            device.as_playback().set_real_time(False)
        else:
            for sensor in device.query_sensors():
                if sensor.supports(rs.option.global_time_enabled):
                    sensor.set_option(rs.option.global_time_enabled, 1)
        depth_scale = device.first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        with (args.out/'frames.jsonl').open('w') as log:
            sequence = 0
            while args.frames is None or sequence < args.frames:
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=1000)
                except RuntimeError:
                    if args.bag and device.as_playback().current_status() == rs.playback_status.stopped:
                        break
                    raise  # Consumer ages out and pauses if camera disconnects.
                frames = align.process(frames)
                color, depth = frames.get_color_frame(), frames.get_depth_frame()
                if not color or not depth:
                    log.write(json.dumps({'sequence': sequence, 'error': 'missing_color_or_depth'})+'\n')
                    sequence += 1
                    continue
                capture = color.get_timestamp()/1000
                depth_capture = depth.get_timestamp()/1000
                domain = color.get_frame_timestamp_domain()
                if domain != depth.get_frame_timestamp_domain() or abs(capture-depth_capture) > .02:
                    raise ValueError('RGB/depth capture clocks or timestamps are not aligned')
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
                n, local, pixels, rotation = detector.detect(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
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
                    aperture_calibration=None if args.closed_rad is None else (args.closed_rad, args.open_rad))
                geometry = dict(serial=serial, width=intrinsics.width, height=intrinsics.height,
                    fx=intrinsics.fx, fy=intrinsics.fy, ppx=intrinsics.ppx, ppy=intrinsics.ppy,
                    distortion=str(intrinsics.model), coeffs=list(intrinsics.coeffs), depth_scale=depth_scale,
                    detector_blob=blob, selected_hand=args.hand,
                    aperture_calibration=None if args.closed_rad is None else [args.closed_rad, args.open_rad],
                    visibility_method='palm_hull_surface_assumption')
                geometry_hash = hashlib.sha256(json.dumps(geometry, sort_keys=True).encode()).hexdigest()
                acceptance = {} if args.geometry_acceptance is None else json.loads(args.geometry_acceptance.read_text())
                validated = bool(not args.bag and acceptance.get('geometry_sha256') == geometry_hash
                                 and acceptance.get('signed_by') and acceptance.get('date')
                                 and acceptance.get('depth_visibility_and_clock_validated') is True)
                number = color.get_frame_number()
                packet = dict(schema='teleop.vision.v1', observation=observation.as_dict(),
                    geometry_validated=validated, geometry_sha256=geometry_hash, geometry=geometry,
                    diagnostics=diagnostics, probe=row, capture_domain=str(domain),
                    source_frame_number=number,
                    source_frame_gap=None if previous_frame is None else number-previous_frame-1,
                    source='bag_replay' if args.bag else 'live_rgbd')
                previous_frame = number
                encoded = json.dumps(packet, allow_nan=False)
                log.write(encoded+'\n'); log.flush()
                if args.record_images:
                    np.savez_compressed(args.out/f'{sequence:06d}.npz', color_bgr=image, depth_m=depth_m,
                                        timestamp_s=timestamp)
                publish_file(args.vision_file, packet)
                if sock is not None:
                    sock.sendto(encoded.encode(), ('127.0.0.1', args.port))
                if args.display:
                    if n: detector.draw_skeleton_on_image(image, pixels)
                    cv2.imshow('RGB-D hand (unmirrored)', image)
                    if cv2.waitKey(1) & 0xff in (27, ord('q')): break
                sequence += 1
    finally:
        if started: pipeline.stop()
        detector.hand_detector.close()
        if sock is not None: sock.close()
        if args.display: cv2.destroyAllWindows()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--detector-repo', type=Path, required=True)
    p.add_argument('--serial'); p.add_argument('--bag', type=Path)
    p.add_argument('--hand', choices=['Right', 'Left'], default='Right')
    p.add_argument('--closed-rad', type=float); p.add_argument('--open-rad', type=float)
    p.add_argument('--port', type=int, help='Optional localhost UDP output')
    p.add_argument('--vision-file', type=Path, default=Path('outputs/runtime/vision.json'))
    p.add_argument('--out', type=Path, default=Path('outputs/rgbd'))
    p.add_argument('--frames', type=int); p.add_argument('--display', action='store_true')
    p.add_argument('--record-images', action='store_true')
    p.add_argument('--geometry-acceptance', type=Path)
    args = p.parse_args()
    import math
    if (args.closed_rad is None) != (args.open_rad is None):
        p.error('Supply both calibrated angles or neither (gripper then remains held)')
    if args.closed_rad is not None and (not all(math.isfinite(x) for x in (args.closed_rad, args.open_rad)) or args.open_rad-args.closed_rad < .1):
        p.error('Provide measured closed/open angles separated by at least 0.1 radians')
    run(args)
