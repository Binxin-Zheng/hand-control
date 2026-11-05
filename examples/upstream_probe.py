"""Unmodified upstream detector recorder; no robot or global position estimation.

File timestamps use presentation time, not inference wall time. Every decoded frame,
including idle and rejected handedness, is retained. Cameras retain explicitly
unverified host dequeue timestamps. Run in the isolated upstream vision environment.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import platform
import time
from pathlib import Path


def record(args):
    import cv2
    import mediapipe
    import numpy

    path = args.repo / 'example/vector_retargeting/single_hand_detector.py'
    spec = importlib.util.spec_from_file_location('upstream_single_hand_detector', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    detector = mod.SingleHandDetector(hand_type=args.hand, selfie=False)
    source = int(args.camera) if args.camera.isdecimal() else args.camera
    is_file = isinstance(source, str) and Path(source).is_file()
    cap = cv2.VideoCapture(source)
    writer = None
    args.out.parent.mkdir(parents=True, exist_ok=True)
    count = selected = 0
    previous_pts = None
    fps = cap.get(cv2.CAP_PROP_FPS)
    try:
        if not cap.isOpened():
            raise RuntimeError('Input could not be opened')
        if is_file and (not math.isfinite(fps) or fps <= 0):
            raise ValueError('Video must have a valid frame rate')
        with args.out.open('w') as out:
            while True:
                ok, frame = cap.read()
                dequeue = time.monotonic()
                if not ok:
                    break
                pts = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000 if is_file else None
                if is_file and (not math.isfinite(pts) or pts < 0 or
                                (previous_pts is not None and pts <= previous_pts)):
                    raise ValueError('Missing/nonincreasing video PTS; refusing invented capture times')
                previous_pts = pts
                t0 = time.perf_counter()
                n, local, pixels, rotation = detector.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                row = {
                    'schema': 'upstream.probe.v2', 'sequence': count,
                    'timestamp_s': pts if is_file else dequeue,
                    'timestamp_semantics': 'video presentation time' if is_file else
                        'host dequeue, NOT verified camera exposure',
                    'detected': bool(n), 'selected_hand': args.hand,
                    'detection_ms': (time.perf_counter() - t0) * 1000,
                    'image_width': frame.shape[1], 'image_height': frame.shape[0],
                    'global_translation_available': False,
                    'local_points_mano': None if not n else local.tolist(),
                    'model_wrist_rotation': None if not n else rotation.tolist(),
                    'operator2mano': detector.operator2mano.tolist(),
                    'pixels': None if not n else detector.parse_keypoint_2d(pixels, frame.shape[:2]).tolist(),
                }
                out.write(json.dumps(row, allow_nan=False) + '\n')
                count += 1
                selected += bool(n)
                if args.annotated or args.display:
                    if n:
                        detector.draw_skeleton_on_image(frame, pixels)
                    cv2.putText(frame, f'{count-1}  {row["timestamp_s"]:.3f}s  {args.hand}: {bool(n)}',
                                (20, 35), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
                    if args.annotated:
                        if writer is None:
                            args.annotated.parent.mkdir(parents=True, exist_ok=True)
                            writer = cv2.VideoWriter(str(args.annotated), cv2.VideoWriter_fourcc(*'mp4v'),
                                                     fps, (frame.shape[1], frame.shape[0]))
                            if not writer.isOpened():
                                raise RuntimeError('Cannot open annotated video writer')
                        writer.write(frame)
                    if args.display:
                        cv2.imshow('Upstream baseline (mirrored preview only)', cv2.flip(frame, 1))
                        if cv2.waitKey(1) & 0xff in (27, ord('q')):
                            break
        data = path.read_bytes()
        summary = {
            'frames': count, 'selected_hand_frames': selected, 'hand': args.hand,
            'note': 'All-frame counts include idle; NOT hand-present recall. No global position.',
            'input': str(source), 'fps': fps,
            'input_sha256': hashlib.sha256(Path(source).read_bytes()).hexdigest() if is_file else None,
            'detector_git_blob': hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest(),
            'python': platform.python_version(), 'mediapipe': mediapipe.__version__,
            'numpy': numpy.__version__, 'opencv': cv2.__version__,
        }
        args.out.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if args.display:
            cv2.destroyAllWindows()
        detector.hand_detector.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo', type=Path, required=True)
    p.add_argument('--camera', default='0', help='Camera index or complete video path')
    p.add_argument('--hand', choices=['Right', 'Left'], default='Right')
    p.add_argument('--out', type=Path, default=Path('upstream_probe.jsonl'))
    p.add_argument('--display', action='store_true')
    p.add_argument('--annotated', type=Path, help='Save all frames with 21-point overlay (file input only)')
    args = p.parse_args()
    if args.annotated and not Path(args.camera).is_file():
        p.error('--annotated requires a video file with known FPS')
    record(args)


if __name__ == '__main__':
    main()
