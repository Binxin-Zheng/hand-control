"""Color-only upstream detector for the automatic SIMULATION gripper preview.

This preview-only protocol is intentionally not a robot observation envelope.
Capture time is host dequeue time, not hardware-validated exposure time.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import time


def run(args):
    import cv2
    import numpy as np
    from anyteleop_so101.perception import aperture_angle
    from anyteleop_so101.runtime.dex_client import DexClient
    from anyteleop_so101.runtime.preview_ui import hand_view, compose
    from anyteleop_so101.runtime.transport import publish_file
    path=args.detector_repo/'example/vector_retargeting/single_hand_detector.py'
    data=path.read_bytes()
    if hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest() != '4867ad7d2e5ea3984002c1730b937d06b1fe97ee':
        raise ValueError('Detector differs from audited upstream baseline')
    spec=importlib.util.spec_from_file_location('gripper_upstream_detector',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    detector=module.SingleHandDetector(hand_type=args.hand,selfie=False)
    pipeline=video=retargeter=None
    args.out.mkdir(parents=True,exist_ok=True)
    try:
        retargeter=DexClient(args.dex_python,Path('assets/so101/so101_new_calib.urdf').resolve())
        if args.video:
            video=cv2.VideoCapture(str(args.video))
            if not video.isOpened(): raise RuntimeError('Cannot open video')
        else:
            import pyrealsense2 as rs
            pipeline=rs.pipeline();config=rs.config()
            if args.serial:config.enable_device(args.serial)
            config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
            pipeline.start(config)
        sequence=0
        with (args.out/'frames.jsonl').open('w') as log:
            while args.frames is None or sequence<args.frames:
                if video is not None:
                    ok,image=video.read()
                    if not ok:break
                else:
                    frame=pipeline.wait_for_frames(timeout_ms=1000).get_color_frame()
                    if not frame:continue
                    image=np.asanyarray(frame.get_data())
                acquired=time.monotonic()
                count,points,pixels,rotation=detector.detect(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
                angle=None;opening=None;ratio=None;result=None
                if count:
                    try:
                        ratio=aperture_angle(points)
                        result=retargeter.retarget(points)
                        angle=result['angle_rad'];opening=result['opening']
                    except ValueError:pass
                packet=dict(schema='teleop.gripper_preview.v1',sequence=sequence,timestamp_s=acquired,
                            hand=args.hand,angle_rad=angle,detected=bool(count),simulation_only=True,
                            local_points_mano=None if not count else points.tolist(),
                            mapping='dex_palm_thumb',opening=opening,baseline_angle_rad=ratio,
                            joint_limits_rad=None if result is None else result['joint_limits_rad'],
                            optimizer='dex_retargeting.VectorOptimizer',evidence='rgb_model')
                publish_file(args.vision_file,packet)
                log.write(json.dumps(packet,allow_nan=False)+'\n');log.flush()
                preview=image.copy()
                if count:detector.draw_skeleton_on_image(preview,pixels)
                uv=None if not count else detector.parse_keypoint_2d(pixels,image.shape[:2])
                hand=hand_view(preview,uv)
                try:robot=np.load(args.out/'robot.npy',allow_pickle=False)
                except (OSError,ValueError):robot=None
                status='Paused' if (args.out/'paused').exists() else 'Show your '+args.hand.lower()+' hand' if opening is None else 'Following automatically'
                view=compose(robot,hand,opening,ratio,status,joint_angle_rad=angle)
                if sequence%30==0:cv2.imwrite(str(args.out/'preview.png'),view)
                if not args.headless:
                    cv2.imshow('Gripper | Hand',view)
                    key=cv2.waitKey(1)&0xff
                    if key in (27,ord('q')):break
                    if key==ord(' '):
                        if (args.out/'paused').exists():(args.out/'paused').unlink()
                        else:(args.out/'paused').touch()
                sequence+=1
        print(f'Processed {sequence} color frames; no depth or motor connection.',flush=True)
    finally:
        if retargeter is not None:retargeter.close()
        if pipeline is not None:pipeline.stop()
        if video is not None:video.release()
        detector.hand_detector.close()
        if not args.headless:cv2.destroyAllWindows()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--detector-repo',type=Path,default=Path('/home/bx/repos/dex-retargeting'))
    p.add_argument('--dex-python',type=Path,default=Path('/home/bx/repos/dex-retargeting/.venv/bin/python'))
    p.add_argument('--vision-file',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--hand',choices=['Right','Left'],default='Right')
    p.add_argument('--serial');p.add_argument('--video',type=Path)
    p.add_argument('--headless',action='store_true');p.add_argument('--frames',type=int)
    try:
        run(p.parse_args())
    except KeyboardInterrupt:
        print('Gripper camera preview stopped.',flush=True)
