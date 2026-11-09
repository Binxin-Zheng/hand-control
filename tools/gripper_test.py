"""Automatic SO101 gripper preview: open, show your hand, follow. No calibration.

Simulation only. Reuses the audited upstream detector and model-based finger
opening, mapped to the official revolute joint range (radians). Missing
frames hold the last display pose; reacquisition resumes this kinematic preview.
This does NOT change physical-arm or full-arm clutch/re-engagement semantics.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

import numpy as np
from anyteleop_so101.runtime.bullet import BulletWorld


def preview_target(packet, now, lower, upper, hand='Right'):
    """Direct angle retargeting, not per-operator range calibration."""
    if packet.get('schema') != 'teleop.gripper_preview.v1' or packet.get('simulation_only') is not True:
        raise ValueError('Expected a simulation-only gripper packet')
    stamp=packet.get('timestamp_s');angle=packet.get('angle_rad')
    if packet.get('mapping') in ('thumb_palm_proximity','thumb_fixed_jaw'):
        opening=packet.get('opening')
        angle=opening*upper if isinstance(opening,(int,float)) and 0 <= opening <= 1 else None
    if not isinstance(stamp,(int,float)) or not np.isfinite(stamp) or not 0 <= now-stamp <= .15:
        return None
    if packet.get('hand') != hand or not packet.get('detected') or angle is None:
        return None
    if not isinstance(angle,(int,float)) or not np.isfinite(angle) or not (lower if packet.get('mapping') == 'dex_palm_thumb' else 0) <= angle <= np.pi:
        return None
    return float(np.clip(angle,lower,upper))


def run(args):
    camera=world=None
    # Every launch gets fresh IPC/logs, never a previous session's stale image.
    out=args.out or Path('outputs/gripper-preview')/datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    out.mkdir(parents=True,exist_ok=True)
    vision=args.vision_file or out/'vision.json'
    try:
        if not args.no_camera:
            cmd=[str(args.vision_python),'tools/vision_gripper.py','--detector-repo',str(args.detector_repo),
                 '--hand',args.hand,'--out',str(out),'--vision-file',str(vision)]
            if args.headless:cmd.append('--headless')
            if args.video:cmd.extend(['--video',str(args.video)])
            camera=subprocess.Popen(cmd)
        config=json.loads(Path('config/runtime.sim.json').read_text())
        world=BulletWorld(config['urdf'],gui=False,**config['collision'])
        q=np.asarray(config['sim_initial_q_rad'])
        joint_angle=float(np.clip(0.,world.gmin,world.gmax))
        world.set_configuration(q,(joint_angle-world.gmin)/(world.gmax-world.gmin))
        p,client=world.p,world.client
        tip=next(v[0] for v in world.info.values() if v[12].decode()=='gripper_link')
        target=np.asarray(p.getLinkState(world.body,tip,computeForwardKinematics=True,physicsClientId=client)[4])
        keep={tip,world.gripper}
        for link in world.links:
            if link not in keep:p.changeVisualShape(world.body,link,rgbaColor=[0,0,0,0],physicsClientId=client)
        p.changeVisualShape(world.table,-1,rgbaColor=[0,0,0,0],physicsClientId=client)
        state=p.getLinkState(world.body,tip,computeForwardKinematics=True,physicsClientId=client)
        rotation=np.asarray(p.getMatrixFromQuaternion(state[5])).reshape(3,3)
        center=target+rotation@np.array([0,0,-.045])
        view=p.computeViewMatrix(center+rotation@np.array([0,.32,0]),center,rotation@np.array([0,0,-1]))
        projection=p.computeProjectionMatrixFOV(43,1,.01,2.)
        paused=False;overlay=-1;last_status='';start=time.monotonic()
        print('Automatic SIMULATION preview. Show your hand. Space pauses/resumes; Q quits. No calibration.',flush=True)
        print('Session: '+str(out),flush=True)
        with (out/'preview.jsonl').open('w') as log:
            while args.duration is None or time.monotonic()-start<args.duration:
                now=time.monotonic();command=None
                if camera is not None and camera.poll() is not None:
                    if camera.returncode:raise RuntimeError('Camera worker failed; see its error above')
                    break
                if select.select([sys.stdin],[],[],0)[0]:command=sys.stdin.readline().strip()
                if command=='quit':break
                if command=='pause':
                    if (out/'paused').exists():(out/'paused').unlink()
                    else:(out/'paused').touch()
                paused=(out/'paused').exists()
                new_angle=None;error=None
                try:
                    if vision.stat().st_size>65536:raise ValueError('Oversized packet')
                    packet=json.loads(vision.read_bytes())
                    new_angle=preview_target(packet,now,world.gmin,world.gmax,args.hand)
                except (OSError,ValueError,TypeError,KeyError) as exc:
                    error=str(exc)
                status='Paused' if paused else 'Show your hand — holding last pose' if new_angle is None else 'Following'
                if new_angle is not None and not paused:joint_angle=new_angle
                # Official URDF joint and visual transforms; arm remains fixed.
                world.set_configuration(q,(joint_angle-world.gmin)/(world.gmax-world.gmin))
                _,_,rgba,_,seg=p.getCameraImage(512,512,viewMatrix=view,projectionMatrix=projection,
                    renderer=p.ER_TINY_RENDERER,flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,physicsClientId=client)
                pixels=np.asarray(rgba,dtype=np.uint8).reshape(512,512,4).copy()
                segmentation=np.asarray(seg).reshape(512,512)
                keep_pixels=((segmentation&((1<<24)-1))==world.body)&np.isin((segmentation>>24)-1,list(keep))
                pixels[~keep_pixels]=[24,28,35,255]
                with (out/'robot.tmp').open('wb') as image_file:np.save(image_file,pixels)
                (out/'robot.tmp').replace(out/'robot.npy')
                if status!=last_status:print(status,flush=True);last_status=status
                log.write(json.dumps(dict(timestamp_s=now,status=status,joint_angle_rad=joint_angle,
                                          input_valid=new_angle is not None,error=error),allow_nan=False)+'\n')
                time.sleep(.02)
    finally:
        if world is not None:world.close()
        if camera is not None and camera.poll() is None:
            camera.send_signal(signal.SIGINT)
            try:camera.wait(timeout=3)
            except subprocess.TimeoutExpired:camera.kill();camera.wait()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-camera',action='store_true',help='Use an external preview producer for testing')
    parser.add_argument('--start-camera',action='store_true',help=argparse.SUPPRESS)  # Old launch command still works.
    parser.add_argument('--vision-python',type=Path,default=Path('.venv-vision/bin/python'))
    parser.add_argument('--detector-repo',type=Path,default=Path('/home/bx/repos/dex-retargeting'))
    parser.add_argument('--vision-file',type=Path)
    parser.add_argument('--out',type=Path)
    parser.add_argument('--hand',choices=['Right','Left'],default='Right')
    parser.add_argument('--headless',action='store_true');parser.add_argument('--duration',type=float)
    parser.add_argument('--video',type=Path,help='Offline video input for repeatable detector/preview tests')
    run(parser.parse_args())
