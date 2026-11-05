"""Deterministic ideal-plant exercise; no camera, dynamics, collision model or robot.
Run from repository root: PYTHONPATH=src python examples/offline_demo.py
"""
import argparse,json,time,platform
from pathlib import Path
import numpy as np
from anyteleop_so101.geometry import Pose
from anyteleop_so101.kinematics import URDFChain
from anyteleop_so101.mapping import RelativeMapper
from anyteleop_so101.servo import PositionPriorityServo
from anyteleop_so101.session import TeleopSession,RobotState
from anyteleop_so101.protocol import HandObservation


def build():
    root=Path(__file__).resolve().parents[1]
    chain=URDFChain(root/'assets/so101_kinematics_only.urdf','base_link','gripper_link')
    session=TeleopSession(PositionPriorityServo(chain),RelativeMapper(np.eye(3)),allow_synthetic=True)
    return chain,session


def run(output_dir,frames=1200):
    out=Path(output_dir);out.mkdir(parents=True,exist_ok=True)
    chain,s=build(); q0=np.array([.1,-.3,.4,.15,.2]); q=q0.copy(); jaw=.5
    home=chain.fk(q0); h0=np.array([0.,0.,.7]); dt=.01; current=None; sequence=0
    error=[]; leaks=[]; states={}; jumps=[]; rows=[]; max_velocity=0.; t0=time.perf_counter()
    for i in range(frames):
        t=1+i*dt
        if i%4==0:
            phase=i*dt
            qref=q0+np.array([.04*np.sin(phase),.05*np.sin(.7*phase),.06*np.sin(.6*phase),.03*np.sin(.8*phase),.1*np.sin(.5*phase)])
            goal=chain.fk(qref); hand=Pose(h0+goal.p-home.p,goal.R@home.R.T)
            sequence+=1
            current=HandObservation(sequence,t,'right:0','camera_color_optical',hand,.5+.4*np.sin(.5*phase),'synthetic','synthetic','synthetic')
        observation=None if 400<=i<420 else current
        clutch=not (440<=i<450 or 820<=i<850)
        events={'clutch':clutch,'ready':True,'estop':i==800,'reset_estop':i==820}
        measured=RobotState(t,q.copy(),jaw)
        d=s.tick(observation,measured,now_s=t,dt=dt,**events)
        states[d.state]=states.get(d.state,0)+1
        if d.reason=='engaged_no_jump': jumps.append(float(np.max(np.abs(d.q_command-q))))
        if 'position_error_m' in d.diagnostics:
            error.append(d.diagnostics['position_error_m']); leaks.append(d.diagnostics['primary_leak_m_s'])
        if d.q_command is not None:
            max_velocity=max(max_velocity,float(np.max(np.abs(d.q_command-q)))/dt)
        assert d.state=='ACTIVE' or d.q_command is None
        row={'now_s':t,'dt':dt,'events':events,'observation':None if observation is None else observation.as_dict(),
             'robot':{'timestamp_s':t,'q_rad':q.tolist(),'aperture':jaw},
             'decision':{'state':d.state,'reason':d.reason,'q_command_rad':None if d.q_command is None else d.q_command.tolist(),
                         'aperture_command':d.aperture_command,'diagnostics':d.diagnostics}}
        rows.append(row)
        # Ideal plant follows one validated numerical step. This is NOT physical simulation.
        if d.q_command is not None: q=d.q_command.copy();jaw=d.aperture_command
    with (out/'trace.jsonl').open('w') as f:
        for row in rows:f.write(json.dumps(row,allow_nan=False)+'\n')
    summary={'test_kind':'synthetic ideal-plant numerical replay; NOT hardware or perception accuracy',
             'frames':frames,'states':states,'max_reengage_joint_jump_rad':max(jumps,default=0.),
             'position_tracking_error_p95_m':float(np.quantile(error,.95)),
             'max_primary_velocity_leak_m_s':max(leaks,default=0.),'max_joint_velocity_rad_s':max_velocity,
             'wall_time_s':time.perf_counter()-t0,'python':platform.python_version(),
             'no_motor_driver':True,'collision_model':False,'endpoint':'gripper_link origin; NOT calibrated TCP'}
    assert summary['max_reengage_joint_jump_rad']<1e-12
    assert summary['max_primary_velocity_leak_m_s']<1e-7
    (out/'metrics.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))
    return summary

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--out',default='outputs/demo');parser.add_argument('--frames',type=int,default=1200)
    args=parser.parse_args()
    if args.frames<900:parser.error('Use >=900 frames to include dropout/re-engage/E-stop cases')
    run(args.out,args.frames)
