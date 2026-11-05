"""Explicit simulator or physical teleoperation entry point. No auto-engage.

Terminal commands (press Enter): engage, release, estop, reset, quit.
Simulator keyboard equivalents: E, R, X, C, Q. A synthetic source is allowed ONLY
in simulation and is labeled synthetic. JSONL logs retain every control decision.
"""
import argparse
import json
from pathlib import Path
import select
import sys
import time
import numpy as np
from anyteleop_so101.geometry import Pose
from anyteleop_so101.kinematics import URDFChain
from anyteleop_so101.mapping import RelativeMapper
from anyteleop_so101.protocol import HandObservation
from anyteleop_so101.servo import PositionPriorityServo
from anyteleop_so101.session import TeleopSession
from anyteleop_so101.runtime.bullet import BulletWorld
from anyteleop_so101.runtime.control import GuardedController
from anyteleop_so101.runtime.transport import ObservationReceiver, FileObservationReceiver
from anyteleop_so101.runtime.robot_client import RobotWorkerClient


def run(args):
    config = json.loads(args.config.read_text())
    if args.backend == 'robot' and (not args.execute or args.synthetic):
        raise ValueError('Hardware requires --execute and real observations')
    tcp = config.get('tcp')
    if args.backend == 'robot' and tcp is None:
        raise ValueError('Calibrated fixed TCP is required for hardware')
    chain = URDFChain(config['urdf'], 'base_link', 'gripper_link',
                      None if tcp is None else Pose.from_xyzw(tcp['position_m'], tcp['quaternion_xyzw']))
    session = TeleopSession(PositionPriorityServo(chain),
        RelativeMapper(config['R_base_camera'], config['translation_scale']),
        camera_frame=config.get('camera_frame', 'camera_color_optical'), allow_synthetic=args.synthetic)
    guard = plant = receiver = None
    clutch = estop = False
    sim_holding = True
    sequence = 0
    try:
        guard = BulletWorld(config['urdf'], **config.get('collision', {}))
        controller = GuardedController(session, guard)
        if args.backend == 'sim':
            plant = BulletWorld(config['urdf'], gui=not args.headless, **config.get('collision', {}))
            q = np.asarray(config['sim_initial_q_rad'])
            plant.set_configuration(q, config['sim_initial_aperture'])
            plant.command(q, config['sim_initial_aperture'])
        else:
            # Gate BEFORE launching the worker, including source/TCP/scene acceptance.
            hardware = json.loads(Path(config['robot_config']).read_text())
            from importlib.util import spec_from_file_location, module_from_spec
            spec = spec_from_file_location('robot_worker_validation', Path(__file__).with_name('robot_worker.py'))
            module = module_from_spec(spec); spec.loader.exec_module(module)
            module.validate_acceptance(hardware)
            plant = RobotWorkerClient(config['robot_python'], config['robot_config'], enable_torque=args.enable_torque)
        if not args.synthetic:
            receiver = (ObservationReceiver(args.vision_port) if args.vision_port is not None
                        else FileObservationReceiver(args.vision_file))
        args.log.parent.mkdir(parents=True, exist_ok=True)
        start = previous = time.monotonic()
        deadline = start
        last_status = ''
        print('Commands: engage / release / estop / reset / quit (Enter). Starts RELEASED.', flush=True)
        with args.log.open('w') as log:
            while True:
                now = time.monotonic()
                if args.duration is not None and now-start >= args.duration:
                    break
                command = None
                if select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline()
                    command = line.strip() if line else None
                if args.backend == 'sim' and not args.headless:
                    keys = plant.p.getKeyboardEvents(physicsClientId=plant.client)
                    for key, value in keys.items():
                        if value & plant.p.KEY_WAS_TRIGGERED:
                            command = {ord('e'): 'engage', ord('r'): 'release', ord('x'): 'estop',
                                       ord('c'): 'reset', ord('q'): 'quit'}.get(key, command)
                reset = command == 'reset' and not clutch
                if command == 'quit': break
                if command == 'engage': clutch = True
                if command == 'release': clutch = False
                if command == 'estop': estop = True
                if reset: estop = False
                measured = plant.read_state()
                if measured is None:
                    if now-start > 5: raise RuntimeError('No driver feedback within 5 seconds')
                    time.sleep(.01); previous = time.monotonic(); continue
                observation = None if receiver is None else receiver.poll()
                if args.synthetic:
                    phase = now-start
                    observation = HandObservation(sequence, now, 'synthetic:0', session.camera_frame,
                        Pose([.01*np.sin(phase), .01*np.sin(.7*phase), .6+.01*np.sin(.5*phase)], np.eye(3)),
                        .5+.2*np.sin(phase), 'synthetic', 'synthetic', 'synthetic')
                ready = args.backend == 'sim' or (receiver is not None and receiver.metadata.get('geometry_validated') is True)
                dt = max(now-previous, 1e-6); previous = now
                decision = controller.tick(observation, measured, now_s=now, dt=dt,
                    clutch=clutch, estop=estop, reset_estop=reset, ready=ready)
                if time.monotonic()-now > .04:
                    decision = session._stop('PAUSED', 'control_cycle_overrun')
                if args.backend == 'sim':
                    if decision.q_command is None:
                        if not sim_holding: plant.stop()
                        sim_holding = True
                    else:
                        plant.command(decision.q_command, decision.aperture_command)
                        sim_holding = False
                    for _ in range(4): plant.step()
                else:
                    mode = ('reset' if reset else 'estop' if decision.state == 'ESTOP' else
                            'release' if decision.state == 'IDLE' else 'active' if decision.q_command is not None else 'stop')
                    plant.send(mode, decision.q_command, decision.aperture_command)
                    # Driver refusal/expiry must also reset controller reference.
                    if plant.reason not in ('active', 'released', 'starting') and decision.state == 'ACTIVE':
                        session._stop('PAUSED', 'driver_requires_release')
                status = decision.state+': '+decision.reason
                if status != last_status:
                    print(status, flush=True); last_status = status
                log.write(json.dumps(dict(sequence=sequence, timestamp_s=now, dt=dt,
                    observation=None if observation is None else observation.as_dict(),
                    measured_q=measured.q.tolist(), measured_aperture=measured.aperture,
                    target=None if decision.desired_pose is None else decision.desired_pose.as_dict(),
                    state=decision.state, reason=decision.reason,
                    q_command=None if decision.q_command is None else decision.q_command.tolist(),
                    aperture_command=decision.aperture_command, diagnostics=decision.diagnostics), allow_nan=False)+'\n')
                sequence += 1
                deadline += .02
                time.sleep(max(0, deadline-time.monotonic()))
                if time.monotonic()-deadline > .05: deadline = time.monotonic()
    finally:
        try:
            if plant is not None:
                try:
                    if args.backend == 'sim': plant.stop()
                finally: plant.close()
        finally:
            try:
                if guard is not None: guard.close()
            finally:
                if receiver is not None: receiver.close()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=Path('config/runtime.sim.json'))
    p.add_argument('--backend', choices=['sim', 'robot'], default='sim')
    p.add_argument('--headless', action='store_true'); p.add_argument('--synthetic', action='store_true')
    p.add_argument('--execute', action='store_true'); p.add_argument('--enable-torque', action='store_true')
    p.add_argument('--vision-port', type=int, help='Optional localhost UDP instead of file IPC')
    p.add_argument('--vision-file', type=Path, default=Path('outputs/runtime/vision.json'))
    p.add_argument('--duration', type=float)
    p.add_argument('--log', type=Path, default=Path('outputs/runtime/trace.jsonl'))
    run(p.parse_args())
