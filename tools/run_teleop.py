"""Explicit simulator or physical teleoperation entry point.

Terminal commands (press Enter): engage, release, estop, reset, quit.
Simulator keyboard equivalents: E, R, X, C, Q. A synthetic source is allowed ONLY
in simulation and is labeled synthetic. JSONL logs retain every control decision.
Explicit --automatic-preview is simulation-only: Space pauses/resumes, Q quits.
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
from anyteleop_so101.runtime.control import GuardedController, configure_hardware_geometry
from anyteleop_so101.runtime.transport import ObservationReceiver, FileObservationReceiver
from anyteleop_so101.runtime.robot_client import RobotWorkerClient
from anyteleop_so101.runtime.automatic_preview import AutomaticPreview


class TimedCollisionGuard:
    """Time the existing guard without changing its calls, result, or exceptions."""
    def __init__(self, guard):
        self.guard = guard
        self.duration_s = 0.

    def check_segment(self, *args, **kwargs):
        started = time.monotonic()
        try:
            return self.guard.check_segment(*args, **kwargs)
        finally:
            self.duration_s += time.monotonic()-started

    def __getattr__(self, name):
        return getattr(self.guard, name)


def observation_status(observation, receiver_error, now_s):
    if observation is None:
        return 'camera input rejected: '+receiver_error if receiver_error else 'no camera input'
    if observation.pose is None:
        return 'wrist XYZ unavailable'
    return f'XYZ available; age {1000*(now_s-observation.timestamp_s):.0f} ms'


def overlay_refresh_due(status, previous_status, now_s, next_refresh_s):
    return status != previous_status or now_s >= next_refresh_s


def run(args):
    automatic_preview = getattr(args, 'automatic_preview', False)
    if automatic_preview and (args.backend != 'sim' or args.control_mode != 'position'):
        raise ValueError('Automatic visual preview requires --backend sim --control-mode position; it cannot control hardware')
    if automatic_preview and getattr(args, 'vision_port', None) is not None:
        raise ValueError('Automatic visual preview uses latest-file IPC; omit --vision-port and use --vision-file')
    config = json.loads(args.config.read_text())
    if args.backend == 'robot' and (not args.execute or args.synthetic):
        raise ValueError('Hardware requires --execute and real observations')
    tcp = config.get('tcp')
    if args.backend == 'robot' and tcp is None:
        raise ValueError('Calibrated fixed TCP is required for hardware')
    chain = URDFChain(config['urdf'], 'base_link', 'gripper_link',
                      None if tcp is None else Pose.from_xyzw(tcp['position_m'], tcp['quaternion_xyzw']))
    collision = dict(config.get('collision', {}))
    if args.backend == 'robot':
        # Validate before opening any worker, then give IK the physical/model
        # intersection and the collision guard the driver's actual jaw geometry.
        hardware = json.loads(Path(config['robot_config']).read_text())
        from importlib.util import spec_from_file_location, module_from_spec
        spec = spec_from_file_location('robot_worker_validation', Path(__file__).with_name('robot_worker.py'))
        module = module_from_spec(spec); spec.loader.exec_module(module)
        module.validate_acceptance(hardware)
        collision = configure_hardware_geometry(chain, hardware, collision)
    session = TeleopSession(PositionPriorityServo(chain),
        RelativeMapper(config['R_base_camera'], config['translation_scale']),
        camera_frame=config.get('camera_frame', 'camera_color_optical'), allow_synthetic=args.synthetic,
        control_mode=args.control_mode)
    preview = AutomaticPreview() if automatic_preview else None
    guard = plant = receiver = None
    clutch = estop = False
    sim_holding = True
    sequence = 0
    status_overlay = -1
    overlay_status = None
    next_overlay_refresh = 0.
    pause_cause = ''
    try:
        guard = BulletWorld(config['urdf'], **collision)
        timed_guard = TimedCollisionGuard(guard)
        controller = GuardedController(session, timed_guard)
        if args.backend == 'sim':
            plant = BulletWorld(config['urdf'], gui=not args.headless, **collision)
            if not args.headless and 'sim_view' in config:
                view = config['sim_view']
                plant.p.resetDebugVisualizerCamera(view['distance'], view['yaw'], view['pitch'], view['target'],
                                                   physicsClientId=plant.client)
            q = np.asarray(config['sim_initial_q_rad'])
            plant.set_configuration(q, config['sim_initial_aperture'])
            plant.command(q, config['sim_initial_aperture'])
        else:
            plant = RobotWorkerClient(config['robot_python'], config['robot_config'], enable_torque=args.enable_torque)
        if not args.synthetic:
            receiver = (ObservationReceiver(args.vision_port) if args.vision_port is not None
                        else FileObservationReceiver(args.vision_file))
        args.log.parent.mkdir(parents=True, exist_ok=True)
        start = previous = time.monotonic()
        deadline = start
        last_status = ''
        previous_cycle_timing = None
        if preview:
            print('Automatic visual preview — simulation only. '
                  'Show your hand to follow. Space pauses/resumes; Q quits. Terminal: pause / resume / quit (Enter).', flush=True)
        else:
            print(f'Control mode: {args.control_mode}. Commands: engage / release / estop / reset / quit (Enter). Starts RELEASED.', flush=True)
        with args.log.open('w') as log:
            while True:
                now = time.monotonic()
                if args.duration is not None and now-start >= args.duration:
                    break
                timings = {}
                command = None
                if select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline()
                    command = line.strip() if line else None
                if args.backend == 'sim' and not args.headless:
                    keys = plant.p.getKeyboardEvents(physicsClientId=plant.client)
                    for key, value in keys.items():
                        if value & plant.p.KEY_WAS_TRIGGERED:
                            key = ord(chr(key).lower()) if 0 <= key < 128 else key
                            command = {ord('e'): 'engage', ord('r'): 'release', ord('x'): 'estop',
                                       ord('c'): 'reset', ord('q'): 'quit'}.get(key, command)
                            if preview and key == ord(' '):
                                command = 'toggle_pause'
                if command == 'quit': break
                if preview:
                    preview.command(command)
                else:
                    if command == 'engage': clutch = True
                    if command == 'release': clutch = False
                reset = command == 'reset' and not clutch
                if command == 'estop': estop = True
                if reset and not preview: estop = False
                phase_started = time.monotonic()
                timings['input'] = phase_started-now
                measured = plant.read_state()
                timings['robot_read'] = time.monotonic()-phase_started
                if measured is None:
                    if now-start > 5: raise RuntimeError('No driver feedback within 5 seconds')
                    time.sleep(.01); previous = time.monotonic(); continue
                phase_started = time.monotonic()
                observation = None if receiver is None else receiver.poll()
                receiver_error = None if receiver is None else receiver.error
                receiver_error_kind = None if receiver is None else receiver.error_kind
                if args.synthetic:
                    phase = now-start
                    observation = HandObservation(sequence, now, 'synthetic:0', session.camera_frame,
                        Pose([.01*np.sin(phase), .01*np.sin(.7*phase), .6+.01*np.sin(.5*phase)], np.eye(3)),
                        .5+.2*np.sin(phase), 'synthetic', 'synthetic', 'synthetic')
                timings['observation_read'] = time.monotonic()-phase_started
                ready = args.backend == 'sim' or (receiver is not None and receiver.metadata.get('geometry_validated') is True)
                if preview:
                    clutch = preview.clutch_for_tick(observation, now_s=now, max_age_s=session.max_age,
                        camera_frame=session.camera_frame, allow_synthetic=args.synthetic,
                        receiver_error=receiver_error, receiver_error_kind=receiver_error_kind)
                    reset = command == 'reset' and not clutch
                    if reset: estop = False
                dt = max(now-previous, 1e-6); previous = now
                phase_started = time.monotonic()
                timed_guard.duration_s = 0.
                decision = controller.tick(observation, measured, now_s=now, dt=dt,
                    clutch=clutch, estop=estop, reset_estop=reset, ready=ready)
                decision_completed = time.monotonic()
                timings['collision'] = timed_guard.duration_s
                timings['session_ik'] = max(0., decision_completed-phase_started-timed_guard.duration_s)
                timings['control_total'] = decision_completed-now
                if timings['control_total'] > .04:
                    decision = session._stop('PAUSED', 'control_cycle_overrun')
                if preview:
                    preview.observe(decision)
                phase_started = time.monotonic()
                if args.backend == 'sim':
                    if decision.q_command is None:
                        if not sim_holding: plant.stop()
                        sim_holding = True
                    else:
                        plant.command(decision.q_command, decision.aperture_command)
                        sim_holding = False
                    timings['command'] = time.monotonic()-phase_started
                    phase_started = time.monotonic()
                    for _ in range(4): plant.step()
                    timings['physics'] = time.monotonic()-phase_started
                else:
                    mode = ('reset' if reset else 'estop' if decision.state == 'ESTOP' else
                            'release' if decision.state == 'IDLE' else 'active' if decision.q_command is not None else 'stop')
                    plant.send(mode, decision.q_command, decision.aperture_command)
                    # Driver refusal/expiry must also reset controller reference.
                    if plant.reason not in ('active', 'released', 'starting') and decision.state == 'ACTIVE':
                        session._stop('PAUSED', 'driver_requires_release')
                    timings['command'] = time.monotonic()-phase_started
                    timings['physics'] = 0.
                phase_started = time.monotonic()
                status = args.control_mode+' | '+decision.state+': '+decision.reason
                preview_state = None if preview is None else preview.as_dict()
                if preview:
                    status = 'AUTOMATIC VISUAL PREVIEW | '+preview_state['state']
                    if preview.fault:
                        status += ': '+preview.fault
                if decision.state == 'PAUSED' and decision.reason != 'release_then_engage_required':
                    pause_cause = decision.reason
                    if decision.reason == 'tracking_unavailable' and receiver_error:
                        pause_cause += ' ('+receiver_error+')'
                if decision.state in ('IDLE', 'ACTIVE'):
                    pause_cause = ''
                observation_text = observation_status(observation, receiver_error, now)
                hint = (' | Last fault: '+pause_cause+' | R release, then E engage'
                        if decision.state == 'PAUSED' else ' | E engage / R release / X stop / C reset / Q quit')
                if preview:
                    hint = (' | C reset, then Space resume | Q quit' if preview.stopped else
                            ' | Space resume | Q quit' if preview.paused or preview.fault else
                            ' | Space pause | Q quit | X stop')
                if status != last_status:
                    print(status+' | '+observation_text+hint, flush=True); last_status = status
                timings['status_output'] = time.monotonic()-phase_started
                phase_started = time.monotonic()
                if (args.backend == 'sim' and not args.headless
                        and overlay_refresh_due(status, overlay_status, phase_started, next_overlay_refresh)):
                    status_overlay = plant.p.addUserDebugText(
                        status+'\n'+observation_text+'\n'+hint,
                        [0, 0, .45], textColorRGB=[1, .2, .1] if decision.state in ('PAUSED','ESTOP') else [0,.5,0],
                        textSize=1.2, replaceItemUniqueId=status_overlay, physicsClientId=plant.client)
                    overlay_status = status
                    next_overlay_refresh = time.monotonic()+.1
                before_log = time.monotonic()
                timings['ui'] = before_log-phase_started
                timings['post_decision'] = before_log-decision_completed
                timings['before_log'] = before_log-now
                log.write(json.dumps(dict(sequence=sequence, timestamp_s=now, dt=dt, control_mode=args.control_mode,
                    automatic_preview=preview_state,
                    observation=None if observation is None else observation.as_dict(),
                    observation_error=receiver_error, observation_error_kind=receiver_error_kind, timing_s=timings,
                    post_decision_overrun=timings['before_log'] > .04 and timings['control_total'] <= .04,
                    previous_cycle_timing=previous_cycle_timing,
                    measured_q=measured.q.tolist(), measured_aperture=measured.aperture,
                    target=None if decision.desired_pose is None else decision.desired_pose.as_dict(),
                    state=decision.state, reason=decision.reason,
                    q_command=None if decision.q_command is None else decision.q_command.tolist(),
                    aperture_command=decision.aperture_command, diagnostics=decision.diagnostics), allow_nan=False)+'\n')
                after_log = time.monotonic()
                # A row cannot include its own completed write/sleep. Attach those
                # to the next row with the originating sequence, without extra IO.
                previous_cycle_timing = dict(sequence=sequence, log_write_s=after_log-before_log,
                                             work_s=after_log-now)
                sequence += 1
                deadline += .02
                time.sleep(max(0, deadline-time.monotonic()))
                after_sleep = time.monotonic()
                previous_cycle_timing.update(sleep_s=after_sleep-after_log,
                                             deadline_lateness_s=max(0., after_sleep-deadline))
                if after_sleep-deadline > .05: deadline = time.monotonic()
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
    p.add_argument('--control-mode', choices=['full_pose', 'position'], default='full_pose',
                   help='Explicit position mode holds the robot orientation latched at engagement; full_pose follows hand rotation')
    p.add_argument('--automatic-preview', action='store_true',
                   help='Simulation-only automatic visual follow/reacquisition with latest-file IPC; requires position mode; Space pauses/resumes')
    p.add_argument('--headless', action='store_true'); p.add_argument('--synthetic', action='store_true')
    p.add_argument('--execute', action='store_true'); p.add_argument('--enable-torque', action='store_true')
    p.add_argument('--vision-port', type=int, help='Optional localhost UDP instead of file IPC')
    p.add_argument('--vision-file', type=Path, default=Path('outputs/runtime/vision.json'))
    p.add_argument('--duration', type=float)
    p.add_argument('--log', type=Path, default=Path('outputs/runtime/trace.jsonl'))
    try:
        run(p.parse_args())
    except KeyboardInterrupt:
        pass  # run() stops and closes the plant in its finally block.
