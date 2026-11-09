"""Launch automatic D455 hand-driven SO101 visual simulation.

Space pauses/resumes and Q quits. Tracking loss freezes the preview; fresh
tracking resumes from a new measured simulation anchor. --manual-control keeps
the explicit engage/release workflow for control validation. This launcher has
no physical-robot mode or motor access.
Headless mode still uses the camera; it only disables the preview windows.
"""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time
import uuid


ROOT = Path(__file__).resolve().parents[1]


def build_commands(args, session, root=ROOT):
    """Fixed simulation-only commands; no arbitrary child flags are forwarded."""
    vision = [str(root/'.venv-vision/bin/python'), str(root/'tools/vision_rgbd.py'),
              '--detector-repo', str(args.detector_repo), '--hand', args.hand,
              '--depth-size', getattr(args, 'depth_size', '640x360'), '--position-method', 'wrist_surface_fit',
              '--out', str(session/'camera'), '--vision-file', str(session/'vision.json')]
    controller = [str(root/'.venv-core/bin/python'), str(root/'tools/run_teleop.py'),
                  '--backend', 'sim', '--control-mode', 'position',
                  '--config', str(root/'config/runtime.sim.json'),
                  '--vision-file', str(session/'vision.json'), '--log', str(session/'control.jsonl')]
    if not args.manual_control:
        controller.append('--automatic-preview')
    if args.headless:
        controller.append('--headless')
    else:
        vision.extend(['--display', '--mirror-preview'])
    if args.record_images:
        vision.append('--record-images')
    if getattr(args, 'record_stereo', False):
        vision.append('--record-stereo')
    return {'vision': vision, 'controller': controller}


def isolated_environment(python, root=ROOT):
    env = os.environ.copy()
    for name in ('PYTHONPATH', 'PYTHONHOME', 'VIRTUAL_ENV', 'CONDA_PREFIX', 'CONDA_DEFAULT_ENV', 'CONDA_SHLVL'):
        env.pop(name, None)
    environment = Path(python).parent.parent
    env.update(PYTHONPATH=str(root/'src'), PYTHONUNBUFFERED='1', PYTHONNOUSERSITE='1',
               VIRTUAL_ENV=str(environment), PATH=str(environment/'bin')+os.pathsep+env.get('PATH', ''))
    return env


def pump_output(selector, timeout=.1):
    for key, _ in selector.select(timeout):
        name, log = key.data
        try:
            chunk = os.read(key.fileobj.fileno(), 65536)
        except BlockingIOError:
            continue
        if not chunk:
            selector.unregister(key.fileobj)
            key.fileobj.close()
            continue
        log.write(chunk)
        log.flush()
        try:
            sys.stdout.write(f'[{name}] '+chunk.decode('utf-8', errors='replace'))
            sys.stdout.flush()
        except (BrokenPipeError, OSError):
            pass  # Closed console must not interrupt child cleanup or file logs.


def group_alive(process):
    process.poll()  # Reap a finished leader before testing its process group.
    try:
        os.killpg(process.pid, 0)
    except ProcessLookupError:
        return False
    return True


def stop_children(children, selector, grace_s=3., terminate_s=2.):
    """Stop both groups together, drain their logs, and reap every direct child."""
    output_errors = []
    def drain(timeout):
        try:
            pump_output(selector, timeout)
        except Exception as error:
            # A log/console failure must not abandon a still-running child.
            if not output_errors:
                output_errors.append(error)

    for sig, timeout in ((signal.SIGINT, grace_s), (signal.SIGTERM, terminate_s), (signal.SIGKILL, 1.)):
        active = [process for process in children.values() if group_alive(process)]
        if not active:
            break
        for process in active:
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and any(group_alive(process) for process in active):
            drain(min(.05, max(0., deadline-time.monotonic())))
    for process in children.values():
        process.wait(timeout=1.)
    # Reaped children may still have buffered diagnostic output in their pipes.
    deadline = time.monotonic()+1.
    while selector.get_map() and time.monotonic() < deadline:
        drain(.05)
    if output_errors:
        raise RuntimeError('Could not preserve all child output during cleanup') from output_errors[0]


def supervise(commands, session, *, duration=None, root=ROOT, cleanup_grace_s=3., cleanup_terminate_s=2.):
    """Supervise foreground children. Either exiting ends this whole session."""
    children = {}
    stop = {'signal': None}
    previous = {}
    reason, returncode = 'startup_failed', 1
    started = time.monotonic()
    metadata = dict(schema='teleop.hand_follow_session.v1', simulation_only=True,
                    started_utc=datetime.now(timezone.utc).isoformat(), control_mode='position',
                    position_method='wrist_surface_fit',
                    automatic_preview='--automatic-preview' in commands.get('controller', []),
                    commands=commands, status='starting')

    def save_metadata():
        temporary = session/'session.tmp'
        temporary.write_text(json.dumps(metadata, indent=2)+'\n')
        temporary.replace(session/'session.json')

    def request_stop(signum, _frame):
        stop['signal'] = signum

    save_metadata()
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, request_stop)
        with ExitStack() as stack:
            selector = stack.enter_context(selectors.DefaultSelector())
            try:
                for name, command in commands.items():
                    log = stack.enter_context((session/f'{name}.log').open('xb'))
                    process = subprocess.Popen(command, cwd=root,
                        env=isolated_environment(command[0], root),
                        stdin=subprocess.DEVNULL if name == 'vision' else None,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0,
                        start_new_session=True)
                    children[name] = process
                    os.set_blocking(process.stdout.fileno(), False)
                    selector.register(process.stdout, selectors.EVENT_READ, (name, log))
                    if stop['signal'] is not None:
                        break
                metadata.update(status='running', child_pids={name: p.pid for name, p in children.items()})
                save_metadata()
                while True:
                    if stop['signal'] is not None:
                        reason, returncode = 'signal_'+signal.Signals(stop['signal']).name, 0
                        break
                    exited = [(name, process.poll()) for name, process in children.items() if process.poll() is not None]
                    if exited:
                        failed = next(((name, code) for name, code in exited if code != 0), None)
                        name, code = failed or exited[0]
                        reason, returncode = name+('_failed' if code else '_exited'), 1 if code else 0
                        break
                    if duration is not None and time.monotonic()-started >= duration:
                        reason, returncode = 'duration_elapsed', 0
                        break
                    pump_output(selector, .05)
            finally:
                stop_children(children, selector, cleanup_grace_s, cleanup_terminate_s)
                for process in children.values():
                    if process.stdout and not process.stdout.closed:
                        process.stdout.close()
    except BaseException as error:
        metadata['error'] = str(error)
        raise
    finally:
        metadata.update(status='finished', reason=reason, returncode=returncode,
                        elapsed_s=time.monotonic()-started,
                        child_pids={name: p.pid for name, p in children.items()},
                        child_returncodes={name: p.returncode for name, p in children.items()})
        try:
            save_metadata()
        finally:
            for sig, previous_handler in previous.items():
                signal.signal(sig, previous_handler)
    return returncode


def run(args):
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        raise ValueError('Duration must be finite and positive')
    args.detector_repo = args.detector_repo.expanduser().resolve()
    detector = args.detector_repo/'example/vector_retargeting/single_hand_detector.py'
    if not detector.is_file():
        raise ValueError('Upstream detector not found: '+str(detector))
    for name in ('.venv-core', '.venv-vision'):
        python = ROOT/name/'bin/python'
        if not python.is_file() or not os.access(python, os.X_OK):
            raise ValueError('Prepared isolated Python environment is required: '+str(python))
    output_root = args.out.expanduser()
    if not output_root.is_absolute():
        output_root = ROOT/output_root
    session = output_root/(datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
    session.mkdir(parents=True, exist_ok=False)
    commands = build_commands(args, session)
    if args.manual_control:
        print('SO101 SIMULATION ONLY — position mode; starts RELEASED.', flush=True)
        print('E engage / R release / X stop / C reset / Q quit. Terminal commands also work.', flush=True)
        print('Tracking loss requires release and re-engage.', flush=True)
    else:
        print('SO101 AUTOMATIC VISUAL PREVIEW — simulation only; no physical robot.', flush=True)
        print('Space pause/resume / Q quit. Tracking loss freezes; fresh tracking re-anchors and resumes.', flush=True)
    print('Session: '+str(session), flush=True)
    result = supervise(commands, session, duration=args.duration)
    print('Session ended. Complete logs: '+str(session), flush=True)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--detector-repo', type=Path, default=Path('/home/bx/repos/dex-retargeting'))
    parser.add_argument('--hand', choices=['Right', 'Left'], default='Right')
    parser.add_argument('--depth-size', choices=['640x480', '640x360', '480x270', '424x240'],
                        default='640x360', help='Native depth profile experiment; color/detector stay 640x480')
    parser.add_argument('--out', type=Path, default=Path('outputs/hand-follow'),
                        help='Parent directory; every launch creates a unique session beneath it')
    parser.add_argument('--headless', action='store_true', help='Disable windows; still uses the D455')
    parser.add_argument('--duration', type=float, help='End both workers after this many seconds')
    parser.add_argument('--record-images', action='store_true', help='Also record complete RGB/depth arrays')
    parser.add_argument('--record-stereo', action='store_true',
                        help='Record native depth and both infrared streams for depth-hole diagnosis (includes RGB/depth)')
    parser.add_argument('--manual-control', action='store_true',
                        help='Use explicit engage/release behavior instead of automatic visual preview')
    sys.exit(run(parser.parse_args()))
