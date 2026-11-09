"""Exercise launcher process lifecycle using harmless children, never a camera."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import sys
from types import SimpleNamespace

import pytest


@pytest.fixture
def launcher():
    path = Path(__file__).parents[1]/'tools/hand_follow.py'
    spec = importlib.util.spec_from_file_location('hand_follow_launcher_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_commands_are_explicit_position_simulation_and_use_fresh_shared_ipc(launcher, tmp_path):
    args = SimpleNamespace(detector_repo=Path('/existing/upstream'), hand='Left', headless=False,
                           record_images=True, manual_control=False)
    commands = launcher.build_commands(args, tmp_path, Path('/project'))
    vision, core = commands['vision'], commands['controller']
    assert vision[0] == '/project/.venv-vision/bin/python'
    assert core[0] == '/project/.venv-core/bin/python'
    assert core[core.index('--backend')+1] == 'sim'
    assert core[core.index('--control-mode')+1] == 'position'
    assert '--automatic-preview' in core
    assert vision[vision.index('--position-method')+1] == 'wrist_surface_fit'
    assert vision[vision.index('--depth-size')+1] == '640x360'
    assert vision[vision.index('--vision-file')+1] == core[core.index('--vision-file')+1] == str(tmp_path/'vision.json')
    assert {'--display', '--mirror-preview', '--record-images'} <= set(vision)
    assert not {'--synthetic', '--execute', '--enable-torque'} & set(core+vision)
    args.headless, args.record_images = True, False
    headless = launcher.build_commands(args, tmp_path)
    assert '--headless' in headless['controller'] and '--display' not in headless['vision']
    assert '--record-images' not in headless['vision']
    args.manual_control = True
    manual = launcher.build_commands(args, tmp_path)
    assert '--automatic-preview' not in manual['controller']
    assert manual['controller'][manual['controller'].index('--backend')+1] == 'sim'
    args.record_stereo = True
    stereo = launcher.build_commands(args, tmp_path)
    assert '--record-stereo' in stereo['vision'] and '--record-stereo' not in stereo['controller']
    args.depth_size = '424x240'
    nearer = launcher.build_commands(args, tmp_path)
    assert nearer['vision'][nearer['vision'].index('--depth-size')+1] == '424x240'
    assert nearer['controller'] == stereo['controller']


def test_child_environments_exclude_parent_python_and_conda_paths(launcher, tmp_path, monkeypatch):
    monkeypatch.setenv('PYTHONPATH', '/old/ros/python')
    monkeypatch.setenv('PYTHONHOME', '/other/python')
    monkeypatch.setenv('CONDA_PREFIX', '/other/conda')
    env = launcher.isolated_environment('/project/.venv-vision/bin/python', tmp_path)
    assert env['PYTHONPATH'] == str(tmp_path/'src')
    assert env['VIRTUAL_ENV'] == '/project/.venv-vision'
    assert env['PATH'].split(os.pathsep)[0] == '/project/.venv-vision/bin'
    assert env['PYTHONNOUSERSITE'] == '1'
    assert 'PYTHONHOME' not in env and 'CONDA_PREFIX' not in env


STUBBORN = """import signal,time
signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
print('vision ready',flush=True)
while True: time.sleep(.02)
"""


@pytest.mark.parametrize('controller_status', [0, 7])
def test_child_exit_stops_and_reaps_other_child_with_bounded_escalation(launcher, tmp_path, controller_status):
    commands = dict(vision=[sys.executable, '-c', STUBBORN],
                    controller=[sys.executable, '-c',
                                f"import time;time.sleep(.15);print('controller exit',flush=True);raise SystemExit({controller_status})"])
    result = launcher.supervise(commands, tmp_path, duration=3, root=tmp_path,
                                cleanup_grace_s=.05, cleanup_terminate_s=.05)
    metadata = json.loads((tmp_path/'session.json').read_text())
    assert result == (1 if controller_status else 0)
    assert metadata['reason'] == ('controller_failed' if controller_status else 'controller_exited')
    assert metadata['child_returncodes'] == {'vision': -signal.SIGKILL, 'controller': controller_status}
    assert 'vision ready' in (tmp_path/'vision.log').read_text()
    assert 'controller exit' in (tmp_path/'controller.log').read_text()
    for pid in metadata['child_pids'].values():
        with pytest.raises(ProcessLookupError):
            os.killpg(pid, 0)


def test_duration_ends_both_workers_without_restart(launcher, tmp_path):
    child = [sys.executable, '-c', "import time;print('started',flush=True);time.sleep(60)"]
    result = launcher.supervise({'vision': child, 'controller': child}, tmp_path,
                                duration=.15, root=tmp_path, cleanup_grace_s=.2, cleanup_terminate_s=.05)
    metadata = json.loads((tmp_path/'session.json').read_text())
    assert result == 0 and metadata['reason'] == 'duration_elapsed'
    assert all(code is not None for code in metadata['child_returncodes'].values())
    assert (tmp_path/'vision.log').read_text().count('started') == 1
    assert (tmp_path/'controller.log').read_text().count('started') == 1


def test_second_child_start_failure_still_reaps_first(launcher, tmp_path, monkeypatch):
    original = launcher.subprocess.Popen
    processes = []
    def spawn(command, **kwargs):
        if processes:
            raise OSError('test controller startup failure')
        process = original(command, **kwargs)
        processes.append(process)
        return process
    monkeypatch.setattr(launcher.subprocess, 'Popen', spawn)
    child = [sys.executable, '-c', 'import time;time.sleep(60)']
    with pytest.raises(OSError, match='test controller startup failure'):
        launcher.supervise({'vision': child, 'controller': child}, tmp_path, root=tmp_path,
                            cleanup_grace_s=.1, cleanup_terminate_s=.05)
    assert processes[0].poll() is not None
    metadata = json.loads((tmp_path/'session.json').read_text())
    assert metadata['reason'] == 'startup_failed' and 'startup failure' in metadata['error']
