"""Visual-only automatic recovery drives the unchanged supervisor explicitly."""
import importlib.util
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from anyteleop_so101.mapping import RelativeMapper
from anyteleop_so101.runtime.automatic_preview import AutomaticPreview
from anyteleop_so101.servo import PositionPriorityServo
from anyteleop_so101.session import TeleopSession
from conftest import obs, state


def make_session(chain):
    return TeleopSession(PositionPriorityServo(chain), RelativeMapper(np.eye(3)),
                         allow_synthetic=True, control_mode='position')


def tick(preview, session, observation, q, now, receiver_error=None, receiver_error_kind=None, **events):
    clutch = preview.clutch_for_tick(observation, now_s=now, max_age_s=session.max_age,
                                    camera_frame=session.camera_frame, allow_synthetic=True,
                                    receiver_error=receiver_error, receiver_error_kind=receiver_error_kind)
    decision = session.tick(observation, state(q, now), now_s=now, dt=.02, clutch=clutch, **events)
    preview.observe(decision)
    return clutch, decision


@pytest.mark.parametrize('loss', ['missing', 'stale', 'identity'])
def test_automatic_recovery_releases_then_anchors_both_poses_from_measured_state(chain, home_q, loss):
    preview = AutomaticPreview(); session = make_session(chain)
    clutch, decision = tick(preview, session, obs(1., 1), home_q, 1.)
    assert not clutch and decision.q_command is None
    assert tick(preview, session, obs(1.02, 2), home_q, 1.02)[1].state == 'ACTIVE'
    lost = {'missing': None, 'stale': obs(.5, 3), 'identity': obs(1.04, 3, track='new')}[loss]
    decision = tick(preview, session, lost, home_q, 1.04)[1]
    assert decision.reason in preview.RECOVERABLE and decision.q_command is None
    moved_q = home_q+np.array([.02, .01, 0, 0, .02])
    clutch, released = tick(preview, session, obs(1.06, 4, p=[.2, .3, .6], track='new'), moved_q, 1.06)
    assert not clutch and released.reason == 'clutch_released'
    resumed = tick(preview, session, obs(1.08, 5, p=[.2, .3, .6], track='new'), moved_q, 1.08)[1]
    assert resumed.reason == 'engaged_no_jump'
    np.testing.assert_array_equal(resumed.q_command, moved_q)
    expected = chain.fk(moved_q)
    np.testing.assert_allclose(resumed.desired_pose.p, expected.p)
    np.testing.assert_allclose(resumed.desired_pose.R, expected.R)
    np.testing.assert_allclose(session.mapper.hand0.p, [.2, .3, .6])


@pytest.mark.parametrize('observation', [None, obs(.1), obs(pe='predicted'), replace(obs(), frame_id='other')])
def test_initial_preview_waits_for_fresh_supported_position_without_commands(chain, home_q, observation):
    preview = AutomaticPreview(); session = make_session(chain)
    for now in (1., 1.02, 1.04):
        clutch, decision = tick(preview, session, observation, home_q, now)
        assert not clutch and decision.q_command is None
    assert preview.as_dict()['state'] == 'WAITING_FOR_HAND'


@pytest.mark.parametrize('fault', ['collision_requires_reengage', 'target_backlog_requires_reengage',
                                  'control_cycle_overrun', 'invalid_clock_or_dt', 'implausible_pose_jump'])
def test_safety_faults_require_deliberate_resume_then_release_before_latch(chain, home_q, fault):
    preview = AutomaticPreview(); session = make_session(chain)
    tick(preview, session, obs(1., 1), home_q, 1.)
    tick(preview, session, obs(1.02, 2), home_q, 1.02)
    preview.observe(session._stop('PAUSED', fault))
    for index in range(3):
        now = 1.04+index*.02
        assert tick(preview, session, obs(now, index+3), home_q, now)[1].q_command is None
        assert preview.fault == fault
    preview.command('toggle_pause')
    clutch, released = tick(preview, session, obs(1.12, 7), home_q, 1.12)
    assert not clutch and released.reason == 'clutch_released'
    assert tick(preview, session, obs(1.14, 8), home_q, 1.14)[1].reason == 'engaged_no_jump'


def test_space_pause_and_resume_cannot_clear_estop(chain, home_q):
    preview = AutomaticPreview(); session = make_session(chain)
    tick(preview, session, obs(1., 1), home_q, 1.)
    tick(preview, session, obs(1.02, 2), home_q, 1.02)
    preview.command('toggle_pause')
    assert tick(preview, session, obs(1.04, 3), home_q, 1.04)[1].q_command is None
    assert preview.as_dict()['state'] == 'PAUSED'
    preview.command('toggle_pause')
    assert not tick(preview, session, obs(1.06, 4), home_q, 1.06)[0]
    assert tick(preview, session, obs(1.08, 5), home_q, 1.08)[1].state == 'ACTIVE'
    assert tick(preview, session, obs(1.10, 6), home_q, 1.10, estop=True)[1].state == 'ESTOP'
    preview.command('toggle_pause')
    assert tick(preview, session, obs(1.12, 7), home_q, 1.12)[1].state == 'ESTOP'
    preview.command('reset')
    assert tick(preview, session, obs(1.14, 8), home_q, 1.14, reset_estop=True)[1].reason == 'estop_reset_requires_new_engage'
    assert tick(preview, session, obs(1.16, 9), home_q, 1.16)[1].q_command is None
    preview.command('toggle_pause')
    assert not tick(preview, session, obs(1.18, 10), home_q, 1.18)[0]
    assert tick(preview, session, obs(1.20, 11), home_q, 1.20)[1].reason == 'engaged_no_jump'


@pytest.mark.parametrize('receiver_error', ['Out of order input', 'Vision packet too large',
                                           'Wrong vision envelope', 'Expecting value: line1'])
def test_protocol_errors_are_not_automatic_tracking_recovery(chain, home_q, receiver_error):
    preview = AutomaticPreview(); session = make_session(chain)
    tick(preview, session, obs(1., 1), home_q, 1.)
    tick(preview, session, obs(1.02, 2), home_q, 1.02)
    assert tick(preview, session, None, home_q, 1.04, receiver_error=receiver_error)[1].q_command is None
    assert preview.fault == 'vision_input_rejected: '+receiver_error
    for now in (1.06, 1.08):
        assert tick(preview, session, obs(now, 4), home_q, now)[1].q_command is None
        assert preview.fault == 'vision_input_rejected: '+receiver_error
    preview.command('toggle_pause')
    assert not tick(preview, session, obs(1.10, 5), home_q, 1.10)[0]
    assert tick(preview, session, obs(1.12, 6), home_q, 1.12)[1].reason == 'engaged_no_jump'


@pytest.mark.parametrize('receiver_error,kind', [('Stale/future vision input', 'stale'),
    ("[Errno 2] No such file or directory: '/fresh/session/vision.json'", 'missing')])
def test_absent_or_stale_file_waits_without_latching_protocol_fault(chain, home_q, receiver_error, kind):
    preview = AutomaticPreview(); session = make_session(chain)
    for now in (1., 1.02):
        assert tick(preview, session, None, home_q, now, receiver_error=receiver_error,
                    receiver_error_kind=kind)[1].q_command is None
        assert preview.fault is None
    assert tick(preview, session, obs(1.04, 1), home_q, 1.04)[1].reason == 'engaged_no_jump'


def test_future_camera_clock_is_not_automatic_stale_recovery(chain, home_q):
    preview = AutomaticPreview(); session = make_session(chain)
    tick(preview, session, None, home_q, 1., receiver_error='Stale/future vision input',
         receiver_error_kind='future')
    assert preview.fault == 'vision_input_rejected: Stale/future vision input'
    assert tick(preview, session, obs(1.02, 1), home_q, 1.02)[1].q_command is None


def test_future_observation_is_latched_even_without_transport(chain, home_q):
    preview = AutomaticPreview(); session = make_session(chain)
    tick(preview, session, obs(1., 1), home_q, 1.)
    tick(preview, session, obs(1.02, 2), home_q, 1.02)
    assert tick(preview, session, obs(2., 3), home_q, 1.04)[1].q_command is None
    assert preview.fault == 'future_observation_clock'
    assert tick(preview, session, obs(1.06, 4), home_q, 1.06)[1].q_command is None


@pytest.mark.parametrize('backend,mode', [('robot', 'position'), ('sim', 'full_pose')])
def test_preview_rejects_invalid_backend_or_mode_before_reading_config_or_opening_world(backend, mode):
    path = Path(__file__).parents[1]/'tools/run_teleop.py'
    spec = importlib.util.spec_from_file_location('automatic_preview_entry_test', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    # No config attribute is provided: this refusal must happen before even
    # reading hardware configuration, constructing a world or opening a worker.
    with pytest.raises(ValueError, match='cannot control hardware'):
        module.run(SimpleNamespace(automatic_preview=True, backend=backend, control_mode=mode))


def test_automatic_preview_rejects_optional_udp_before_opening_anything():
    path = Path(__file__).parents[1]/'tools/run_teleop.py'
    spec = importlib.util.spec_from_file_location('automatic_preview_udp_test', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    with pytest.raises(ValueError, match='latest-file IPC'):
        module.run(SimpleNamespace(automatic_preview=True, backend='sim', control_mode='position', vision_port=0))
