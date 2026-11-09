from dataclasses import replace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from anyteleop_so101.geometry import Pose
from anyteleop_so101.mapping import RelativeMapper
from anyteleop_so101.servo import PositionPriorityServo
from anyteleop_so101.session import TeleopSession
from conftest import obs, state


def make_session(chain, mode):
    return TeleopSession(PositionPriorityServo(chain), RelativeMapper(np.eye(3)),
                         allow_synthetic=True, control_mode=mode)


@pytest.mark.parametrize('mode', ['full_pose', 'position'])
def test_orientation_spike_behavior_is_an_explicit_mode_choice(chain, home_q, mode):
    session = make_session(chain, mode)
    initial = session.tick(obs(), state(home_q), now_s=1, dt=.01, clutch=True)
    changed = replace(obs(1.01, 2, p=[.001, 0, .7]),
                      pose=Pose([.001, 0, .7], Rotation.from_rotvec([2., 0, 0]).as_matrix()))
    result = session.tick(changed, state(home_q, 1.01), now_s=1.01, dt=.01, clutch=True)
    if mode == 'full_pose':
        assert result.reason == 'implausible_pose_jump'
    else:
        assert result.state == 'ACTIVE'
        np.testing.assert_allclose(result.desired_pose.R, initial.desired_pose.R)
        np.testing.assert_allclose(result.desired_pose.p-initial.desired_pose.p, [.001, 0, 0])


@pytest.mark.parametrize('fault', ['missing', 'predicted', 'identity', 'jump', 'stale', 'estop'])
def test_xyz_mode_retains_position_and_stop_guards(chain, home_q, fault):
    session = make_session(chain, 'position')
    session.tick(obs(), state(home_q), now_s=1., dt=.01, clutch=True)
    current = obs(1.01, 2)
    if fault == 'missing': current = None
    if fault == 'predicted': current = obs(1.01, 2, pe='predicted')
    if fault == 'identity': current = obs(1.01, 2, track='different')
    if fault == 'jump': current = obs(1.01, 2, p=[1., 0, .7])
    if fault == 'stale': current = obs(.5, 2)
    result = session.tick(current, state(home_q, 1.01), now_s=1.01, dt=.01,
                          clutch=True, estop=fault == 'estop')
    assert result.state in ('PAUSED', 'ESTOP') and result.q_command is None
    recovered = session.tick(obs(1.02, 3), state(home_q, 1.02), now_s=1.02, dt=.01, clutch=True)
    assert recovered.state != 'ACTIVE'


def test_xyz_reengagement_latches_new_measured_position_and_orientation(chain, home_q):
    session = make_session(chain, 'position')
    session.tick(obs(), state(home_q), now_s=1., dt=.01, clutch=True)
    session.tick(None, state(home_q, 1.01), now_s=1.01, dt=.01, clutch=False)
    moved_q = home_q+np.array([.02, .01, 0, 0, 0])
    result = session.tick(obs(1.02, 3, p=[.2, .3, .6]), state(moved_q, 1.02), now_s=1.02, dt=.01, clutch=True)
    expected = chain.fk(moved_q)
    np.testing.assert_allclose(result.desired_pose.p, expected.p)
    np.testing.assert_allclose(result.desired_pose.R, expected.R)
    np.testing.assert_allclose(result.q_command, moved_q)


def test_full_pose_remains_default_and_invalid_modes_rejected(chain):
    assert TeleopSession(PositionPriorityServo(chain), RelativeMapper(np.eye(3))).control_mode == 'full_pose'
    with pytest.raises(ValueError): make_session(chain, 'automatic_fallback')
