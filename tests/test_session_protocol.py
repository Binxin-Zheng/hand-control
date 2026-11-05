import numpy as np
import pytest
from conftest import obs,state
from anyteleop_so101.protocol import HandObservation
from anyteleop_so101.session import TeleopSession
from anyteleop_so101.servo import PositionPriorityServo
from anyteleop_so101.mapping import RelativeMapper

def start(s,q): return s.tick(obs(),state(q),now_s=1,dt=.01,clutch=True)

def test_protocol_round_trip():
    import json
    a=obs(); b=HandObservation.from_dict(json.loads(a.to_json()))
    assert a.to_json()==b.to_json()

def test_wrong_clock_rejected():
    d=obs().as_dict();d['clock']='wall_time'
    with pytest.raises(ValueError): HandObservation.from_dict(d)

def test_zero_jump_at_engage(session,home_q):
    d=start(session,home_q); assert d.reason=='engaged_no_jump'
    assert np.allclose(d.q_command,home_q)

def test_repeated_frame_valid_at_faster_control_rate(session,home_q):
    start(session,home_q)
    d=session.tick(obs(),state(home_q,1.01),now_s=1.01,dt=.01,clutch=True)
    assert d.state=='ACTIVE'

def test_tracking_loss_requires_explicit_release(session,home_q):
    start(session,home_q)
    d=session.tick(None,state(home_q,1.01),now_s=1.01,dt=.01,clutch=True)
    assert d.state=='PAUSED' and d.q_command is None
    d=session.tick(obs(1.02,2),state(home_q,1.02),now_s=1.02,dt=.01,clutch=True)
    assert d.reason=='release_then_engage_required'
    session.tick(obs(1.03,3),state(home_q,1.03),now_s=1.03,dt=.01,clutch=False)
    d=session.tick(obs(1.04,4,p=[.5,.2,.8]),state(home_q,1.04),now_s=1.04,dt=.01,clutch=True)
    assert d.reason=='engaged_no_jump' and np.allclose(d.q_command,home_q)

@pytest.mark.parametrize('kind',['stale','future','track','frame','jump','sequence','predicted','changed_duplicate'])
def test_bad_observation_pauses(session,home_q,kind):
    start(session,home_q); o=obs(1.01,2)
    if kind=='stale': o=obs(.5,2)
    elif kind=='future': o=obs(2.,2)
    elif kind=='track': o=obs(1.01,2,track='right:1')
    elif kind=='frame':
        d=o.as_dict();d['frame_id']='mirrored_camera';o=HandObservation.from_dict(d)
    elif kind=='jump': o=obs(1.01,2,p=[1,0,.7])
    elif kind=='sequence': o=obs(1.01,0)
    elif kind=='predicted': o=obs(1.01,2,pe='predicted')
    elif kind=='changed_duplicate': o=obs(1.,1,p=[.01,0,.7])
    d=session.tick(o,state(home_q,1.01),now_s=1.01,dt=.01,clutch=True)
    assert d.state=='PAUSED' and d.q_command is None

def test_target_backlog_not_accumulated(session,home_q):
    start(session,home_q)
    # Plausible movement over 0.1 s, but greater than the allowed robot target backlog.
    d=session.tick(obs(1.1,2,p=[.07,0,.7]),state(home_q,1.1),now_s=1.1,dt=.01,clutch=True)
    assert d.reason=='target_backlog_requires_reengage'

def test_estop_latched(session,home_q):
    start(session,home_q)
    d=session.tick(obs(),state(home_q),now_s=1.,dt=.01,clutch=True,estop=True)
    assert d.state=='ESTOP'
    d=session.tick(obs(),state(home_q),now_s=1.,dt=.01,clutch=True,reset_estop=True)
    assert d.state=='ESTOP'
    d=session.tick(obs(),state(home_q),now_s=1.,dt=.01,clutch=False,reset_estop=True)
    assert d.state=='IDLE'

def test_synthetic_never_allowed_by_default(chain,home_q):
    s=TeleopSession(PositionPriorityServo(chain),RelativeMapper(np.eye(3)))
    assert start(s,home_q).reason=='unsupported_pose_evidence'

def test_stale_robot_feedback(session,home_q):
    d=session.tick(obs(2,2),state(home_q,1.),now_s=2.,dt=.01,clutch=True)
    assert d.reason=='stale_robot_feedback'

def test_aperture_prediction_not_driving_gripper(session,home_q):
    start(session,home_q)
    o=obs(1.01,2,ae='predicted'); d=session.tick(o,state(home_q,1.01),now_s=1.01,dt=.01,clutch=True)
    assert d.aperture_command==.5

def test_monotonic_clock_rollback_pauses(session,home_q):
    start(session,home_q)
    d=session.tick(obs(.99,2),state(home_q,.99),now_s=.99,dt=.01,clutch=True)
    assert d.state=='PAUSED' and d.reason=='host_clock_moved_backwards'
    assert d.q_command is None
