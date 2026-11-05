import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from anyteleop_so101.geometry import Pose,rotation_error
from anyteleop_so101.servo import PositionPriorityServo

def test_exact_arm_joint_names(chain):
    assert chain.names==('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll')
    assert chain.n==5

@pytest.mark.parametrize('seed',[0,1,2,3,4])
def test_analytic_jacobian_vs_finite_difference(chain,seed):
    q=np.random.default_rng(seed).uniform(-.6,.6,5); T,J=chain.fk_jacobian(q); eps=1e-7
    numeric=np.zeros_like(J)
    for i in range(5):
        qp=q.copy(); qp[i]+=eps; Tp=chain.fk(qp)
        numeric[:3,i]=(Tp.p-T.p)/eps
        numeric[3:,i]=rotation_error(Tp.R,T.R)/eps
    assert np.allclose(J,numeric,atol=2e-7)

def test_reachable_goal_converges(chain,home_q):
    s=PositionPriorityServo(chain); q=home_q.copy(); goal=chain.fk(q+[.05,.03,-.04,.04,.05]); velocity=np.zeros(5)
    initial=np.linalg.norm(goal.p-chain.fk(q).p)
    for _ in range(300):
        result=s.step(q,goal,.01,velocity); q=result.q_next; velocity=result.velocity
    assert result.position_error_m<.001 and result.position_error_m<initial*.1
    assert result.primary_leak_m_s<1e-7

def test_limits_acceleration_and_primary_priority(chain,home_q):
    s=PositionPriorityServo(chain); T=chain.fk(home_q)
    goal=Pose(T.p+[.01,-.01,.01],Rotation.from_euler('xyz',[1,.5,.8]).as_matrix()@T.R)
    r=s.step(home_q,goal,.01,np.zeros(5))
    assert np.max(np.abs(r.velocity))<=.04+1e-7
    assert r.primary_leak_m_s<1e-7
    assert np.all(r.q_next>chain.lower) and np.all(r.q_next<chain.upper)

def test_incompatible_orientation_is_reported(chain,home_q):
    T=chain.fk(home_q); s=PositionPriorityServo(chain)
    goal=Pose(T.p,Rotation.from_euler('xyz',[.8,.6,.5]).as_matrix()@T.R)
    r=s.step(home_q,goal,.01)
    assert r.orientation_error_rad>.1; assert r.primary_leak_m_s<1e-7

@pytest.mark.parametrize('dt',[0,-.1,.2,float('nan')])
def test_bad_dt_rejected(chain,home_q,dt):
    with pytest.raises(ValueError): PositionPriorityServo(chain).step(home_q,chain.fk(home_q),dt)

def test_outside_soft_limits_rejected(chain):
    with pytest.raises(ValueError): PositionPriorityServo(chain).step(chain.upper,chain.fk(chain.upper),.01)
