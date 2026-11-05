from pathlib import Path
import numpy as np
import pytest
from anyteleop_so101.kinematics import URDFChain
from anyteleop_so101.geometry import Pose
from anyteleop_so101.servo import PositionPriorityServo
from anyteleop_so101.mapping import RelativeMapper
from anyteleop_so101.session import TeleopSession,RobotState
from anyteleop_so101.protocol import HandObservation

@pytest.fixture
def chain():
    return URDFChain(Path(__file__).parents[1]/'assets/so101_kinematics_only.urdf','base_link','gripper_link')

@pytest.fixture
def home_q(): return np.array([.1,-.3,.4,.15,.2])

@pytest.fixture
def session(chain):
    return TeleopSession(PositionPriorityServo(chain),RelativeMapper(np.eye(3)),allow_synthetic=True)

def hand_points(alpha=.7):
    p=np.zeros((21,3)); p[1]=[.015,.01,0]; p[2]=[.025,.015,0]
    p[3]=p[2]+[np.sin(alpha)*.02,np.cos(alpha)*.02,0]
    p[4]=p[2]+[np.sin(alpha)*.04,np.cos(alpha)*.04,0]
    for a,x,y in [(5,.025,.06),(9,0,.065),(13,-.015,.06),(17,-.03,.05)]:
        for k in range(4): p[a+k]=[x,y+k*.015,0]
    return p

def obs(t=1.,seq=1,p=None,R=None,track='right:0',**kwargs):
    return HandObservation(seq,t,track,'camera_color_optical',Pose(np.array([0.,0.,.7]) if p is None else p,np.eye(3) if R is None else R),.5,kwargs.get('pe','synthetic'),kwargs.get('re','synthetic'),kwargs.get('ae','synthetic'))

def state(q,t=1.,aperture=.5): return RobotState(t,np.array(q),aperture)
