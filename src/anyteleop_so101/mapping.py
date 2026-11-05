"""Intent mapping independent of the camera and the motor SDK."""
import numpy as np
from .geometry import Pose, proper_rotation


class RelativeMapper:
    def __init__(self, R_base_camera, scale=1.):
        self.C=proper_rotation(R_base_camera)
        if not np.isfinite(scale) or scale<=0:
            raise ValueError("Translation scale must be positive")
        self.scale=float(scale); self.hand0=None; self.robot0=None

    def engage(self, hand: Pose, measured_robot: Pose):
        self.hand0=hand; self.robot0=measured_robot

    def reset(self):
        self.hand0=None; self.robot0=None

    def target(self, hand: Pose):
        if self.hand0 is None:
            raise RuntimeError("Must engage on a fresh trusted observation")
        p=self.robot0.p+self.scale*self.C@(hand.p-self.hand0.p)
        R=self.C@(hand.R@self.hand0.R.T)@self.C.T@self.robot0.R
        return Pose(p,R)


class GripperPickup:
    """Avoid automatic opening/closing on engage. All values: 0 closed, 1 open."""
    def __init__(self,tolerance=.08,max_rate=1.):
        if not (0<tolerance<.5) or max_rate<=0:
            raise ValueError("Invalid gripper policy")
        self.tolerance=tolerance; self.max_rate=max_rate
        self.value=0.; self.picked_up=False; self.previous_hand=None

    def engage(self,measured_aperture):
        if not np.isfinite(measured_aperture) or not 0<=measured_aperture<=1:
            raise ValueError("Invalid measured aperture")
        self.value=float(measured_aperture); self.picked_up=False; self.previous_hand=None

    def update(self,hand_aperture,dt):
        if not np.isfinite(dt) or dt<=0:
            raise ValueError("Invalid dt")
        if hand_aperture is None:
            return self.value
        h=float(hand_aperture)
        if not np.isfinite(h) or not 0<=h<=1:
            raise ValueError("Invalid hand aperture")
        if not self.picked_up:
            crossed=self.previous_hand is not None and (self.previous_hand-self.value)*(h-self.value)<=0
            self.picked_up=abs(h-self.value)<=self.tolerance or crossed
        self.previous_hand=h
        if self.picked_up:
            self.value += float(np.clip(h-self.value,-self.max_rate*dt,self.max_rate*dt))
        return self.value
