"""Adapter contracts. Explicit opt-in implementations live in runtime/ and tools/robot_worker.py."""
from typing import Protocol
import numpy as np
from .geometry import Pose
from .protocol import HandObservation
from .session import RobotState

class HandSource(Protocol):
    def latest(self) -> HandObservation | None: ...

class MotionBackend(Protocol):
    def update_goal(self,target:Pose) -> None: ...
    def step(self,state:RobotState,dt:float) -> np.ndarray: ...

class SweptCollisionGuard(Protocol):
    def check_segment(self,q_from:np.ndarray,q_to:np.ndarray,gripper_from:float,
                      gripper_to:float) -> bool:
        """True ONLY if checked against fresh full-link geometry and scene data."""
        ...

class RobotPort(Protocol):
    def read_state(self) -> RobotState: ...
    def send_validated_target(self,joints_rad:dict[str,float],gripper_aperture:float,
                              deadline_monotonic_s:float) -> None: ...
    def controlled_stop(self,reason:str) -> None: ...
