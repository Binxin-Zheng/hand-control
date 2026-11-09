"""Offline supervisor emitting candidate joint targets; it NEVER sends motor commands.
PAUSED/IDLE/ESTOP outputs q_command=None. A real driver must actively stop via its
local watchdog, NOT keep chasing its last distant setpoint and NOT disable torque
blindly. Collision guarding and deterministic physical stopping are external gates.
"""
from dataclasses import dataclass, field
import numpy as np
from .geometry import Pose, finite_array, rotation_error
from .mapping import RelativeMapper,GripperPickup
from .protocol import HandObservation
from .servo import PositionPriorityServo

@dataclass(frozen=True)
class RobotState:
    timestamp_s: float
    q: np.ndarray
    aperture: float

@dataclass
class Decision:
    state: str
    reason: str
    q_command: np.ndarray | None=None
    aperture_command: float | None=None
    desired_pose: Pose | None=None
    diagnostics: dict=field(default_factory=dict)


class TeleopSession:
    def __init__(self,servo:PositionPriorityServo,mapper:RelativeMapper,
                 camera_frame='camera_color_optical',max_age_s=.15,max_backlog_m=.06,
                 allow_synthetic=False, control_mode='full_pose'):
        if control_mode not in ('full_pose', 'position'):
            raise ValueError('Choose explicit full_pose or position control mode')
        self.control_mode=control_mode
        self.servo=servo; self.mapper=mapper; self.camera_frame=camera_frame
        self.max_age=max_age_s; self.max_backlog=max_backlog_m; self.allow_synthetic=allow_synthetic
        if not np.isfinite([max_age_s,max_backlog_m]).all() or min(max_age_s,max_backlog_m)<=0: raise ValueError('Invalid policy')
        self.gripper=GripperPickup(); self.state='IDLE'; self.need_release=False
        self.last_obs=None; self.track=None; self.velocity=np.zeros(servo.chain.n); self.last_now=None

    def _stop(self,state,reason,need_release=True):
        self.state=state; self.need_release=need_release; self.mapper.reset()
        self.velocity=np.zeros(self.servo.chain.n); self.last_obs=None; self.track=None
        return Decision(state,reason)

    def tick(self,observation:HandObservation|None,robot:RobotState,*,now_s,dt,
             clutch,ready=True,estop=False,reset_estop=False):
        if estop: return self._stop('ESTOP','emergency_stop')
        if self.state=='ESTOP':
            if reset_estop and not clutch:
                return self._stop('IDLE','estop_reset_requires_new_engage',False)
            return Decision('ESTOP','latched_emergency_stop')
        if not np.isfinite([now_s,dt,robot.timestamp_s,robot.aperture]).all() or not 0<dt<=.05:
            return self._stop('PAUSED','invalid_clock_or_dt')
        if self.last_now is not None and now_s < self.last_now:
            return self._stop('PAUSED','host_clock_moved_backwards')
        self.last_now=now_s
        if not 0<=robot.aperture<=1:
            return self._stop('PAUSED','invalid_robot_aperture')
        try: q=finite_array(robot.q,(self.servo.chain.n,),'robot q')
        except ValueError: return self._stop('PAUSED','invalid_robot_state')
        if robot.timestamp_s>now_s+.005 or now_s-robot.timestamp_s>self.max_age:
            return self._stop('PAUSED','stale_robot_feedback')
        if not ready: return self._stop('PAUSED','system_not_ready')
        if not clutch: return self._stop('IDLE','clutch_released',False)
        if self.need_release: return Decision('PAUSED','release_then_engage_required')
        obs=observation
        if obs is None or obs.pose is None:
            return self._stop('PAUSED','tracking_unavailable')
        if obs.frame_id!=self.camera_frame:
            return self._stop('PAUSED','coordinate_frame_changed')
        if obs.timestamp_s>now_s+.005 or now_s-obs.timestamp_s>self.max_age:
            return self._stop('PAUSED','stale_or_future_observation')
        good_p={'rgbd_fused','stereo'}; good_r={'rgb_model','rgbd_fit','stereo'}
        if self.allow_synthetic: good_p.add('synthetic'); good_r.add('synthetic')
        if obs.position_evidence not in good_p or (self.control_mode=='full_pose' and obs.orientation_evidence not in good_r):
            return self._stop('PAUSED','unsupported_pose_evidence')
        if self.last_obs is not None:
            prev=self.last_obs
            if obs.track_id!=self.track: return self._stop('PAUSED','hand_identity_changed')
            if obs.sequence<prev.sequence or obs.timestamp_s<prev.timestamp_s:
                return self._stop('PAUSED','out_of_order_observation')
            if obs.sequence==prev.sequence:
                if obs.to_json()!=prev.to_json(): return self._stop('PAUSED','sequence_reused_with_changed_data')
            else:
                elapsed=obs.timestamp_s-prev.timestamp_s
                if elapsed<=0: return self._stop('PAUSED','nonincreasing_capture_time')
                dp=np.linalg.norm(obs.pose.p-prev.pose.p)
                dr=np.linalg.norm(rotation_error(obs.pose.R,prev.pose.R))
                if dp>.015+1.5*elapsed or (self.control_mode=='full_pose' and dr>.12+8*elapsed):
                    return self._stop('PAUSED','implausible_pose_jump')
        self.last_obs=obs
        actual=self.servo.chain.fk(q)
        if self.state!='ACTIVE':
            if np.any(q<self.servo.chain.lower+self.servo.margin) or np.any(q>self.servo.chain.upper-self.servo.margin):
                return self._stop('PAUSED','outside_soft_limits_at_engage')
            self.mapper.engage(obs.pose,actual); self.gripper.engage(robot.aperture)
            self.track=obs.track_id; self.state='ACTIVE'; self.velocity[:]=0
            return Decision('ACTIVE','engaged_no_jump',q.copy(),robot.aperture,actual,
                            {'gripper_pickup':False,'control_mode':self.control_mode})
        target=self.mapper.target(obs.pose)
        if self.control_mode=='position':
            # Named XYZ mode holds the measured orientation latched at engagement.
            # It never changes full-pose behavior or integrates an orientation bias.
            target=Pose(target.p,self.mapper.robot0.R)
        if np.linalg.norm(target.p-actual.p)>self.max_backlog:
            return self._stop('PAUSED','target_backlog_requires_reengage')
        try: step=self.servo.step(q,target,dt,self.velocity)
        except ValueError as e: return self._stop('PAUSED','servo_rejected: '+str(e))
        self.velocity=step.velocity
        grip=obs.aperture if obs.aperture_evidence in ({'rgb_model','synthetic'} if self.allow_synthetic else {'rgb_model'}) else None
        jaw=self.gripper.update(grip,dt)
        return Decision('ACTIVE','tracking',step.q_next,jaw,target,
            {'position_error_m':step.position_error_m,'orientation_error_rad':step.orientation_error_rad,
             'primary_leak_m_s':step.primary_leak_m_s,'sigma_min':step.sigma_min,
             'control_mode':self.control_mode,
             'gripper_pickup':self.gripper.picked_up,'secondary_success':step.secondary_success,
             'collision_checked':False,'hardware_authorized':False})
