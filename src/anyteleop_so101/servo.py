"""Position-priority differential IK baseline, NOT cuRobo and NOT a safety controller.
Two stages preserve the primary task at the LINEARIZED velocity level; finite-step
FK residual must still be checked. No collision or dynamic guarantee is provided.
"""
from dataclasses import dataclass
import numpy as np
from scipy.linalg import null_space
from scipy.optimize import lsq_linear, minimize, LinearConstraint
from .geometry import Pose, finite_array, rotation_error, cap_norm
from .kinematics import URDFChain

@dataclass(frozen=True)
class ServoResult:
    q_next: np.ndarray
    velocity: np.ndarray
    position_error_m: float
    orientation_error_rad: float
    primary_leak_m_s: float
    sigma_min: float
    secondary_success: bool


class PositionPriorityServo:
    def __init__(self,chain:URDFChain,velocity_limit=.8,acceleration_limit=4.,
                 position_gain=4.,rotation_gain=3.,max_linear_speed=.12,
                 max_angular_speed=1.,damping=.01,joint_margin=.01):
        self.chain=chain
        self.vmax=np.broadcast_to(np.asarray(velocity_limit,dtype=float),(chain.n,)).copy()
        self.amax=np.broadcast_to(np.asarray(acceleration_limit,dtype=float),(chain.n,)).copy()
        values=[position_gain,rotation_gain,max_linear_speed,max_angular_speed,damping,joint_margin]
        if not np.isfinite(self.vmax).all() or not np.isfinite(self.amax).all() or np.min(self.vmax)<=0 or np.min(self.amax)<=0 or not np.isfinite(values).all() or min(values)<=0:
            raise ValueError("Servo limits/gains must be finite and positive")
        self.kp=position_gain; self.kr=rotation_gain; self.max_v=max_linear_speed
        self.max_w=max_angular_speed; self.damping=damping; self.margin=joint_margin
        if np.any(chain.upper-chain.lower <= 2*joint_margin): raise ValueError("Margin erases joint range")

    def step(self,q,target:Pose,dt,previous_velocity=None):
        q=finite_array(q,(self.chain.n,),"q")
        if not np.isfinite(dt) or not 0<dt<=.05:
            raise ValueError("dt must be in (0,0.05]; missed cycles must not accumulate")
        prev=np.zeros_like(q) if previous_velocity is None else finite_array(previous_velocity,q.shape,"previous velocity")
        if np.any(q < self.chain.lower+self.margin-1e-10) or np.any(q > self.chain.upper-self.margin+1e-10):
            raise ValueError("Joint outside configured soft limits")
        actual,J=self.chain.fk_jacobian(q); A=J[:3]; B=J[3:]
        v=cap_norm(self.kp*(target.p-actual.p),self.max_v)
        w=cap_norm(self.kr*rotation_error(target.R,actual.R),self.max_w)
        lower=np.maximum.reduce([-self.vmax,(self.chain.lower+self.margin-q)/dt,prev-self.amax*dt])
        upper=np.minimum.reduce([self.vmax,(self.chain.upper-self.margin-q)/dt,prev+self.amax*dt])
        if np.any(lower>=upper): raise ValueError("Infeasible joint/acceleration bounds")
        singular_values=np.linalg.svd(A,compute_uv=False); sigma=float(singular_values[-1])
        damping=self.damping*(1+max(0.,.02-sigma)/.02)
        objective=np.vstack([A,damping*np.eye(self.chain.n)])
        rhs=np.r_[v,np.zeros(self.chain.n)]
        primary=lsq_linear(objective,rhs,bounds=(lower,upper),tol=1e-10,max_iter=100)
        if not primary.success: raise ValueError("Primary bounded IK failed")
        v1=primary.x; velocity=v1.copy(); ok=True
        N=null_space(A,rcond=1e-8)
        if N.shape[1]:
            C=B@N; d=w-B@v1; regularizer=1e-3
            f=lambda z: .5*np.sum((C@z-d)**2)+.5*regularizer*np.sum(z*z)
            jac=lambda z: C.T@(C@z-d)+regularizer*z
            secondary=minimize(f,np.zeros(N.shape[1]),jac=jac,method='SLSQP',
                constraints=[LinearConstraint(N,lower-v1,upper-v1)],
                options={'ftol':1e-10,'maxiter':80})
            candidate=v1+N@secondary.x
            ok=bool(secondary.success and np.all(candidate>=lower-1e-8) and np.all(candidate<=upper+1e-8))
            if ok: velocity=candidate
        leak=float(np.linalg.norm(A@(velocity-v1)))
        if leak>1e-7: raise ValueError("Secondary task changed primary task")
        q_next=q+dt*velocity
        if not np.isfinite(q_next).all() or np.any(q_next<self.chain.lower+self.margin-1e-8) or np.any(q_next>self.chain.upper-self.margin+1e-8):
            raise ValueError("Unsafe numerical output")
        reached=self.chain.fk(q_next)
        return ServoResult(q_next,velocity,float(np.linalg.norm(target.p-reached.p)),
            float(np.linalg.norm(rotation_error(target.R,reached.R))),leak,sigma,ok)
