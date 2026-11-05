"""Optional PyBullet plant and separate collision-check world; no real hardware.

Collision checks sample configurations, with a conservative clearance allowance
for interpolation. They are not a certified continuous collision detector.
"""
import math
import time
import numpy as np
from .model import ARM_NAMES, validate_model
from ..session import RobotState
from ..geometry import finite_array


class BulletWorld:
    def __init__(self, urdf, *, gui=False, table_z=-.02, clearance=.005,
                 sweep_step=.01, reach_bound=1.0):
        path = validate_model(urdf)
        if not np.isfinite([table_z, clearance, sweep_step, reach_bound]).all() or min(clearance, sweep_step, reach_bound) <= 0:
            raise ValueError('Invalid collision configuration')
        import pybullet as p
        self.p = p
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        if self.client < 0:
            raise RuntimeError('Cannot open PyBullet')
        try:
            self.body = p.loadURDF(str(path), useFixedBase=True,
                                  flags=p.URDF_USE_SELF_COLLISION, physicsClientId=self.client)
            self.info = {p.getJointInfo(self.body, i, physicsClientId=self.client)[1].decode():
                         p.getJointInfo(self.body, i, physicsClientId=self.client)
                         for i in range(p.getNumJoints(self.body, physicsClientId=self.client))}
            self.ids = [self.info[n][0] for n in ARM_NAMES]
            self.gripper = self.info['gripper'][0]
            self.gmin, self.gmax = self.info['gripper'][8:10]
            if self.gmax <= self.gmin:
                raise ValueError('Invalid gripper limits')
            self.clearance, self.sweep_step, self.reach_bound = clearance, sweep_step, reach_bound
            shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 1, .02], physicsClientId=self.client)
            self.table = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                                           basePosition=[0, 0, table_z-.02], physicsClientId=self.client)
            self.links = [-1] + [v[0] for v in self.info.values()]
            self.parents = {v[0]: v[16] for v in self.info.values()}
            if gui:
                p.resetDebugVisualizerCamera(.65, 45, -25, [0, 0, .15], physicsClientId=self.client)
            p.setGravity(0, 0, -9.81, physicsClientId=self.client)
            p.setTimeStep(.005, physicsClientId=self.client)
        except BaseException:
            p.disconnect(self.client)
            raise

    def set_configuration(self, q, aperture):
        q = finite_array(q, (5,), 'q')
        if not np.isfinite(aperture) or not 0 <= aperture <= 1:
            raise ValueError('Invalid aperture')
        for i, value in zip(self.ids, q):
            self.p.resetJointState(self.body, i, float(value), physicsClientId=self.client)
        self.p.resetJointState(self.body, self.gripper, self.gmin+aperture*(self.gmax-self.gmin),
                              physicsClientId=self.client)

    def read_state(self):
        values = self.p.getJointStates(self.body, self.ids+[self.gripper], physicsClientId=self.client)
        aperture = (values[-1][0]-self.gmin)/(self.gmax-self.gmin)
        return RobotState(time.monotonic(), np.array([v[0] for v in values[:5]]), float(np.clip(aperture, 0, 1)))

    def command(self, q, aperture):
        q = finite_array(q, (5,), 'q')
        if not np.isfinite(aperture) or not 0 <= aperture <= 1:
            raise ValueError('Invalid aperture')
        self.p.setJointMotorControlArray(self.body, self.ids+[self.gripper], self.p.POSITION_CONTROL,
            targetPositions=q.tolist()+[self.gmin+aperture*(self.gmax-self.gmin)],
            forces=[5]*6, physicsClientId=self.client)

    def stop(self):
        # Simulation holds measured state, discarding the previous distant target.
        state = self.read_state()
        self.p.setJointMotorControlArray(self.body, self.ids, self.p.POSITION_CONTROL,
            targetPositions=state.q.tolist(), forces=[5]*5, physicsClientId=self.client)

    def step(self):
        self.p.stepSimulation(physicsClientId=self.client)

    def check_segment(self, q_from, q_to, gripper_from, gripper_to):
        start = finite_array(q_from, (5,), 'q_from')
        end = finite_array(q_to, (5,), 'q_to')
        delta = np.r_[end-start, (gripper_to-gripper_from)*(self.gmax-self.gmin)]
        steps = max(1, math.ceil(float(np.max(np.abs(delta)))/self.sweep_step))
        if steps > 1000:
            return False
        # Bound displacement between samples assuming every link point is within
        # reach_bound of each ancestor axis. Site configuration must validate it.
        margin = self.clearance + self.reach_bound*float(np.sum(np.abs(delta)))/steps
        p = self.p
        for alpha in np.linspace(0, 1, steps+1):
            self.set_configuration(start+alpha*(end-start), gripper_from+alpha*(gripper_to-gripper_from))
            p.performCollisionDetection(physicsClientId=self.client)
            if p.getClosestPoints(self.body, self.table, margin, physicsClientId=self.client):
                return False
            for index, a in enumerate(self.links):
                for b in self.links[index+1:]:
                    if self.parents.get(a) == b or self.parents.get(b) == a:
                        continue  # URDF-connected neighbors share material at joints.
                    if p.getClosestPoints(self.body, self.body, margin, linkIndexA=a,
                                          linkIndexB=b, physicsClientId=self.client):
                        return False
        return True

    def close(self):
        self.p.disconnect(self.client)
