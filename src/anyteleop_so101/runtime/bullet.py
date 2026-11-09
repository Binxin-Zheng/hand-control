"""Optional PyBullet plant and separate collision-check world; no real hardware.

Collision checks sample configurations, with a conservative clearance allowance
for interpolation. They are not a certified continuous collision detector.
"""
import math
import time
import numpy as np
from .model import ARM_NAMES, validate_model
from .motion_bounds import BoundUnavailable, bounds_from_loaded_urdf, validated_aabb
from ..session import RobotState
from ..geometry import finite_array


class BulletWorld:
    def __init__(self, urdf, *, gui=False, table_z=-.02, clearance=.005,
                 sweep_step=.01, reach_bound=1.0, gripper_angle_range_rad=None):
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
            if gripper_angle_range_rad is not None:
                limits = finite_array(gripper_angle_range_rad, (2,), 'physical gripper angle range')
                if not self.gmin <= limits[0] < limits[1] <= self.gmax:
                    raise ValueError('Physical gripper angle range is outside URDF limits')
                self.gmin, self.gmax = map(float, limits)
            self.clearance, self.sweep_step, self.reach_bound = clearance, sweep_step, reach_bound
            shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 1, .02], physicsClientId=self.client)
            self.table = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                                           basePosition=[0, 0, table_z-.02], physicsClientId=self.client)
            self.links = [-1] + [v[0] for v in self.info.values()]
            self.parents = {v[0]: v[16] for v in self.info.values()}
            self.link_names = {-1: 'base_link', **{v[0]: v[12].decode() for v in self.info.values()}}
            self.self_pairs = tuple((a, b) for index, a in enumerate(self.links) for b in self.links[index+1:]
                                    if self.parents.get(a) != b and self.parents.get(b) != a)
            self.motion_bounds = None
            self.motion_bound_fallback_reason = None
            try:
                self.motion_bounds = bounds_from_loaded_urdf(path, p, self.body, self.client,
                    self.info, self.links, self.ids+[self.gripper], self.self_pairs)
            except (BoundUnavailable, ValueError, TypeError, KeyError, AttributeError) as exc:
                # Unsupported geometry/kinematics retains the existing global
                # envelope and every query. Never substitute a guessed radius.
                self.motion_bound_fallback_reason = str(exc)
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
        if not all(np.isfinite(g) and 0 <= g <= 1 for g in (gripper_from, gripper_to)):
            raise ValueError('Invalid aperture')
        delta = np.r_[end-start, (gripper_to-gripper_from)*(self.gmax-self.gmin)]
        steps = max(1, math.ceil(float(np.max(np.abs(delta)))/self.sweep_step))
        self.last_collision = None
        self.last_sample_count = 0
        motion_bounds = getattr(self, 'motion_bounds', None)
        travel = None if motion_bounds is None else motion_bounds.travel(delta)
        self.last_bound_method = 'global_reach' if travel is None else 'pairwise_rigid_tree'
        if steps > 1000:
            self.last_collision = {'reason': 'collision_sampling_budget'}
            return False
        # Test interval midpoints with a proven per-pair displacement allowance.
        # Common ancestor transforms cancel for self-distance. Unsupported
        # models retain the original global reach envelope. An envelope overlap
        # subdivides; it never relaxes clearance or the1000-sample budget.
        intervals = [(i/steps, (i+1)/steps) for i in reversed(range(steps))]
        checks = 0
        while intervals:
            if checks >= 1000:
                self.last_collision = {'reason': 'collision_sampling_budget'}
                return False
            lo, hi = intervals.pop()
            alpha = (lo+hi)/2
            self.set_configuration(start+alpha*(end-start), gripper_from+alpha*(gripper_to-gripper_from))
            if travel is None:
                margin = self.clearance + self.reach_bound*float(np.sum(np.abs(delta)))*(hi-lo)
                contact = self._clearance_contact(margin)
            else:
                half_width = (hi-lo)/2
                contact = self._clearance_contact_by_pair(self.clearance+half_width*travel[0],
                                                        self.clearance+half_width*travel[1])
            checks += 1
            self.last_sample_count = checks
            if contact is None:
                continue
            if contact['distance_m'] <= self.clearance:
                self.last_collision = contact
                return False
            intervals.extend([(alpha, hi), (lo, alpha)])
        return True

    def _clearance_contact(self, margin):
        """Closest relevant contact within the requested envelope, if any."""
        return self._clearance_contact_by_pair([margin]*len(self.links), [margin]*len(self.self_pairs))

    def _clearance_contact_by_pair(self, table_margins, self_margins):
        if (len(table_margins) != len(self.links) or len(self_margins) != len(self.self_pairs)
                or not all(math.isfinite(value) and value > 0 for value in (*table_margins, *self_margins))):
            raise ValueError('Invalid pairwise collision margins')
        p = self.p
        # getClosestPoints computes distances independently of the global
        # contact manifold. Rebuilding that manifold at every subdivision adds
        # work without changing this query (PyBullet Quickstart, getClosestPoints).
        names = self.link_names
        table_by_link = dict(zip(self.links, table_margins))
        contacts = []
        for point in p.getClosestPoints(self.body, self.table, max(table_margins), physicsClientId=self.client):
            if not math.isfinite(point[8]):
                raise ValueError('Invalid collision distance')
            if point[8] <= table_by_link[point[3]]:
                contacts.append((point[8], names[point[3]], 'table'))
        bounds = {}
        for link in self.links:
            box = p.getAABB(self.body, link, physicsClientId=self.client)
            try:
                validated_aabb(box)
            except (TypeError, ValueError) as exc:
                raise ValueError('Invalid collision AABB for link '+str(link)) from exc
            bounds[link] = box
        # Distance between enclosing AABBs is a lower bound on shape distance.
        # A tiny outward tolerance retains borderline pairs; it does not alter
        # the requested narrow-phase margin or the actual clearance threshold.
        for (a, b), margin in zip(self.self_pairs, self_margins):
            margin_squared = (margin+1e-9)**2
            alo, ahi = bounds[a]; blo, bhi = bounds[b]
            dx = max(alo[0]-bhi[0], blo[0]-ahi[0], 0.)
            dy = max(alo[1]-bhi[1], blo[1]-ahi[1], 0.)
            dz = max(alo[2]-bhi[2], blo[2]-ahi[2], 0.)
            if dx*dx+dy*dy+dz*dz > margin_squared:
                continue
            for point in p.getClosestPoints(self.body, self.body, margin, linkIndexA=a,
                                           linkIndexB=b, physicsClientId=self.client):
                if not math.isfinite(point[8]):
                    raise ValueError('Invalid collision distance')
                contacts.append((point[8], names[a], names[b]))
        if not contacts:
            return None
        distance, a, b = min(contacts)
        return {'reason': 'collision_clearance', 'link_a': a, 'link_b': b,
                'distance_m': distance, 'required_clearance_m': self.clearance}

    def close(self):
        self.p.disconnect(self.client)
