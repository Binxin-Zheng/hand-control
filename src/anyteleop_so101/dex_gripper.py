"""Simulation-only palm/thumb binding to the actual upstream vector optimizer."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
from .geometry import finite_array


def palm_thumb_vector(points):
    """Unit thumb ray in a palm frame: outward, normal, wrist-to-MCP.

    Uses no finger tips to establish the fixed side. All 21 detector landmarks
    remain intact. This is model-derived geometry, not a depth measurement.
    """
    p = finite_array(points, (21, 3), 'hand points')
    forward = p[[5, 9, 13, 17]].mean(axis=0) - p[0]
    if np.linalg.norm(forward) < 1e-6:
        raise ValueError('Degenerate palm length')
    forward /= np.linalg.norm(forward)
    outward = p[5] - p[17]
    outward -= outward.dot(forward) * forward
    thumb = p[4] - p[2]
    if min(np.linalg.norm(outward), np.linalg.norm(thumb)) < 1e-6:
        raise ValueError('Degenerate palm/thumb')
    outward /= np.linalg.norm(outward)
    normal = np.cross(forward, outward)
    return np.array([thumb @ outward, thumb @ normal, thumb @ forward]) / np.linalg.norm(thumb)


class DexGripper:
    """Upstream VectorOptimizer + SeqRetargeting; only gripper is optimized."""
    def __init__(self, urdf, alpha=.2):
        from dex_retargeting.optimizer import VectorOptimizer
        from dex_retargeting.robot_wrapper import RobotWrapper
        from dex_retargeting.seq_retarget import SeqRetargeting
        from dex_retargeting.optimizer_utils import LPFilter
        tree = ET.parse(urdf)
        root = tree.getroot()
        # Auxiliary FK markers only; keep every official joint/link unchanged.
        # Tip marker is centered at mesh distal y=-82mm, with visual z=18.9mm.
        for name, parent, xyz in [
            ('retarget_palm', 'moving_jaw_so101_v1_link', '0 0 0.0189'),
            ('retarget_thumb', 'moving_jaw_so101_v1_link', '0 -0.082 0.0189')]:
            ET.SubElement(root, 'link', name=name)
            joint = ET.SubElement(root, 'joint', name=name+'_joint', type='fixed')
            ET.SubElement(joint, 'parent', link=parent)
            ET.SubElement(joint, 'child', link=name)
            ET.SubElement(joint, 'origin', xyz=xyz, rpy='0 0 0')
        # The pivot marker lies on the axis and is rigidly bound to the fixed
        # gripper body. Transform its 18.9mm axial offset through joint.origin.
        from scipy.spatial.transform import Rotation
        origin = root.find("joint[@name='gripper']/origin")
        R = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
        pivot = np.fromstring(origin.get('xyz'), sep=' ') + R @ [0, 0, .0189]
        j = root.find("joint[@name='retarget_palm_joint']")
        j.find('parent').set('link', 'gripper_link')
        j.find('origin').set('xyz', ' '.join(map(str, pivot)))
        with tempfile.TemporaryDirectory(prefix='so101-dex-') as tmp:
            path = Path(tmp)/'markers.urdf'
            tree.write(path)
            self.robot = RobotWrapper(str(path))
        optimizer = VectorOptimizer(self.robot, ['gripper'], ['retarget_palm'],
                                    ['retarget_thumb'], np.array([[2], [4]]), scaling=1.)
        self.sequence = SeqRetargeting(optimizer, lp_filter=LPFilter(alpha))
        self.index = self.robot.get_joint_index('gripper')
        self.lower, self.upper = self.robot.joint_limits[self.index]
        q = np.zeros(self.robot.dof)
        q[self.index] = self.lower
        self.robot.compute_forward_kinematics(q)
        a = self.robot.get_link_pose(self.robot.get_link_index('retarget_palm'))[:3, 3].copy()
        b = self.robot.get_link_pose(self.robot.get_link_index('retarget_thumb'))[:3, 3].copy()
        self.closed_vector = b-a
        grip_R = self.robot.get_link_pose(self.robot.get_link_index('gripper_link'))[:3, :3]
        axis = grip_R @ R @ np.fromstring(root.find("joint[@name='gripper']/axis").get('xyz'), sep=' ')
        self.tangent = np.cross(axis, self.closed_vector)
        self.normal = axis * np.linalg.norm(self.closed_vector)
        self.fixed = q[optimizer.idx_pin2fixed]
        self.sequence.set_qpos(q)

    def target_vector(self, points):
        outward, normal, forward = palm_thumb_vector(points)
        return (outward*self.tangent + normal*self.normal + forward*self.closed_vector)[None, :]

    def retarget(self, points):
        q = self.sequence.retarget(self.target_vector(points), self.fixed)
        angle = float(np.clip(q[self.index], self.lower, self.upper))
        return dict(angle_rad=angle, opening=float((angle-self.lower)/(self.upper-self.lower)),
                    joint_limits_rad=[float(self.lower), float(self.upper)])
