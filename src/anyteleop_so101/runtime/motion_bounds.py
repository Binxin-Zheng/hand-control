"""Conservative swept-distance bounds for a fixed-base rigid URDF tree.

For link A, an enclosing ball of radius r_A is centered on its URDF link
origin. For ancestor revolute joint j, every collision point is at most
R(j,A) = r_A + sum(||origin.xyz||) from that joint's origin; the sum includes
EVERY descendant edge from child(j) to A, including fixed edges. Rotations
preserve lengths, so this bound holds for all joint angles.

For a self pair A,B, their shared ancestor transform is a common rigid
isometry and cancels from separation distance. Only joints below their LCA
contribute. Along linear q(s), an interval of width h about its midpoint has
point displacement at most h/2 * sum(R(j,A)*abs(delta_q_j)) per branch.
Triangle/Hausdorff distance bounds separation decrease by the sum for both
branches. A static table uses every ancestor of A once. No closest-point
normal, sampled orientation, or assumed 1m lever is used in this proof.

The loaded Bullet AABB encloses every collision shape on a link. The farthest
corner from the TRUE URDF link origin therefore bounds the rigid local shape;
an outward pad covers float32 link-frame serialization. Unknown geometry or
kinematic conventions refuse this optimization, retaining the old envelope.
Enclosures are cached for the fixed-base, rigid loaded model; callers must not
replace/scale collision shapes afterwards. This proves the checked straight
configuration segment, not unmodeled physical actuator overshoot.
"""
from itertools import product
import math
import xml.etree.ElementTree as ET

import numpy as np


class BoundUnavailable(ValueError):
    """The optimized enclosure could not be established for this model."""


def validated_aabb(box):
    try:
        if len(box) != 2 or any(len(endpoint) != 3 for endpoint in box):
            raise ValueError('Wrong AABB shape')
        lower, upper = box
        if (not all(math.isfinite(value) for value in (*lower, *upper))
                or any(lo > hi for lo, hi in zip(lower, upper))):
            raise ValueError('Nonfinite or inverted AABB')
    except (TypeError, ValueError) as exc:
        raise ValueError('Invalid collision AABB') from exc
    return lower, upper


class RigidTreeMotionBounds:
    def __init__(self, links, parents, origin_lengths, radii, active_ids, pairs):
        self.links = tuple(links)
        self.pairs = tuple(pairs)
        active = {joint: column for column, joint in enumerate(active_ids)}
        if (len(active) != len(active_ids) or -1 not in self.links
                or set(parents) != set(self.links)-{-1}
                or set(origin_lengths) != set(parents) or set(radii) != set(self.links)
                or not set(active) <= set(parents)):
            raise BoundUnavailable('Incomplete rigid-tree mapping')
        if not all(math.isfinite(x) and x >= 0 for x in (*origin_lengths.values(), *radii.values())):
            raise BoundUnavailable('Invalid geometry radius or origin length')
        self.link_radii = dict(radii)
        coefficients = {}
        paths = {}
        for link in self.links:
            vector = np.zeros(len(active_ids))
            path = set()
            distance = radii[link]
            current = link
            while current != -1:
                if current not in parents or current in path:
                    raise BoundUnavailable('Invalid or cyclic parent tree')
                path.add(current)
                if current in active:
                    vector[active[current]] = distance
                # The incoming origin of j is excluded from j's own lever,
                # but included for every earlier ancestor (fixed edges too).
                distance += origin_lengths[current]
                current = parents[current]
            coefficients[link], paths[link] = vector, path
        self.table_coefficients = np.array([coefficients[link] for link in self.links])
        pair_coefficients = []
        for a, b in self.pairs:
            vector = coefficients[a]+coefficients[b]
            for common in paths[a] & paths[b]:
                if common in active:
                    vector[active[common]] = 0.
            pair_coefficients.append(vector)
        self.pair_coefficients = np.asarray(pair_coefficients).reshape(-1, len(active_ids))
        if not np.isfinite(self.table_coefficients).all() or not np.isfinite(self.pair_coefficients).all():
            raise BoundUnavailable('Nonfinite motion coefficients')

    def travel(self, delta):
        delta = np.asarray(delta, dtype=float)
        if delta.shape != (self.table_coefficients.shape[1],) or not np.isfinite(delta).all():
            raise ValueError('Invalid joint displacement for collision bound')
        absolute = np.abs(delta)
        return self.table_coefficients @ absolute, self.pair_coefficients @ absolute


def bounds_from_loaded_urdf(path, p, body, client, info, links, active_ids, pairs):
    """Validate parsed URDF paths against loaded joints, then enclose all shapes."""
    root = ET.parse(path).getroot()
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    xml_links = {link.get('name'): link for link in root.findall('link')}
    names = {-1: 'base_link', **{item[0]: item[12].decode() for item in info.values()}}
    if set(joints) != set(info) or set(xml_links) != set(names.values()) or len(names) != len(xml_links):
        raise BoundUnavailable('Loaded model and URDF names differ')
    link_ids = {name: index for index, name in names.items()}
    parents, lengths = {}, {}
    moving = set()
    for name, item in info.items():
        joint = joints[name]
        kind = joint.get('type')
        if kind not in ('fixed', 'revolute') or joint.find('mimic') is not None:
            raise BoundUnavailable('Only independent fixed/revolute joints have proven bounds')
        if item[2] != (p.JOINT_FIXED if kind == 'fixed' else p.JOINT_REVOLUTE):
            raise BoundUnavailable('Loaded joint type differs from URDF')
        child = joint.find('child').get('link'); parent = joint.find('parent').get('link')
        if child != names[item[0]] or parent not in link_ids or item[16] != link_ids[parent]:
            raise BoundUnavailable('Loaded joint parent differs from URDF')
        origin = joint.find('origin')
        try:
            xyz = tuple(float(x) for x in ('0 0 0' if origin is None else origin.get('xyz', '0 0 0')).split())
            rpy = tuple(float(x) for x in ('0 0 0' if origin is None else origin.get('rpy', '0 0 0')).split())
            if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(x) for x in (*xyz, *rpy)):
                raise ValueError('Invalid joint origin')
        except ValueError as exc:
            raise BoundUnavailable('Invalid URDF joint translation') from exc
        parents[item[0]], lengths[item[0]] = item[16], math.hypot(*xyz)
        if kind == 'revolute':
            axis = item[13]
            if len(axis) != 3 or not all(math.isfinite(x) for x in axis) or not math.isclose(math.hypot(*axis), 1., abs_tol=1e-6):
                raise BoundUnavailable('Loaded revolute axis is not unit length')
            moving.add(item[0])
    if moving != set(active_ids):
        raise BoundUnavailable('A moving joint is not represented in the sweep')
    bounded_types = {p.GEOM_BOX, p.GEOM_SPHERE, p.GEOM_CYLINDER, p.GEOM_CAPSULE, p.GEOM_MESH}
    radii = {}
    for link in links:
        shapes = p.getCollisionShapeData(body, link, physicsClientId=client)
        if len(shapes) != len(xml_links[names[link]].findall('collision')):
            raise BoundUnavailable('Cannot account for every loaded collision shape')
        if any(shape[2] not in bounded_types for shape in shapes):
            raise BoundUnavailable('Unsupported collision geometry')
        if link == -1 or not shapes:
            # The base is fixed: its radius cannot contribute to any motion.
            # Geometryless links remain in the tree and ordinary query set.
            radii[link] = 0.
            continue
        try:
            lower, upper = validated_aabb(p.getAABB(body, link, physicsClientId=client))
            origin = p.getLinkState(body, link, computeForwardKinematics=True, physicsClientId=client)[4]
            if len(origin) != 3 or not all(math.isfinite(x) for x in origin):
                raise ValueError('Invalid URDF link frame')
            radius = max(math.dist(corner, origin) for corner in product(*zip(lower, upper)))
            radii[link] = radius+1e-6*max(1., math.hypot(*origin))
        except (TypeError, ValueError, IndexError) as exc:
            raise BoundUnavailable('Cannot establish loaded-shape enclosure') from exc
    return RigidTreeMotionBounds(links, parents, lengths, radii, active_ids, pairs)
