"""Validate complete follower geometry before any simulator/driver startup."""
from pathlib import Path
import xml.etree.ElementTree as ET

ARM_NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')


def validate_model(path):
    path = Path(path).resolve()
    root = ET.parse(path).getroot()
    joints = {j.get('name'): j for j in root.findall('joint') if j.get('type') != 'fixed'}
    if set(joints) != set(ARM_NAMES) | {'gripper'}:
        raise ValueError('Expected five SO101 arm joints and separate gripper')
    if 'leader' in root.get('name', '').lower():
        raise ValueError('Follower model required')
    meshes = root.findall('.//mesh')
    if not meshes or not root.findall('.//collision'):
        raise ValueError('Full visual/collision model required; kinematic extract is not allowed')
    for mesh in meshes:
        name = mesh.get('filename', '')
        if '://' in name or not name or not (path.parent / name).is_file():
            raise ValueError(f'Missing or unresolved mesh: {name}')
    return path
