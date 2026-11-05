"""Minimal URDF serial-chain FK and geometric Jacobian for numerical verification.
No mesh loading, collision checking, dynamics, calibration, or motor communication.
"""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import Pose, finite_array, unit

@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    origin: Pose
    axis: np.ndarray
    lower: float=-np.inf
    upper: float=np.inf


class URDFChain:
    def __init__(self,urdf_path,base_link,tip_link,tip_T_tcp=None):
        root=ET.parse(Path(urdf_path)).getroot()
        links={x.attrib['name'] for x in root.findall('link')}
        if base_link not in links or tip_link not in links:
            raise ValueError("Unknown base or tip link")
        child_to_joint={}
        for j in root.findall('joint'):
            child=j.find('child').attrib['link']
            if child in child_to_joint:
                raise ValueError("URDF has multiple parents")
            child_to_joint[child]=j
        nodes=[]; tip=tip_link; seen=set()
        while tip!=base_link:
            if tip in seen or tip not in child_to_joint:
                raise ValueError("No acyclic chain from base to tip")
            seen.add(tip); j=child_to_joint[tip]; nodes.append(j)
            tip=j.find('parent').attrib['link']
        self.joints=[]
        for j in reversed(nodes):
            kind=j.attrib['type']
            if kind not in ('fixed','revolute','prismatic') or j.find('mimic') is not None:
                raise ValueError("Unsupported joint; do not silently approximate")
            origin=j.find('origin')
            xyz=np.fromstring(origin.get('xyz','0 0 0') if origin is not None else '0 0 0',sep=' ')
            rpy=np.fromstring(origin.get('rpy','0 0 0') if origin is not None else '0 0 0',sep=' ')
            a=j.find('axis'); axis=unit(np.fromstring(a.get('xyz','1 0 0') if a is not None else '1 0 0',sep=' '))
            low,high=-np.inf,np.inf
            if kind!='fixed':
                lim=j.find('limit')
                if lim is None: raise ValueError("Explicit joint limits required")
                low,high=float(lim.attrib['lower']),float(lim.attrib['upper'])
                if not np.isfinite([low,high]).all() or low>=high: raise ValueError("Invalid joint limits")
            self.joints.append(Joint(j.attrib['name'],kind,Pose(xyz,Rotation.from_euler('xyz',rpy).as_matrix()),axis,low,high))
        self.active=[j for j in self.joints if j.kind!='fixed']
        self.names=tuple(j.name for j in self.active); self.n=len(self.names)
        self.lower=np.array([j.lower for j in self.active]); self.upper=np.array([j.upper for j in self.active])
        self.tip_T_tcp=tip_T_tcp or Pose.identity()

    def fk_jacobian(self,q):
        q=finite_array(q,(self.n,),"joint radians")
        T=Pose.identity(); axes=[]; origins=[]; kinds=[]; k=0
        for joint in self.joints:
            T=T.compose(joint.origin)
            if joint.kind=='fixed': continue
            axes.append(T.R@joint.axis); origins.append(T.p); kinds.append(joint.kind)
            if joint.kind=='revolute':
                motion=Pose(np.zeros(3),Rotation.from_rotvec(joint.axis*q[k]).as_matrix())
            else:
                motion=Pose(joint.axis*q[k],np.eye(3))
            T=T.compose(motion); k+=1
        T=T.compose(self.tip_T_tcp); J=np.zeros((6,self.n))
        for i,(a,o,kind) in enumerate(zip(axes,origins,kinds)):
            if kind=='revolute': J[:3,i]=np.cross(a,T.p-o); J[3:,i]=a
            else: J[:3,i]=a
        return T,J

    def fk(self,q):
        return self.fk_jacobian(q)[0]
