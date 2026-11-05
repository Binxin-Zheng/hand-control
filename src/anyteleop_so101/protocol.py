"""Explicit wire contract. No pickle and no dependency on legacy handtrack packets."""
from dataclasses import dataclass
import json
import numpy as np
from .geometry import Pose

POSITION_EVIDENCE={'rgbd_fused','stereo','synthetic','unavailable','predicted','monocular_model'}
ORIENTATION_EVIDENCE={'rgb_model','rgbd_fit','stereo','synthetic','unavailable','predicted'}

@dataclass(frozen=True)
class HandObservation:
    sequence: int
    timestamp_s: float
    track_id: str
    frame_id: str
    pose: Pose | None
    aperture: float | None
    position_evidence: str
    orientation_evidence: str
    aperture_evidence: str='rgb_model'

    def __post_init__(self):
        if not isinstance(self.sequence,int) or self.sequence<0 or not np.isfinite(self.timestamp_s) or self.timestamp_s<0:
            raise ValueError("Invalid sequence/timestamp")
        if not self.track_id or not self.frame_id: raise ValueError("Identity and frame required")
        if self.position_evidence not in POSITION_EVIDENCE or self.orientation_evidence not in ORIENTATION_EVIDENCE:
            raise ValueError("Unknown evidence; never silently promote prediction")
        if self.aperture_evidence not in {'rgb_model','synthetic','unavailable','predicted'}:
            raise ValueError("Invalid aperture evidence")
        if self.aperture is not None and (not np.isfinite(self.aperture) or not 0<=self.aperture<=1):
            raise ValueError("Aperture must be 0 closed .. 1 open")
        if self.pose is None and (self.position_evidence!='unavailable' or self.orientation_evidence!='unavailable'):
            raise ValueError("Missing pose requires unavailable evidence")
        if self.aperture is None and self.aperture_evidence!='unavailable':
            raise ValueError("Missing aperture requires unavailable evidence")

    def as_dict(self):
        return {'schema':'teleop.hand_observation.v1','sequence':self.sequence,'timestamp_s':self.timestamp_s,
                'clock':'host_monotonic','track_id':self.track_id,'frame_id':self.frame_id,
                'pose':None if self.pose is None else self.pose.as_dict(),'aperture':self.aperture,
                'position_evidence':self.position_evidence,'orientation_evidence':self.orientation_evidence,
                'aperture_evidence':self.aperture_evidence}

    @classmethod
    def from_dict(cls,d):
        expected={'schema','sequence','timestamp_s','clock','track_id','frame_id','pose','aperture','position_evidence','orientation_evidence','aperture_evidence'}
        if set(d)!=expected or d['schema']!='teleop.hand_observation.v1' or d['clock']!='host_monotonic':
            raise ValueError("Wrong schema, clock, or fields")
        p=d['pose']
        if p is not None and set(p)!={'position_m','quaternion_xyzw'}: raise ValueError("Wrong pose fields")
        pose=None if p is None else Pose.from_xyzw(p['position_m'],p['quaternion_xyzw'])
        return cls(d['sequence'],d['timestamp_s'],d['track_id'],d['frame_id'],pose,d['aperture'],
                   d['position_evidence'],d['orientation_evidence'],d['aperture_evidence'])

    def to_json(self):
        return json.dumps(self.as_dict(),allow_nan=False,separators=(',',':'))
