"""Single-jaw thumb mapping and the previous proximity baseline."""
import numpy as np
from .geometry import finite_array


def thumb_jaw_opening(points):
    """One-DOF jaw model: fixed fingers and a thumb rotating outward.

    Signed rotation is measured in the finger/knuckle-width plane, excluding
    palm-normal flexion. Parallel/inward is closed, perpendicular-outward is
    full opening. No thresholds fitted to user endpoints or surface distance.
    """
    p=finite_array(points,(21,3),'hand points')
    rays=np.array([p[b]-p[a] for a,b in ((5,8),(9,12),(13,16),(17,20))])
    lengths=np.linalg.norm(rays,axis=1)
    if np.any(lengths<1e-6):raise ValueError('Degenerate fixed-jaw fingers')
    fixed=np.mean(rays/lengths[:,None],axis=0)
    if np.linalg.norm(fixed)<1e-6:raise ValueError('Degenerate fixed-jaw direction')
    fixed/=np.linalg.norm(fixed)
    outward=p[5]-p[17];outward-=np.dot(outward,fixed)*fixed
    if np.linalg.norm(outward)<1e-6:raise ValueError('Degenerate opening plane')
    outward/=np.linalg.norm(outward)
    thumb=p[4]-p[2]
    lateral=float(thumb@outward);forward=float(thumb@fixed)
    if np.hypot(lateral,forward)<1e-6:raise ValueError('Thumb direction unavailable in opening plane')
    angle=float(np.arctan2(lateral,forward))
    return float(np.clip(angle/(np.pi/2),0,1)),angle


def thumb_palm_opening(points, closed=.15, opened=.85):
    """Distance to a finite triangulated palm, normalized by MCP palm width.

    Defaults are provisional, NOT fitted from the identical supplied clips.
    This is model geometry/proximity, not a sensor measurement of skin contact.
    """
    p=finite_array(points,(21,3),'hand points')
    width=np.linalg.norm(p[5]-p[17])
    if width<1e-6 or not 0<=closed<opened:raise ValueError('Invalid palm geometry or endpoints')
    tip=p[4];distances=[]
    for ids in ((0,5,9),(0,9,13),(0,13,17)):
        a,b,c=p[list(ids)];basis=np.column_stack((b-a,c-a))
        if np.linalg.matrix_rank(basis)<2:continue
        uv=np.linalg.lstsq(basis,tip-a,rcond=None)[0]
        if uv[0]>=0 and uv[1]>=0 and uv.sum()<=1:
            distances.append(np.linalg.norm(tip-a-basis@uv))
        for x,y in ((a,b),(b,c),(c,a)):
            edge=y-x;length=edge@edge
            if length>1e-12:
                t=np.clip((tip-x)@edge/length,0,1)
                distances.append(np.linalg.norm(tip-(x+t*edge)))
    if not distances:raise ValueError('Degenerate palm surface')
    distance=float(min(distances)/width)
    return float(np.clip((distance-closed)/(opened-closed),0,1)),distance
