"""Offline actual-optimizer checks and complete recorded-landmark replay."""
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
from anyteleop_so101.dex_gripper import DexGripper, palm_thumb_vector

r=DexGripper('assets/so101/so101_new_calib.urdf')
# Solver must recover targets produced by official FK, across the joint range.
errors=[]
for angle in np.linspace(r.lower,r.upper,7):
    q=np.zeros(r.robot.dof);q[r.index]=angle
    r.robot.compute_forward_kinematics(q)
    a=r.robot.get_link_pose(r.robot.get_link_index('retarget_palm'))[:3,3].copy()
    b=r.robot.get_link_pose(r.robot.get_link_index('retarget_thumb'))[:3,3].copy()
    for _ in range(60):actual=r.sequence.retarget((b-a)[None,:],r.fixed)[r.index]
    errors.append(abs(float(actual-angle)))
assert max(errors)<.005,errors
# End-to-end palm binding: parallel thumb closes; outward thumb opens.
hand_errors=[]
for theta in np.linspace(0,r.upper-r.lower,7):
    points=np.zeros((21,3))
    points[[5,9,13,17]]=[[.03,.08,0],[.01,.08,0],[-.01,.08,0],[-.03,.08,0]]
    points[2]=[.03,.02,0]
    points[4]=points[2]+.05*np.array([np.sin(theta),np.cos(theta),0])
    for _ in range(60):result=r.retarget(points)
    hand_errors.append(abs(result['angle_rad']-(r.lower+theta)))
assert max(hand_errors)<.005,hand_errors
rows=[json.loads(s) for s in open('outputs/gesture-analysis/shared.jsonl')]
r=DexGripper('assets/so101/so101_new_calib.urdf')
rotation=Rotation.from_euler('xyz',[1.1,-.7,2.]).as_matrix()
outputs=[];durations=[];max_rotation_error=0.
for row in rows:
    result=dict(sequence=row['sequence'],timestamp_s=row['timestamp_s'],detected=row['detected'])
    if row['detected']:
        points=np.array(row['local_points_mano'])
        error=np.max(np.abs(r.target_vector(points)-r.target_vector(1.7*points@rotation.T+[1,2,3])))
        max_rotation_error=max(max_rotation_error,error)
        changed=points.copy();changed[[6,7,8,10,11,12,14,15,16,18,19,20]]+=.03
        np.testing.assert_allclose(palm_thumb_vector(points),palm_thumb_vector(changed),atol=1e-12)
        start=time.perf_counter();result.update(r.retarget(points));durations.append(time.perf_counter()-start)
        assert r.lower<=result['angle_rad']<=r.upper
    outputs.append(result)
assert max_rotation_error<1e-12
out=Path('outputs/readiness/dex-palm-thumb');out.mkdir(parents=True,exist_ok=True)
(out/'replay.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in outputs))
a=np.array([v['opening'] for v in outputs if v['detected']])
summary=dict(total_frames=len(rows),detected_frames=len(a),unselected_or_missing=len(rows)-len(a),
             max_fk_recovery_error_rad=max(errors),max_hand_binding_error_rad=max(hand_errors),max_rotated_target_error_m=max_rotation_error,
             opening_quantiles=np.quantile(a,[0,.05,.5,.95,1]).tolist(),
             optimizer_latency_ms_p50_p95=(np.quantile(durations,[.5,.95])*1000).tolist())
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
