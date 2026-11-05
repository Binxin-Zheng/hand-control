"""Fetch exact audited Git blobs. No pip install or environment modification.
This optional network script was NOT executed in the authoring runtime.
URDF meshes are NOT fetched: use the upstream repository for rendering/collisions.
"""
import argparse,base64,hashlib,json,urllib.request
from pathlib import Path

FILES={
 'so101':('TheRobotStudio/SO-ARM100','9552a231d8b23bed68ec15779eba620c5d875ec4','so101_new_calib.urdf'),
 'detector':('dexsuite/dex-retargeting','4867ad7d2e5ea3984002c1730b937d06b1fe97ee','single_hand_detector.py'),
}

def fetch(key,output):
    repo,sha,name=FILES[key]
    req=urllib.request.Request(f'https://api.github.com/repos/{repo}/git/blobs/{sha}',headers={'User-Agent':'so101-handoff-audit'})
    with urllib.request.urlopen(req,timeout=30) as r:info=json.load(r)
    data=base64.b64decode(info['content'])
    calculated=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
    if calculated!=sha:raise RuntimeError('Upstream blob hash mismatch')
    out=Path(output);out.mkdir(parents=True,exist_ok=True);dest=out/name
    if dest.exists() and dest.read_bytes()!=data:raise FileExistsError(f'Refusing to overwrite {dest}')
    dest.write_bytes(data)
    print(f'Verified {dest}: git blob {sha}. Review upstream license before redistribution.')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('component',choices=FILES);p.add_argument('--out',default='vendor_audit')
    a=p.parse_args();fetch(a.component,a.out)
