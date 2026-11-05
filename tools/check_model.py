"""Offline asset and FK verification; no hardware or camera access."""
import argparse
from pathlib import Path
from anyteleop_so101.runtime.model import validate_model
from anyteleop_so101.runtime.bullet import BulletWorld
from anyteleop_so101.kinematics import URDFChain
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--urdf',type=Path,default=Path('assets/so101/so101_new_calib.urdf'))
    args=p.parse_args()
    path=validate_model(args.urdf)
    chain=URDFChain(path,'base_link','gripper_link')
    world=BulletWorld(path)
    try:
        q=np.array([.1,-.3,.4,.15,.2]); world.set_configuration(q,.5)
        tip=next(v[0] for v in world.info.values() if v[12].decode()=='gripper_link')
        state=world.p.getLinkState(world.body,tip,computeForwardKinematics=True,physicsClientId=world.client)
        np.testing.assert_allclose(state[4],chain.fk(q).p,atol=1e-6)
        from scipy.spatial.transform import Rotation
        np.testing.assert_allclose(Rotation.from_quat(state[5]).as_matrix(),chain.fk(q).R,atol=1e-6)
        print('Complete mesh references, Bullet import, joint mapping, and FK agree.')
        print('Initial pose collision-clear:',world.check_segment(q,q,.5,.5))
        print('This does not validate calibrated TCP, physical geometry or motor dynamics.')
    finally: world.close()


if __name__=='__main__': main()
