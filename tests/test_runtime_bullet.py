"""Adapter tests with deliberately artificial tiny meshes, NOT SO101 validation."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from anyteleop_so101.runtime.bullet import BulletWorld
from anyteleop_so101.kinematics import URDFChain


@pytest.fixture
def mesh_fixture(tmp_path):
    # Official joint transforms with artificial nonphysical tetrahedra. This tests
    # loading, origins, joint-name mapping and engine calls, not robot geometry.
    root=ET.parse(Path(__file__).parents[1]/'assets/so101_kinematics_only.urdf').getroot()
    root.set('name','adapter_test_fixture_NOT_SO101_geometry')
    (tmp_path/'tiny.obj').write_text('v 0 0 0\nv .002 0 0\nv 0 .002 0\nv 0 0 .002\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n')
    for link in root.findall('link'):
        for tag in ('visual','collision'):
            element=ET.SubElement(link,tag)
            ET.SubElement(element,'origin',xyz='0 0 0',rpy='0 0 0')
            ET.SubElement(ET.SubElement(element,'geometry'),'mesh',filename='tiny.obj')
        inertial=ET.SubElement(link,'inertial')
        ET.SubElement(inertial,'mass',value='.02')
        ET.SubElement(inertial,'inertia',ixx='.0001',iyy='.0001',izz='.0001',ixy='0',ixz='0',iyz='0')
    path=tmp_path/'fixture.urdf'; ET.ElementTree(root).write(path)
    return path


def test_engine_feedback_fk_and_actual_dynamics(mesh_fixture):
    pytest.importorskip('pybullet')
    world=BulletWorld(mesh_fixture,table_z=-1)
    try:
        q=np.array([.1,-.3,.4,.15,.2]); world.set_configuration(q,.5)
        measured=world.read_state(); np.testing.assert_allclose(measured.q,q)
        chain=URDFChain(mesh_fixture,'base_link','gripper_link')
        tip=next(v[0] for v in world.info.values() if v[12].decode()=='gripper_link')
        pose=world.p.getLinkState(world.body,tip,computeForwardKinematics=True,physicsClientId=world.client)
        np.testing.assert_allclose(pose[4],chain.fk(q).p,atol=1e-7)
        world.command(q+np.array([.02,0,0,0,0]),.5)
        for _ in range(20): world.step()
        assert world.read_state().q[0]>q[0]+.001
        world.stop()
    finally: world.close()


def test_table_collision_is_rejected(mesh_fixture):
    pytest.importorskip('pybullet')
    world=BulletWorld(mesh_fixture,table_z=0)
    try:
        q=np.array([.1,-.3,.4,.15,.2])
        assert not world.check_segment(q,q,.5,.5)
    finally: world.close()


def test_headless_controller_runs_through_simulated_feedback(mesh_fixture,tmp_path):
    pytest.importorskip('pybullet')
    import json,os,subprocess,sys
    root=Path(__file__).parents[1]
    config=json.loads((root/'config/runtime.sim.json').read_text())
    config['urdf']=str(mesh_fixture)
    config['collision']['table_z']=-1
    config_path=tmp_path/'config.json'; config_path.write_text(json.dumps(config))
    trace=tmp_path/'trace.jsonl'
    result=subprocess.run([sys.executable,str(root/'tools/run_teleop.py'),'--config',str(config_path),
        '--synthetic','--headless','--duration','.5','--vision-port','0','--log',str(trace)],
        input='engage\n',text=True,capture_output=True,timeout=15,
        env={**os.environ,'PYTHONPATH':str(root/'src')})
    assert result.returncode==0,result.stderr
    rows=[json.loads(line) for line in trace.read_text().splitlines()]
    assert len(rows)>=10
    assert any(row['state']=='ACTIVE' for row in rows),result.stdout
    assert any(row['q_command'] is not None and row['diagnostics'].get('collision_checked') for row in rows)
