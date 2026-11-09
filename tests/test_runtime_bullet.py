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
    assert all(row['observation_error'] is None for row in rows)
    phases = {'input', 'robot_read', 'observation_read', 'session_ik', 'collision',
              'control_total', 'command', 'physics', 'status_output', 'ui', 'post_decision', 'before_log'}
    for index, row in enumerate(rows):
        assert set(row['timing_s']) == phases
        assert all(value >= 0 for value in row['timing_s'].values())
        assert row['timing_s']['control_total'] >= row['timing_s']['collision']
        if index:
            previous = row['previous_cycle_timing']
            assert previous['sequence'] == rows[index-1]['sequence']
            assert previous['work_s'] >= rows[index-1]['timing_s']['before_log']
            assert previous['log_write_s'] >= 0
        else:
            assert row['previous_cycle_timing'] is None


def test_stale_vision_rejection_is_logged_and_shown_without_commands(mesh_fixture, tmp_path):
    pytest.importorskip('pybullet')
    import json, os, subprocess, sys, time
    from anyteleop_so101.protocol import HandObservation
    root = Path(__file__).parents[1]
    config = json.loads((root/'config/runtime.sim.json').read_text())
    config['urdf'] = str(mesh_fixture)
    config['collision']['table_z'] = -1
    config_path = tmp_path/'config.json'; config_path.write_text(json.dumps(config))
    observation = HandObservation(0, time.monotonic()-1., 'test:0', 'camera_color_optical',
                                  None, None, 'unavailable', 'unavailable', 'unavailable')
    vision = tmp_path/'vision.json'
    vision.write_text(json.dumps({'schema': 'teleop.vision.v1', 'observation': observation.as_dict()}))
    trace = tmp_path/'trace.jsonl'
    result = subprocess.run([sys.executable, str(root/'tools/run_teleop.py'), '--config', str(config_path),
        '--headless', '--duration', '.2', '--vision-file', str(vision), '--log', str(trace)],
        input='engage\n', text=True, capture_output=True, timeout=15,
        env={**os.environ, 'PYTHONPATH': str(root/'src')})
    assert result.returncode == 0, result.stderr
    rows = [json.loads(line) for line in trace.read_text().splitlines()]
    assert rows
    assert all(row['observation'] is None and row['q_command'] is None for row in rows)
    assert all(row['observation_error'] == 'Stale/future vision input' for row in rows)
    assert 'camera input rejected: Stale/future vision input' in result.stdout
    assert 'tracking_unavailable (Stale/future vision input)' in result.stdout


@pytest.mark.parametrize('automatic', [False, True])
def test_automatic_preview_is_explicit_and_follows_without_engage_input(mesh_fixture, tmp_path, automatic):
    pytest.importorskip('pybullet')
    import json, os, subprocess, sys
    root = Path(__file__).parents[1]
    config = json.loads((root/'config/runtime.sim.json').read_text())
    config['urdf'] = str(mesh_fixture)
    config['collision']['table_z'] = -1
    config_path = tmp_path/'config.json'; config_path.write_text(json.dumps(config))
    trace = tmp_path/'trace.jsonl'
    command = [sys.executable, str(root/'tools/run_teleop.py'), '--config', str(config_path),
               '--synthetic', '--headless', '--control-mode', 'position', '--duration', '.5', '--log', str(trace)]
    if automatic:
        command.append('--automatic-preview')
    result = subprocess.run(command, input='', text=True, capture_output=True, timeout=15,
                            env={**os.environ, 'PYTHONPATH': str(root/'src')})
    assert result.returncode == 0, result.stderr
    rows = [json.loads(line) for line in trace.read_text().splitlines()]
    assert rows and rows[0]['q_command'] is None
    if automatic:
        assert any(row['reason'] == 'engaged_no_jump' for row in rows)
        assert any(row['reason'] == 'tracking' for row in rows)
        assert all(row['automatic_preview']['simulation_only'] for row in rows)
        assert 'Space pauses/resumes; Q quits' in result.stdout
        assert all(row['automatic_preview']['physical_recovery_validated'] is False for row in rows)
        assert 'E engage / R release' not in result.stdout
    else:
        assert all(row['q_command'] is None and row['automatic_preview'] is None for row in rows)
        assert 'Starts RELEASED' in result.stdout


def test_refinement_finds_collision_between_clear_endpoints():
    # Analytic one-axis scene: endpoints and the first midpoint are clear, but
    # the path crosses a 4 mm clearance at q=.25. Refinement must reject it.
    world = BulletWorld.__new__(BulletWorld)
    world.gmin, world.gmax = 0., 1.
    world.clearance, world.reach_bound, world.sweep_step = .005, 1., 2.
    position = [0.]
    def set_configuration(q, aperture):
        position[0] = q[0]
    def contact(margin):
        distance = abs(position[0]-.25)+.004
        return {'distance_m': distance} if distance <= margin else None
    world.set_configuration = set_configuration
    world._clearance_contact = contact
    assert not world.check_segment(np.zeros(5), np.array([1.,0,0,0,0]), .5, .5)
    assert world.last_collision['distance_m'] < .005


def test_refinement_budget_fails_closed():
    world = BulletWorld.__new__(BulletWorld)
    world.gmin, world.gmax = 0., 1.
    world.clearance, world.reach_bound, world.sweep_step = .005, 1., 2.
    world.set_configuration = lambda q, aperture: None
    world._clearance_contact = lambda margin: {'distance_m': np.nextafter(.005, 1.)}
    assert not world.check_segment(np.zeros(5), np.ones(5), .5, .5)
    assert world.last_collision['reason'] == 'collision_sampling_budget'


def test_official_model_small_gripper_motion_preserves_clearance():
    pytest.importorskip('pybullet')
    model = Path(__file__).parents[1]/'assets/so101/so101_new_calib.urdf'
    if not model.is_file():
        pytest.skip('Official model not downloaded')
    world = BulletWorld(model)
    try:
        q = np.array([.1,-.3,.4,.15,.2])
        # Previously rejected by the coarse 13 mm envelope despite >5 mm
        # actual clearance. No exclusions or clearance thresholds are changed.
        assert world.check_segment(q, q, .5, .505)
        assert world.last_collision is None
    finally:
        world.close()


def test_physical_gripper_range_is_used_for_render_feedback_command_and_guard(mesh_fixture, monkeypatch):
    pytest.importorskip('pybullet')
    world = BulletWorld(mesh_fixture, table_z=-1, gripper_angle_range_rad=[-.04, 1.57])
    q = np.array([.1, -.3, .4, .15, .2])
    angle = lambda aperture: -.04+aperture*1.61
    try:
        for aperture in (0, .25, 1):
            world.set_configuration(q, aperture)
            physical = world.p.getJointState(world.body, world.gripper, physicsClientId=world.client)[0]
            assert physical == pytest.approx(angle(aperture))
            assert world.read_state().aperture == pytest.approx(aperture)
        commands = []
        monkeypatch.setattr(world.p, 'setJointMotorControlArray', lambda *args, **kwargs: commands.append(kwargs))
        world.command(q, .75)
        assert commands[-1]['targetPositions'][-1] == pytest.approx(angle(.75))
        visited = []
        original = world.set_configuration
        def sample(q, aperture):
            original(q, aperture)
            actual = world.p.getJointState(world.body, world.gripper, physicsClientId=world.client)[0]
            assert actual == pytest.approx(angle(aperture))
            visited.append(aperture)
        monkeypatch.setattr(world, 'set_configuration', sample)
        monkeypatch.setattr(world, '_clearance_contact', lambda margin: None)
        assert world.check_segment(q, q, .2, .8)
        assert visited and all(.2 <= aperture <= .8 for aperture in visited)
    finally:
        world.close()


def test_physical_gripper_range_cannot_exceed_urdf(mesh_fixture):
    pytest.importorskip('pybullet')
    with pytest.raises(ValueError, match='outside URDF'):
        BulletWorld(mesh_fixture, gripper_angle_range_rad=[-.3, 1.57])


def exhaustive_clearance_contact(world, margin):
    """Pre-optimization reference: refresh all contacts and query every pair."""
    p = world.p
    p.performCollisionDetection(physicsClientId=world.client)
    names = {-1: 'base_link', **{v[0]: v[12].decode() for v in world.info.values()}}
    contacts = [(point[8], names[point[3]], 'table') for point in
                p.getClosestPoints(world.body, world.table, margin, physicsClientId=world.client)]
    for index, a in enumerate(world.links):
        for b in world.links[index+1:]:
            if world.parents.get(a) == b or world.parents.get(b) == a:
                continue
            contacts.extend((point[8], names[a], names[b]) for point in
                p.getClosestPoints(world.body, world.body, margin, linkIndexA=a,
                                   linkIndexB=b, physicsClientId=world.client))
    if not contacts:
        return None
    distance, a, b = min(contacts)
    return {'reason': 'collision_clearance', 'link_a': a, 'link_b': b,
            'distance_m': distance, 'required_clearance_m': world.clearance}


def test_optimized_official_clearance_matches_exhaustive_after_each_pose_reset():
    pytest.importorskip('pybullet')
    model = Path(__file__).parents[1]/'assets/so101/so101_new_calib.urdf'
    if not model.is_file():
        pytest.skip('Official model not downloaded')
    world = BulletWorld(model)
    rng = np.random.default_rng(4911)
    try:
        lower = [world.info[name][8] for name in ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')]
        upper = [world.info[name][9] for name in ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')]
        for _ in range(128):
            world.set_configuration(rng.uniform(lower, upper), float(rng.random()))
            margin = float(rng.uniform(.005, .15))
            # Optimized query must observe the NEW pose before any global
            # detection pass. The reference then refreshes and checks all pairs.
            optimized = world._clearance_contact(margin)
            assert optimized == exhaustive_clearance_contact(world, margin)
    finally:
        world.close()


def test_recorded_expensive_jaw_segment_keeps_clearance_with_fewer_proven_samples():
    pytest.importorskip('pybullet')
    model = Path(__file__).parents[1]/'assets/so101/so101_new_calib.urdf'
    if not model.is_file():
        pytest.skip('Official model not downloaded')
    # Live retest sequence1790: >.11rad jaw travel near a wrist/jaw pair,
    # originally180 conservative samples and~32ms, despite clear geometry.
    q0 = np.array([.14671862607362077, -.3205680742740232, .412942098204465,
                   .16127667557122716, .21194552323846133])
    q1 = np.array([.14004497117796605, -.3156116723476675, .4095295365014495,
                   .15940839042163427, .2102436800655274])
    world = BulletWorld(model)
    try:
        assert world.motion_bounds is not None, world.motion_bound_fallback_reason
        optimized_bounds = world.motion_bounds
        results = []
        for bound in (optimized_bounds, None):
            world.motion_bounds = bound
            world._clearance_contact = lambda margin: exhaustive_clearance_contact(world, margin)
            clear = world.check_segment(q0, q1, .48029333296573085, .4221635212955718)
            results.append((clear, world.last_collision, world.last_sample_count))
        assert results[0][:2] == results[1][:2] == (True, None)
        assert results[1][2] == 180
        assert results[0][2] < results[1][2]
    finally:
        world.close()


@pytest.mark.parametrize('bad_box', [
    ((float('inf'),)*3, (float('inf'),)*3),
    ((10., 0., 0.), (float('nan'), 1., 1.)),
    ((10., 0., 0.), (-10., 1., 1.)),
    ((0., 0.), (1., 1., 1.)),
    ((0., 0., 0.),),
    None,
])
def test_invalid_aabb_cannot_cull_a_penetrating_pair(bad_box):
    from types import SimpleNamespace
    world = BulletWorld.__new__(BulletWorld)
    world.body, world.table, world.client = 1, 2, 0
    world.links = [0, 2]
    world.link_names = {0: 'a', 2: 'b'}
    world.self_pairs = ((0, 2),)
    world.clearance = .005
    penetration = [None]*9
    penetration[8] = -.01
    world.p = SimpleNamespace(
        getAABB=lambda body, link, **kwargs: bad_box,
        getClosestPoints=lambda body_a, body_b, margin, **kwargs:
            [penetration] if body_a == body_b else [])
    with pytest.raises(ValueError, match='Invalid collision AABB'):
        world._clearance_contact(.005)


@pytest.mark.parametrize('invalid_distance', [float('nan'), float('inf'), -float('inf')])
def test_pair_specific_table_filter_rejects_nonfinite_contact_distance(invalid_distance):
    from types import SimpleNamespace
    world = BulletWorld.__new__(BulletWorld)
    world.body, world.table, world.client = 1, 2, 0
    world.links = [0]; world.self_pairs = (); world.link_names = {0: 'a'}
    point = [None]*9; point[3] = 0; point[8] = invalid_distance
    world.p = SimpleNamespace(getClosestPoints=lambda *args, **kwargs: [point])
    with pytest.raises(ValueError, match='Invalid collision distance'):
        world._clearance_contact_by_pair([.005], [])


def test_pair_bound_refinement_rejects_interior_collision_and_budget_exhaustion():
    from types import SimpleNamespace
    world = BulletWorld.__new__(BulletWorld)
    world.gmin, world.gmax = 0., 1.
    world.clearance, world.reach_bound, world.sweep_step = .005, 1., 2.
    world.motion_bounds = SimpleNamespace(travel=lambda delta: (np.array([1.]), np.array([0.])))
    position = [0.]
    world.set_configuration = lambda q, aperture: position.__setitem__(0, q[0])
    def contact(table, pairs):
        distance = abs(position[0]-.25)+.004
        return {'distance_m': distance} if distance <= table[0] else None
    world._clearance_contact_by_pair = contact
    assert not world.check_segment(np.zeros(5), np.array([1., 0, 0, 0, 0]), .5, .5)
    assert world.last_collision['distance_m'] < .005
    world._clearance_contact_by_pair = lambda table, pairs: {'distance_m': np.nextafter(.005, 1.)}
    assert not world.check_segment(np.zeros(5), np.ones(5), .5, .5)
    assert world.last_collision['reason'] == 'collision_sampling_budget'
    assert world.last_sample_count == 1000


def test_pair_bound_zero_motion_and_exact_clearance_remain_checked():
    from types import SimpleNamespace
    world = BulletWorld.__new__(BulletWorld)
    world.gmin, world.gmax = 0., 1.
    world.clearance, world.reach_bound, world.sweep_step = .005, 1., .01
    world.motion_bounds = SimpleNamespace(travel=lambda delta: (np.array([0.]), np.array([0.])))
    world.set_configuration = lambda q, aperture: None
    def contact(table, pairs):
        assert table[0] == pairs[0] == .005
        return {'distance_m': .005}
    world._clearance_contact_by_pair = contact
    assert not world.check_segment(np.zeros(5), np.zeros(5), .5, .5)
    assert world.last_sample_count == 1


def test_unsupported_bound_fallback_keeps_full_global_envelope():
    world = BulletWorld.__new__(BulletWorld)
    world.motion_bounds = None
    world.gmin, world.gmax = 0., 1.
    world.clearance, world.reach_bound, world.sweep_step = .005, 2., 2.
    world.set_configuration = lambda q, aperture: None
    margins = []
    world._clearance_contact = lambda margin: margins.append(margin)
    assert world.check_segment(np.zeros(5), np.array([-.2, 0, 0, 0, 0]), .5, .4)
    assert margins == pytest.approx([.005+2.*(.2+.1)])
    assert world.last_bound_method == 'global_reach'


def test_loaded_bounds_cover_rotated_scaled_multi_shape_geometry_and_nonzero_com(mesh_fixture):
    pytest.importorskip('pybullet')
    from scipy.spatial.transform import Rotation
    root = ET.parse(mesh_fixture).getroot()
    jaw = next(link for link in root.findall('link') if link.get('name') == 'moving_jaw_so101_v1_link')
    inertial = jaw.find('inertial')
    ET.SubElement(inertial, 'origin', xyz='.04 -.03 .02', rpy='.2 -.4 .6')
    original = jaw.find('collision')
    original.find('origin').set('xyz', '.025 -.015 .02')
    original.find('origin').set('rpy', '.7 -.3 .5')
    original.find('geometry/mesh').set('scale', '3 2 4')
    second = ET.SubElement(jaw, 'collision')
    ET.SubElement(second, 'origin', xyz='-.04 .03 -.02', rpy='-.3 .6 .4')
    ET.SubElement(ET.SubElement(second, 'geometry'), 'mesh', filename='tiny.obj', scale='2 4 3')
    ET.ElementTree(root).write(mesh_fixture)
    world = BulletWorld(mesh_fixture, table_z=-1)
    try:
        assert world.motion_bounds is not None, world.motion_bound_fallback_reason
        radius = world.motion_bounds.link_radii[world.gripper]
        vertices = np.array([[0., 0., 0.], [.002, 0., 0.], [0., .002, 0.], [0., 0., .002]])
        for collision in jaw.findall('collision'):
            origin = collision.find('origin')
            offset = np.fromstring(origin.get('xyz'), sep=' ')
            rotation = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
            scale = np.fromstring(collision.find('geometry/mesh').get('scale'), sep=' ')
            points = (vertices*scale)@rotation.T+offset
            assert np.max(np.linalg.norm(points, axis=1)) <= radius
        state = world.p.getLinkState(world.body, world.gripper, computeForwardKinematics=True, physicsClientId=world.client)
        assert np.linalg.norm(np.array(state[0])-state[4]) > .04
        assert radius < .15  # COM offset must not replace the URDF link origin.
    finally:
        world.close()


def test_pairwise_midpoint_queries_match_exhaustive_per_pair_margins():
    pytest.importorskip('pybullet')
    model = Path(__file__).parents[1]/'assets/so101/so101_new_calib.urdf'
    if not model.is_file():
        pytest.skip('Official model not downloaded')
    world = BulletWorld(model)
    rng = np.random.default_rng(791)
    try:
        assert world.motion_bounds is not None, world.motion_bound_fallback_reason
        for _ in range(40):
            world.set_configuration(rng.uniform(-1., 1., 5), float(rng.random()))
            table = rng.uniform(.005, .15, len(world.links))
            pairs = rng.uniform(.005, .15, len(world.self_pairs))
            actual = world._clearance_contact_by_pair(table, pairs)
            contacts = []
            for link, margin in zip(world.links, table):
                contacts.extend((point[8], world.link_names[link], 'table') for point in
                    world.p.getClosestPoints(world.body, world.table, margin, linkIndexA=link, physicsClientId=world.client)
                    if point[8] <= margin)
            for (a, b), margin in zip(world.self_pairs, pairs):
                contacts.extend((point[8], world.link_names[a], world.link_names[b]) for point in
                    world.p.getClosestPoints(world.body, world.body, margin, linkIndexA=a, linkIndexB=b, physicsClientId=world.client))
            if contacts:
                distance, a, b = min(contacts)
                assert actual == {'reason': 'collision_clearance', 'link_a': a, 'link_b': b,
                                  'distance_m': distance, 'required_clearance_m': world.clearance}
            else:
                assert actual is None
    finally:
        world.close()


def test_randomized_clear_segments_have_no_dense_sample_collision():
    pytest.importorskip('pybullet')
    model = Path(__file__).parents[1]/'assets/so101/so101_new_calib.urdf'
    if not model.is_file():
        pytest.skip('Official model not downloaded')
    world = BulletWorld(model)
    rng = np.random.default_rng(2917)
    accepted = 0
    try:
        assert world.motion_bounds is not None, world.motion_bound_fallback_reason
        for _ in range(60):
            start = np.array([.1, -.3, .4, .15, .2])+rng.uniform(-.2, .2, 5)
            end = start+rng.uniform(-.1, .1, 5)
            ga, gb = rng.uniform(.1, .9, 2)
            if not world.check_segment(start, end, ga, gb):
                continue
            accepted += 1
            for alpha in np.linspace(0., 1., 41):
                world.set_configuration(start+alpha*(end-start), ga+alpha*(gb-ga))
                assert world._clearance_contact(world.clearance) is None
        assert accepted >= 10
    finally:
        world.close()
