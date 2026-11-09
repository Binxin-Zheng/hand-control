"""Pure conservative-bound construction; no Bullet world or devices opened."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from anyteleop_so101.runtime.motion_bounds import BoundUnavailable, RigidTreeMotionBounds, bounds_from_loaded_urdf


def tree():
    # Link2 is fixed below1. Link3 is an independent branch from the base.
    return RigidTreeMotionBounds([-1, 0, 1, 2, 3], {0: -1, 1: 0, 2: 1, 3: -1},
        {0: 9., 1: .3, 2: .2, 3: 8.}, {-1: 0., 0: .1, 1: .2, 2: .4, 3: .5},
        [0, 1, 3], [(0, 2), (2, 3), (-1, 2)])


def test_levers_include_every_descendant_origin_and_common_ancestors_cancel():
    bounds = tree()
    np.testing.assert_allclose(bounds.table_coefficients[3], [.9, .6, 0.])
    np.testing.assert_allclose(bounds.pair_coefficients[0], [0., .6, 0.])
    np.testing.assert_allclose(bounds.pair_coefficients[1], [.9, .6, .5])
    np.testing.assert_allclose(bounds.pair_coefficients[2], [.9, .6, 0.])
    table, pairs = bounds.travel([.2, 0., 0.])
    assert pairs[0] == 0.  # Joint0 applies a common rigid transform.
    assert table[3] == pytest.approx(.18)  # It still moves relative to table.


def test_zero_negative_and_nonfinite_joint_travel():
    bounds = tree()
    assert all(np.count_nonzero(part) == 0 for part in bounds.travel([0., 0., 0.]))
    for positive, negative in zip(bounds.travel([.2, .1, .3]), bounds.travel([-.2, -.1, -.3])):
        np.testing.assert_array_equal(positive, negative)
    for invalid in ([float('nan'), 0., 0.], [0., float('inf'), 0.], [0.]):
        with pytest.raises(ValueError):
            bounds.travel(invalid)


def test_invalid_geometry_radius_refuses_optimization_and_large_radius_is_not_capped():
    for radius in (-.1, float('nan'), float('inf')):
        with pytest.raises(BoundUnavailable):
            RigidTreeMotionBounds([-1, 0], {0: -1}, {0: .1}, {-1: 0., 0: radius}, [0], [(-1, 0)])
    bounds = RigidTreeMotionBounds([-1, 0], {0: -1}, {0: .1}, {-1: 0., 0: 2.}, [0], [(-1, 0)])
    assert bounds.travel([.1])[0][1] == pytest.approx(.2)


def test_two_branch_displacement_bound_covers_dense_arcs_and_negative_sweeps():
    # Independent two-link planar branches with collision points at each tip.
    # Their common base transform may move arbitrarily without changing distance.
    bounds = RigidTreeMotionBounds([-1, 0, 1], {0: -1, 1: -1}, {0: 0., 1: 0.},
                                   {-1: 0., 0: .2, 1: .35}, [0, 1], [(0, 1)])
    rng = np.random.default_rng(741)
    for _ in range(100):
        midpoint = rng.uniform(-math.pi, math.pi, 2)
        delta = rng.uniform(-5., 5., 2)
        travel = bounds.travel(delta)[1][0]
        def points(q):
            return np.array([.2*np.array([math.cos(q[0]), math.sin(q[0])]),
                             .35*np.array([math.cos(q[1]), math.sin(q[1])])+[.1, .2]])
        center = points(midpoint)
        dmid = np.linalg.norm(center[0]-center[1])
        for alpha in np.linspace(-.5, .5, 21):
            current = points(midpoint+alpha*delta)
            distance = np.linalg.norm(current[0]-current[1])
            assert dmid-distance <= abs(alpha)*travel+1e-12


def test_loaded_enclosure_uses_link_origin_not_com_and_urdf_not_bullet_parent_frame(tmp_path):
    path = tmp_path/'tree.urdf'
    path.write_text('''<robot name="bound_test"><link name="base_link"/>
<link name="a"><collision/><collision/></link><link name="tip"><collision/></link>
<joint name="turn" type="revolute"><parent link="base_link"/><child link="a"/><origin xyz="9 0 0"/></joint>
<joint name="fixed_tip" type="fixed"><parent link="a"/><child link="tip"/><origin xyz="0 .2 0" rpy=".4 .5 .6"/></joint></robot>''')
    def info(index, kind, child, parent):
        row = [None]*17
        row[0], row[2], row[12], row[13], row[14], row[16] = index, kind, child.encode(), [0., 0., 1.], [999., 0., 0.], parent
        return row
    rows = {'turn': info(0, 0, 'a', -1), 'fixed_tip': info(1, 4, 'tip', 0)}
    origin = [10., 20., 30.]
    p = SimpleNamespace(JOINT_FIXED=4, JOINT_REVOLUTE=0, GEOM_BOX=1, GEOM_SPHERE=2,
                        GEOM_CYLINDER=3, GEOM_CAPSULE=4, GEOM_MESH=5)
    p.getCollisionShapeData = lambda body, link, **kw: [(None, None, 5)]*({-1: 0, 0: 2, 1: 1}[link])
    p.getAABB = lambda body, link, **kw: ([9.9, 19.8, 29.7], [10.1, 20.2, 30.3])
    p.getLinkState = lambda body, link, **kw: ([1000., 0., 0.], None, None, None, origin, None)
    result = bounds_from_loaded_urdf(path, p, 0, 0, rows, [-1, 0, 1], [0], [(-1, 1)])
    radius = math.hypot(.1, .2, .3)+1e-6*math.hypot(*origin)
    assert result.table_coefficients[1, 0] == pytest.approx(radius)
    assert result.table_coefficients[2, 0] == pytest.approx(radius+.2)
    rows['turn'][2] = 1
    with pytest.raises(BoundUnavailable, match='type'):
        bounds_from_loaded_urdf(path, p, 0, 0, rows, [-1, 0, 1], [0], [(-1, 1)])
    rows['turn'][2] = 0
    p.getCollisionShapeData = lambda body, link, **kw: []
    with pytest.raises(BoundUnavailable, match='every loaded collision shape'):
        bounds_from_loaded_urdf(path, p, 0, 0, rows, [-1, 0, 1], [0], [(-1, 1)])
