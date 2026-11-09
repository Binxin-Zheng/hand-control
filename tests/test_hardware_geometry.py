"""Physical/model geometry consistency, entirely offline with synthetic encoders."""
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from anyteleop_so101.geometry import Pose
from anyteleop_so101.kinematics import URDFChain
from anyteleop_so101.mapping import RelativeMapper
from anyteleop_so101.protocol import HandObservation
from anyteleop_so101.runtime.control import configure_hardware_geometry
from anyteleop_so101.runtime.driver_policy import Calibration, gripper_angle_range
from anyteleop_so101.servo import PositionPriorityServo
from anyteleop_so101.session import RobotState, TeleopSession


@pytest.fixture
def hardware_geometry():
    chain = URDFChain(Path(__file__).parents[1]/'assets/so101_kinematics_only.urdf', 'base_link', 'gripper_link')
    joints = {name: dict(id=i+1, zero_raw=2000, raw_per_unit=500, lower=-3.5,
                        upper=3.5, raw_min=0, raw_max=4095)
              for i, name in enumerate(chain.names)}
    joints['gripper'] = dict(id=6, zero_raw=1974, raw_per_unit=1046.5,
                            lower=0, upper=1, raw_min=1000, raw_max=3500)
    config = dict(joints=joints, gripper_model_calibration=dict(
        zero_raw=2000, raw_per_rad=650, angle_range_rad=[-.04, 1.57]))
    return chain, config


def test_physical_bounds_intersect_urdf_before_servo_construction(hardware_geometry):
    chain, config = hardware_geometry
    model_lower, model_upper = chain.lower.copy(), chain.upper.copy()
    config['joints']['shoulder_pan'].update(lower=-.4, upper=.6)
    config['joints']['elbow_flex']['upper'] = 1.2
    source = deepcopy(config)
    collision = configure_hardware_geometry(chain, config, {'table_z': -.02})
    expected_lower, expected_upper = model_lower.copy(), model_upper.copy()
    expected_lower[0], expected_upper[0], expected_upper[2] = -.4, .6, 1.2
    np.testing.assert_array_equal(chain.lower, expected_lower)
    np.testing.assert_array_equal(chain.upper, expected_upper)
    assert collision == {'table_z': -.02, 'gripper_angle_range_rad': (-.04, 1.57)}
    assert config == source  # No acceptance changes or raw-bound rewriting.


def test_ik_stays_inside_physical_range_and_rejects_infeasible_deceleration(hardware_geometry):
    chain, config = hardware_geometry
    config['joints']['shoulder_pan'].update(lower=-.02, upper=.02)
    configure_hardware_geometry(chain, config, {})
    servo = PositionPriorityServo(chain)
    q, velocity = np.zeros(5), np.zeros(5)
    start = chain.fk(q)
    target = Pose(start.p+np.array([0, .04, 0]), start.R)
    rejected = False
    for _ in range(150):
        try:
            step = servo.step(q, target, .02, velocity)
        except ValueError as exc:
            # The existing servo fails closed when its bounded acceleration
            # cannot meet the narrowed position bound; it must not step past it.
            assert str(exc) == 'Infeasible joint/acceleration bounds'
            rejected = True
            break
        q, velocity = step.q_next, step.velocity
        assert np.all(q >= chain.lower+servo.margin-1e-10)
        assert np.all(q <= chain.upper-servo.margin+1e-10)
    assert abs(q[0]) <= .0100000001
    assert rejected


def test_outside_physical_pose_rejects_engagement_without_clamping(hardware_geometry):
    chain, config = hardware_geometry
    config['joints']['shoulder_pan'].update(lower=-.02, upper=.02)
    configure_hardware_geometry(chain, config, {})
    session = TeleopSession(PositionPriorityServo(chain), RelativeMapper(np.eye(3)))
    q = np.array([.025, 0, 0, 0, 0])
    observation = HandObservation(0, 1., 'hand', 'camera_color_optical',
        Pose([0, 0, .5], np.eye(3)), None, 'rgbd_fused', 'rgb_model', 'unavailable')
    decision = session.tick(observation, RobotState(1., q, .5), now_s=1., dt=.02, clutch=True)
    assert decision.reason == 'outside_soft_limits_at_engage'
    assert decision.q_command is None
    assert q[0] == .025


def test_nonoverlapping_physical_range_is_rejected(hardware_geometry):
    chain, config = hardware_geometry
    config['joints']['shoulder_pan'].update(lower=2., upper=2.2)
    before = chain.lower.copy(), chain.upper.copy()
    with pytest.raises(ValueError, match='do not overlap'):
        configure_hardware_geometry(chain, config, {})
    np.testing.assert_array_equal(chain.lower, before[0])
    np.testing.assert_array_equal(chain.upper, before[1])


def test_gripper_raw_goals_and_collision_angles_agree(hardware_geometry):
    _, config = hardware_geometry
    calibration = Calibration(config['joints'])
    lo, hi = gripper_angle_range(config)
    model = config['gripper_model_calibration']
    for aperture in (0, .1, .5, .9, 1):
        raw = calibration.to_raw('gripper', aperture)
        physical_angle = (raw-model['zero_raw'])/model['raw_per_rad']
        assert physical_angle == pytest.approx(lo+aperture*(hi-lo), abs=.5/model['raw_per_rad']+1e-12)


def test_gripper_mismatch_cannot_be_hidden_by_collision_override(hardware_geometry):
    chain, config = hardware_geometry
    with pytest.raises(ValueError, match='must match physical calibration'):
        configure_hardware_geometry(chain, config, {'gripper_angle_range_rad': [-.174533, 1.74533]})
    config['joints']['gripper']['zero_raw'] += 1
    with pytest.raises(ValueError, match='mappings disagree'):
        gripper_angle_range(config)
