"""Controller integration shared by simulated and physical ports."""
import numpy as np
from ..session import TeleopSession
from .driver_policy import ARM_NAMES, Calibration, gripper_angle_range


def configure_hardware_geometry(chain, hardware, collision_options):
    """Apply accepted physical limits before building the servo; never clamp feedback.

    Geometry preparation is pure/offline. It does not accept commissioning flags
    or authorize hardware. The same configured aperture drives real encoder goals
    and the guard's jaw angle, instead of assuming full URDF gripper travel.
    """
    Calibration(hardware['joints'])
    if tuple(chain.names) != ARM_NAMES:
        raise ValueError('Physical joint mapping does not match the kinematic chain')
    lower = np.maximum(chain.lower, [hardware['joints'][n]['lower'] for n in ARM_NAMES])
    upper = np.minimum(chain.upper, [hardware['joints'][n]['upper'] for n in ARM_NAMES])
    if np.any(lower >= upper):
        raise ValueError('Physical and URDF joint limits do not overlap')
    angles = gripper_angle_range(hardware)
    collision = dict(collision_options)
    override = collision.get('gripper_angle_range_rad')
    if override is not None and not np.array_equal(np.asarray(override), angles):
        raise ValueError('Collision gripper range must match physical calibration')
    collision['gripper_angle_range_rad'] = angles
    chain.lower, chain.upper = lower, upper
    return collision


class GuardedController:
    def __init__(self, session: TeleopSession, guard):
        self.session, self.guard = session, guard

    def tick(self, observation, measured, **events):
        decision = self.session.tick(observation, measured, **events)
        if decision.q_command is not None:
            try:
                clear = self.guard.check_segment(measured.q, decision.q_command,
                                                 measured.aperture, decision.aperture_command)
            except Exception:
                self.session._stop('PAUSED', 'collision_check_failed')
                raise
            if not clear:
                stopped = self.session._stop('PAUSED', 'collision_requires_reengage')
                stopped.diagnostics['collision'] = getattr(self.guard, 'last_collision', None)
                return stopped
            decision.diagnostics['collision_checked'] = True
        return decision
