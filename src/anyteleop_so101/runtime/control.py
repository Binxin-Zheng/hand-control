"""Controller integration shared by simulated and physical ports."""
from ..session import TeleopSession


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
                return self.session._stop('PAUSED', 'collision_requires_reengage')
            decision.diagnostics['collision_checked'] = True
        return decision
