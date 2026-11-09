"""Explicit visual-simulator policy; never used by physical teleoperation."""
import math


class AutomaticPreview:
    RECOVERABLE = frozenset({'tracking_unavailable', 'stale_or_future_observation',
                             'hand_identity_changed'})

    def __init__(self):
        self.paused = False
        self.waiting = True
        self.release_pending = True
        self.fault = None
        self.stopped = False

    def command(self, command):
        if command == 'toggle_pause':
            command = 'resume' if self.paused or self.fault else 'pause'
        if command in ('pause', 'release'):
            self.paused = True
            self.waiting = self.release_pending = True
        elif command in ('resume', 'engage') and not self.stopped:
            self.paused = False
            self.fault = None
            self.waiting = self.release_pending = True
        elif command == 'reset':
            # Reset is deliberate and performed by TeleopSession. Stay paused
            # afterwards; Space must request a new release-and-latch sequence.
            self.paused = True
            self.waiting = self.release_pending = True

    def clutch_for_tick(self, observation, *, now_s, max_age_s, camera_frame,
                        allow_synthetic=False, receiver_error=None, receiver_error_kind=None):
        # A producer may not have published its first file yet. Stale input is
        # ordinary tracking loss; malformed, oversized and reordered packets
        # are protocol faults, not permission to automatically restart.
        if receiver_error and receiver_error_kind not in ('missing', 'stale'):
            self.fault = self.fault or 'vision_input_rejected: '+receiver_error
            self.waiting = self.release_pending = True
        if observation is not None and observation.timestamp_s > now_s+.005:
            self.fault = self.fault or 'future_observation_clock'
            self.waiting = self.release_pending = True
        if self.paused or self.stopped:
            return False
        if self.fault:
            return True  # Keep the core's fault latched until deliberate resume.
        if self.release_pending:
            self.release_pending = False
            return False  # A real release tick always precedes any new latch.
        if self.waiting:
            evidence = {'rgbd_fused', 'stereo'}
            if allow_synthetic:
                evidence.add('synthetic')
            usable = (observation is not None and observation.pose is not None
                      and observation.position_evidence in evidence
                      and observation.frame_id == camera_frame
                      and math.isfinite(observation.timestamp_s)
                      and -.005 <= now_s-observation.timestamp_s <= max_age_s)
            if not usable:
                return False
        # While following, let the unchanged supervisor diagnose loss/identity.
        return True

    def observe(self, decision):
        if decision.state == 'ESTOP':
            self.stopped = self.paused = self.waiting = self.release_pending = True
            self.fault = 'emergency_stop'
        elif decision.reason == 'estop_reset_requires_new_engage':
            self.stopped = False
            self.fault = None
            self.paused = self.waiting = self.release_pending = True
        elif decision.state == 'ACTIVE':
            self.waiting = False
        elif decision.state == 'PAUSED':
            if self.fault is not None:
                return  # Preserve the first latched cause until deliberate resume.
            if decision.reason in self.RECOVERABLE and self.fault is None:
                self.waiting = self.release_pending = True
            elif decision.reason != 'release_then_engage_required':
                self.fault = decision.reason
                self.waiting = self.release_pending = True

    def as_dict(self):
        state = ('STOPPED' if self.stopped else 'FAULT' if self.fault else
                 'PAUSED' if self.paused else 'WAITING_FOR_HAND' if self.waiting else 'FOLLOWING')
        return dict(state=state, fault=self.fault, release_pending=self.release_pending,
                    simulation_only=True, physical_recovery_validated=False)
