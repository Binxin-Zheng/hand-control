"""SDK-independent calibration and expiring-command policy (stdlib only)."""
import math

ARM_NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class Calibration:
    def __init__(self, joints):
        if set(joints) != set(ARM_NAMES) | {'gripper'}:
            raise ValueError('All six named motors require calibration')
        self.joints = joints
        ids = []
        for name, item in joints.items():
            if set(item) != {'id', 'zero_raw', 'raw_per_unit', 'lower', 'upper', 'raw_min', 'raw_max'}:
                raise ValueError('Incomplete calibration: '+name)
            if not all(finite(v) for v in item.values()):
                raise ValueError('Calibration must be finite numeric data')
            if type(item['id']) is not int or not 1 <= item['id'] <= 253:
                raise ValueError('Invalid motor ID')
            ids.append(item['id'])
            if item['raw_per_unit'] == 0 or not item['lower'] < item['upper']:
                raise ValueError('Invalid scale or limits')
            if not 0 <= item['raw_min'] < item['raw_max'] <= 4095:
                raise ValueError('Expected calibrated single-turn STS3215 raw range')
            if name == 'gripper' and (item['lower'], item['upper']) != (0, 1):
                raise ValueError('Gripper units must be 0 closed .. 1 open')
            self.to_raw(name, item['lower']); self.to_raw(name, item['upper'])
        if len(set(ids)) != 6:
            raise ValueError('Duplicate motor IDs')

    def to_raw(self, name, value):
        c = self.joints[name]
        if not finite(value) or not c['lower'] <= value <= c['upper']:
            raise ValueError('Outside calibrated joint limit: '+name)
        raw = round(c['zero_raw'] + value*c['raw_per_unit'])
        if not c['raw_min'] <= raw <= c['raw_max']:
            raise ValueError('Outside raw servo range: '+name)
        return raw

    def from_raw(self, name, raw):
        c = self.joints[name]
        if not finite(raw) or not c['raw_min'] <= raw <= c['raw_max']:
            raise ValueError('Invalid servo feedback: '+name)
        value = (raw-c['zero_raw'])/c['raw_per_unit']
        # Quantized endpoint readings can differ by half a tick.
        tolerance = .51/abs(c['raw_per_unit'])
        if not c['lower']-tolerance <= value <= c['upper']+tolerance:
            raise ValueError('Feedback outside calibrated limit: '+name)
        return min(c['upper'], max(c['lower'], value))


class CommandLease:
    def __init__(self, max_lease=.1, max_joint_step=.025, max_grip_step=.04):
        if not all(finite(x) and x > 0 for x in (max_lease, max_joint_step, max_grip_step)):
            raise ValueError('Invalid driver policy')
        self.max_lease, self.max_step, self.max_grip = max_lease, max_joint_step, max_grip_step
        self.sequence = -1
        self.deadline = None
        self.released = False
        self.estop = False
        self.reason = 'release_required'

    def stop(self, reason):
        self.deadline = None
        self.released = False
        self.reason = reason

    def poll(self, now):
        if self.deadline is not None and now >= self.deadline:
            self.stop('command_expired')
            return True
        return False

    def accept(self, packet, now, measured):
        self.poll(now)  # New arrivals must not hide an already expired lease.
        if set(packet) != {'sequence', 'deadline_s', 'mode', 'q', 'aperture'}:
            raise ValueError('Wrong command fields')
        seq, deadline = packet['sequence'], packet['deadline_s']
        if type(seq) is not int or seq <= self.sequence:
            raise ValueError('Replayed command')
        if not finite(deadline) or not now < deadline <= now+self.max_lease:
            raise ValueError('Expired or excessively long command lease')
        self.sequence = seq
        mode = packet['mode']
        if mode == 'estop':
            self.estop = True; self.stop('emergency_stop'); return None
        if mode == 'reset':
            self.estop = False; self.stop('reset_requires_release'); return None
        if self.estop:
            raise ValueError('Emergency stop is latched')
        if mode == 'release':
            self.stop('released'); self.released = True; return None
        if mode == 'stop':
            self.stop('controller_stop'); return None
        if mode != 'active' or not self.released:
            raise ValueError('Explicit release required before active commands')
        q, grip = packet['q'], packet['aperture']
        if not isinstance(q, list) or len(q) != 5 or not all(finite(v) for v in q):
            raise ValueError('Invalid joint command')
        if not finite(grip) or not 0 <= grip <= 1:
            raise ValueError('Invalid gripper command')
        if any(abs(a-b) > self.max_step for a, b in zip(q, measured['q'])):
            raise ValueError('Target too far from measured joints')
        if abs(grip-measured['aperture']) > self.max_grip:
            raise ValueError('Target too far from measured gripper')
        self.deadline = deadline
        self.reason = 'active'
        return q, grip
