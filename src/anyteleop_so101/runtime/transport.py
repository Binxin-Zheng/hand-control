"""Same-host latest-only observations. UDP is diagnostic input, never motor access."""
import json
import socket
import time
from ..protocol import HandObservation


class ObservationReceiver:
    def __init__(self, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', port))
        self.socket.setblocking(False)
        self.latest = None
        self.metadata = {}
        self.error = None

    def poll(self):
        # Bound drain work so flooding cannot starve robot stop handling.
        for _ in range(64):
            try:
                raw = self.socket.recv(65536)
            except BlockingIOError:
                break
            try:
                packet = json.loads(raw)
                if packet['schema'] != 'teleop.vision.v1':
                    raise ValueError('Wrong vision envelope')
                obs = HandObservation.from_dict(packet['observation'])
                now = time.monotonic()
                if obs.timestamp_s > now + .005 or now - obs.timestamp_s > .15:
                    raise ValueError('Stale/future vision input')
                if self.latest is not None and obs.timestamp_s < self.latest.timestamp_s:
                    raise ValueError('Out of order input')
                self.latest, self.metadata, self.error = obs, packet, None
            except (ValueError, KeyError, TypeError) as exc:
                self.latest = None
                self.error = str(exc)
        return self.latest

    def close(self):
        self.socket.close()


class FileObservationReceiver:
    """Atomic latest-file IPC for separate local environments; no socket needed."""
    def __init__(self, path):
        from pathlib import Path
        self.path = Path(path)
        self.latest = None
        self.metadata = {}
        self.error = None

    def poll(self):
        try:
            if self.path.stat().st_size > 65536:
                raise ValueError('Vision packet too large')
            packet = json.loads(self.path.read_bytes())
            if packet['schema'] != 'teleop.vision.v1':
                raise ValueError('Wrong vision envelope')
            obs = HandObservation.from_dict(packet['observation'])
            now = time.monotonic()
            if obs.timestamp_s > now+.005 or now-obs.timestamp_s > .15:
                raise ValueError('Stale/future vision input')
            if self.latest is not None and obs.timestamp_s < self.latest.timestamp_s:
                raise ValueError('Out of order input')
            self.latest, self.metadata, self.error = obs, packet, None
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.latest = None
            self.error = str(exc)
        return self.latest

    def close(self):
        pass


def publish_file(path, packet):
    """One producer per destination. Atomic replacement never exposes partial JSON."""
    import os
    from pathlib import Path
    path = Path(path)
    data = json.dumps(packet, allow_nan=False).encode()
    if len(data) > 65536:
        raise ValueError('Vision packet too large')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+f'.{os.getpid()}.tmp')
    try:
        temporary.write_bytes(data)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
