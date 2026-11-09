import json
import socket
import time
import pytest
from anyteleop_so101.runtime.transport import ObservationReceiver
from conftest import obs


def test_receiver_uses_latest_and_rejects_stale_packet():
    try: receiver=ObservationReceiver(0)
    except PermissionError: pytest.skip('Loopback sockets unavailable in sandbox')
    sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    try:
        addr=receiver.socket.getsockname(); now=time.monotonic()
        for sequence in range(2):
            packet={'schema':'teleop.vision.v1','observation':obs(now,sequence).as_dict()}
            sender.sendto(json.dumps(packet).encode(),addr)
        assert receiver.poll().sequence==1
        packet['observation']=obs(now-1,3).as_dict()
        sender.sendto(json.dumps(packet).encode(),addr)
        assert receiver.poll() is None
        assert 'Stale' in receiver.error
    finally:
        sender.close(); receiver.close()


def test_file_ipc_is_latest_only_and_expires(tmp_path):
    from anyteleop_so101.runtime.transport import FileObservationReceiver,publish_file
    path=tmp_path/'latest.json'; receiver=FileObservationReceiver(path)
    assert receiver.poll() is None
    now=time.monotonic()
    for sequence in range(3):
        packet={'schema':'teleop.vision.v1','observation':obs(now,sequence).as_dict()}
        publish_file(path,packet)
    assert receiver.poll().sequence==2
    assert not list(tmp_path.glob('*.tmp'))
    packet['observation']=obs(now-1,3).as_dict(); publish_file(path,packet)
    assert receiver.poll() is None
    path.write_text('{invalid')
    assert receiver.poll() is None


def test_file_receiver_distinguishes_missing_stale_future_and_protocol_errors(tmp_path):
    from anyteleop_so101.runtime.transport import FileObservationReceiver, publish_file
    path = tmp_path/'latest.json'; receiver = FileObservationReceiver(path)
    assert receiver.poll() is None and receiver.error_kind == 'missing'
    now = time.monotonic()
    for timestamp, kind in ((now-1, 'stale'), (now+1, 'future')):
        publish_file(path, {'schema': 'teleop.vision.v1', 'observation': obs(timestamp).as_dict()})
        assert receiver.poll() is None
        assert receiver.error == 'Stale/future vision input' and receiver.error_kind == kind
    publish_file(path, {'schema': 'teleop.vision.v1', 'observation': obs(now, 2).as_dict()})
    assert receiver.poll() is not None and receiver.error_kind is None
    publish_file(path, {'schema': 'teleop.vision.v1', 'observation': obs(now-.01, 3).as_dict()})
    assert receiver.poll() is None and receiver.error_kind == 'out_of_order'
    for raw, kind in (('x'*65537, 'oversized'), ('{"schema":"wrong"}', 'schema'), ('{invalid', 'malformed')):
        path.write_text(raw)
        assert receiver.poll() is None and receiver.error_kind == kind


def test_udp_receiver_error_kinds_without_opening_socket():
    from collections import deque
    pending = deque()
    class FakeSocket:
        def recv(self, size):
            if pending:
                return pending.popleft()
            raise BlockingIOError
    receiver = ObservationReceiver.__new__(ObservationReceiver)
    receiver.socket = FakeSocket()
    receiver.latest = receiver.error = receiver.error_kind = None
    receiver.metadata = {}
    now = time.monotonic()
    for timestamp, kind in ((now-1, 'stale'), (now+1, 'future'), (now, None), (now-.01, 'out_of_order')):
        pending.append(json.dumps({'schema': 'teleop.vision.v1', 'observation': obs(timestamp).as_dict()}).encode())
        result = receiver.poll()
        assert receiver.error_kind == kind
        assert (result is not None) == (kind is None)
    pending.append(b'{invalid')
    assert receiver.poll() is None and receiver.error_kind == 'malformed'
