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
