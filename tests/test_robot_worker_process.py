"""Run the real worker loop with a fake port; never imports or connects a motor SDK."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def test_worker_expires_in_its_own_process_and_stops_on_eof(tmp_path):
    root=Path(__file__).parents[1]
    runner=tmp_path/'fake_worker.py'; events=tmp_path/'events.jsonl'
    runner.write_text('''import importlib.util,json,time,sys
spec=importlib.util.spec_from_file_location("worker",sys.argv[1])
w=importlib.util.module_from_spec(spec);spec.loader.exec_module(w)
class FakePort:
    def __init__(self,config):
        self.bus=self;self.is_connected=True;self.safe_to_hold=True
    def event(self,name):
        with open(sys.argv[2],"a") as f:f.write(json.dumps([name,time.monotonic()])+"\\n")
    def open_and_arm(self,enable):self.event("open")
    def read(self):return dict(timestamp_s=time.monotonic(),q=[0.]*5,aperture=.5)
    def send(self,q,aperture):self.event("send")
    def stop(self):self.event("stop")
    def close(self):self.event("close")
w.RawFeetechPort=FakePort
w.validate_acceptance=lambda config:None
w.run({},False)
''')
    process=subprocess.Popen([sys.executable,str(runner),str(root/'tools/robot_worker.py'),str(events)],
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    def send(seq,mode):
        process.stdin.write(json.dumps(dict(sequence=seq,deadline_s=time.monotonic()+.08,
            mode=mode,q=[0.]*5,aperture=.5))+'\n');process.stdin.flush()
    try:
        assert json.loads(process.stdout.readline())['reason']=='release_required'
        send(0,'release')
        while json.loads(process.stdout.readline())['reason']!='released': pass
        send(1,'active')
        while json.loads(process.stdout.readline())['reason']!='active': pass
        # No heartbeat. Worker must stop without controller help.
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            row=json.loads(process.stdout.readline())
            if row['reason']=='command_expired': break
        assert row['reason']=='command_expired'
        send(2,'active')
        while json.loads(process.stdout.readline())['reason']=='command_expired': pass
        process.stdin.close()
        process.wait(timeout=3)
        assert process.returncode==0,process.stderr.read()
        recorded=[json.loads(line) for line in events.read_text().splitlines()]
        assert [name for name,_ in recorded].count('send')==1
        sent=next(t for name,t in recorded if name=='send')
        stopped=next(t for name,t in recorded if name=='stop' and t>=sent)
        assert 0<stopped-sent<.25
        assert recorded[-1][0]=='close'
    finally:
        if process.poll() is None: process.kill();process.wait()
        process.stdout.close();process.stderr.close()
