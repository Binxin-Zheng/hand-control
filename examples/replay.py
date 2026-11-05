"""Re-run recorded numerical inputs and compare every emitted command/reason."""
import argparse,json
import numpy as np
from offline_demo import build
from anyteleop_so101.session import RobotState
from anyteleop_so101.protocol import HandObservation

def replay(path):
    _,s=build();count=0
    with open(path) as f:
        for line in f:
            r=json.loads(line);o=r['observation'];o=None if o is None else HandObservation.from_dict(o)
            b=r['robot'];state=RobotState(b['timestamp_s'],np.array(b['q_rad']),b['aperture'])
            d=s.tick(o,state,now_s=r['now_s'],dt=r['dt'],**r['events']);expected=r['decision']
            if (d.state,d.reason)!=(expected['state'],expected['reason']):raise AssertionError(f'state mismatch on row {count}')
            if expected['q_command_rad'] is None:
                assert d.q_command is None
            else:
                assert np.allclose(d.q_command,expected['q_command_rad'],atol=1e-9,rtol=0)
                assert abs(d.aperture_command-expected['aperture_command'])<1e-9
            count+=1
    print(f'{count} recorded decisions reproduced; no motor or camera used.')
    return count

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('trace');a=p.parse_args();replay(a.trace)
