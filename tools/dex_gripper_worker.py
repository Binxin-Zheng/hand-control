"""Explicit simulation-only JSON-lines worker in the existing dex environment."""
import argparse
import json
import sys
from anyteleop_so101.dex_gripper import DexGripper

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', required=True)
    args = parser.parse_args()
    retargeter = DexGripper(args.urdf)
    print(json.dumps({'ready': True, 'optimizer': 'dex_retargeting.VectorOptimizer'}), flush=True)
    for line in sys.stdin:
        try:
            result = retargeter.retarget(json.loads(line))
        except ValueError as exc:
            result = {'error': str(exc)}
        print(json.dumps(result, allow_nan=False), flush=True)
