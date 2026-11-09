# SO101 palm/thumb upstream optimizer verification

Uses unmodified dex-retargeting VectorOptimizer, SeqRetargeting and LPFilter
from checkout 3f56141bc8bd2760d5e452e382937269554ebb21.
The isolated upstream environment already contained the required dependencies.
No packages, physical robot state, or upstream source files were changed.

- Core suite: 117 passed, 1 skipped.
- Existing offline control replay: 1,200 recorded decisions reproduced.
- Actual upstream optimizer: seven FK-generated joint targets recovered within
  5.15e-7 radians after settling; seven synthetic palm/thumb targets across the
  joint range recovered within 2.94e-6 radians after settling.
- All 981 recorded frames retained: 942 detected, 39 missing/unselected.
- Palm reference excludes finger tips; perturbing all four finger chains beyond
  their MCPs does not alter the target. All detected poses tested under a rigid
  rotation, translation and scale: maximum target difference 9.42e-16 m.
- Full color-video -> original detector -> isolated optimizer -> PyBullet
  preview ran to EOF: 981 frames, 942 optimizer outputs, 278 fresh renderer
  updates. Negative closed limit -0.174533 radians reached.
- Optimizer replay median/p95 latency approximately 0.35/0.97 ms on this machine;
  these are optimizer-only timings, not end-to-end camera latency.

Commands:

```bash
PYTHONPATH=src .venv-core/bin/python -m pytest -q
PYTHONPATH=src .venv-core/bin/python examples/replay.py outputs/readiness/offline/trace.jsonl
PYTHONPATH=src /home/bx/repos/dex-retargeting/.venv/bin/python tools/validate_dex_gripper.py
PYTHONPATH=src .venv-core/bin/python tools/gripper_test.py --headless --video sample/hand-guesture/close.webm --out outputs/readiness/dex-palm-thumb/video
```

The two uploaded videos are identical. Low output in sampled tucked-thumb
frames is consistent with their appearance, but no independent open-gesture
labels exist. Synthetic rigid-rotation invariance does not establish live
viewpoint robustness: model-estimated hand shape can change with viewpoint.
The one-DOF robot cannot reproduce arbitrary out-of-plane thumb motion.
This change is limited to the automatic gripper preview, not full-arm control.
