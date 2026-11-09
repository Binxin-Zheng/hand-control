# Live01 unchanged upstream detector: raw-result reprobe

This offline replay distinguishes raw MediaPipe failure from the original
wrapper's handedness filtering. It does not change the camera worker, estimator,
upstream source, confidence thresholds, preprocessing, or physical control.

## Complete input and reproducibility

- Recording: `outputs/hand-follow/20261106-093702-636085eb/camera`.
- All **2,857 attempts** retained: seven initial capture-clock errors, **2,849
  readable acquired RGB frames**, one corrupt final archive at sequence 2856.
- Exactly **2,849 original `hand_detector.process` calls**, once per readable
  RGB frame in chronological order, starting with a fresh original graph at the
  beginning of the complete recording. Capture failures had no original
  inference. No internal sequence gaps or silently skipped frames.
- **2,849/2,849 complete selected probes exactly reproduce the saved probes**:
  selected boolean, all 21 image coordinates, all 21 local model coordinates,
  wrist rotation, and operator transform. Maximum numeric difference is zero.
  No replay nondeterminism was observed in this run. The original numeric `n`
  was not saved, so historical comparison covers `bool(n)`; replayed numeric
  counts and all raw outputs are now recorded.
- Recorded selected denominator remains **2,385**; **2,384** selected frames can
  be reprocessed because the final selected frame's archive is corrupt. Its
  saved observation is retained, with no reconstructed raw result claimed.
- Upstream Git blob remains `4867ad7d2e5ea3984002c1730b937d06b1fe97ee` and source
  bytes are unchanged before/after. Input JSONL SHA256 is
  `1423ac3c7cc0e90f6a0d20ffe7cfc4395377a1b906e724d6dd2395a8dc825b85`.
- Individual archive/RGB hashes, package versions, script hash, all raw 21-point
  image/world landmarks, handedness scores, and every comparison are retained
  in the associated JSONL/input manifest. Raw data are replay evidence from this
  instrumented run; equality of all saved outputs supports correspondence but
  does not retroactively create an online raw-result log.

The unchanged constructor uses `hand_type='Right', selfie=False`, hence expects
the raw MediaPipe label **Left**. It keeps the upstream `max_num_hands=1`, temporal
tracking mode, and both original 0.8 detection/tracking confidence thresholds.
Stored BGR arrays receive the same BGR-to-RGB conversion. A transparent proxy
captures the very same `.process` result object returned to original `detect`;
there is no second inference or alternative selection.

## What the 465 wrapper nonselections mean

| Raw result | Frames | Share of 465 nonselections |
| --- | ---: | ---: |
| MediaPipe returned no hand | 269 | 57.85% |
| MediaPipe returned a hand, filtered by original handedness selection | 196 | 42.15% |

All 196 filtered raw labels are `Right` (expected `Left`). Their scores range
from 0.50723 to 0.99716; the median is approximately 0.9947. **This is not 196
proven handedness classification errors.** A raw hand can be the other actual
hand, which the selection rule correctly rejects.

The largest filtered block is sequences **9–164**, 156 frames spanning **8.003
seconds** between first/last capture timestamps. Raw landmarks in inspected
frames 86 and 155 surround the other hand beneath the chin on the right of the
unmirrored image; frame 164 still tracks that hand lowered at image right
(raw wrist approximately `(525,318)` pixels), while the desired raised hand is
fully visible at image left. Frame 165 has no raw hand; frame 166 selects the
raised hand at approximately `(270,273)`. This is evidence of **single-hand
tracking occupying the wrong hand**, so ignoring the label would risk identity
mixing rather than solve the problem.

The second filtered block is sequences **2435–2468**, 34 frames spanning **1.601
seconds**. The hand is fully framed in the comfortable palm-down/horizontal
pose. Raw landmarks remain around the same target hand: wrist approximately
`(278,226)` at 2435, `(273,220)` at 2452, `(274,226)` at 2468. The raw label flips
to the expected label at 2469 with wrist `(276,228)`. This supports a separate
**target handedness instability** mechanism. Frame 2469 is selected but its
depth surface is unsupported, so fixing RGB selection alone would not establish
global position for that frame. Other filtered frames are 194, 322, 1545, 1935,
2373, and 2374.

Longest raw no-hand blocks:

| Sequences | Frames | First-to-last capture span |
| --- | ---: | ---: |
| 1550–1744 | 195 | 9.804 s |
| 280–321 | 42 | 2.101 s |
| 323–342 | 20 | 0.967 s |
| 344–348 | 5 | 0.233 s |

Representative saved images show the hand close to or crossing the image edge
in these long blocks. This describes recorded conditions, **not a proposal to
narrow the user's pose range**. A fully framed open hand at 165 also has one
raw no-hand frame, and phone handling at 2555 has one. The existing contact
sheet `live-01-detector.jpg` shows these examples.

Within the prior longest global-position outage (1537–1955), raw results are
218 selected frames, 199 no-hand frames, and two filtered frames. Thus that
outage contains both RGB tracking failure and independent depth-support failure.

## Consequences and limits

1. Keep the original detector as the reproducible baseline. Simply bypassing
   handedness is contradicted by the distractor-hand evidence. A separately
   evaluated detector/association variant would need to expose multiple hands
   and maintain explicit identity; this report does not implement that change.
2. The palm-down block warrants testing identity association across a raw label
   flip while retaining explicit identity-change handling. Continuity of image
   landmarks alone does not prove identity through overlap or disappearance.
3. Neither selection fix creates missing metric depth. The 478 independently
   retained surface failures and one final archive error remain distinct from
   the 465 RGB nonselections. No prediction is counted as observed position.

This replay establishes diagnosis and software reproducibility for the recorded
sequence, not camera accuracy, current live reliability, or physical acceptance.

## Validation and artifacts

- `tools/reprobe_recorded_detector.py` is offline-only and imports inference
  dependencies only when explicitly invoked. It rejects recordings missing the
  initial sequence, preserves unreadable attempts, and stops temporal inference
  after any internal unreadable frame instead of silently resuming.
- **14 pure/fake tests passed**, including exact original class-body return
  equivalence with/without the process tap, all 21 points, wrong-hand vs no-hand,
  exception transparency, final/interior corruption, missing prefix rejection,
  and guarded malformed/nonfinite diagnostic capture. No devices or model
  inference are used by these tests.
- Full offline command completed successfully in **46.53 s**. Original detect
  call timing: median **12.003 ms**, P95 **14.025 ms**, max **41.627 ms**. These are
  offline timings and do not establish live end-to-end latency.
- `live-01-detector-reprobe.jsonl`: all attempts and complete raw observations.
- `live-01-detector-reprobe.summary.json`: denominators and exact comparisons.
- `live-01-detector-reprobe.inputs.json`: source/package/image fingerprints.
- `live-01-detector-reprobe.log`: stdout/stderr, including terminal archive error.

Commands used:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv-core/bin/python -m pytest -q tests/test_detector_reprobe.py
.venv-vision/bin/python tools/reprobe_recorded_detector.py outputs/hand-follow/20261106-093702-636085eb/camera --out outputs/position-research/live-01-detector-reprobe.jsonl > outputs/position-research/live-01-detector-reprobe.log 2>&1
```
