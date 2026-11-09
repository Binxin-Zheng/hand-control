# Instrumented legacy Holistic: complete live01 A/B

**Body-associated Holistic returns hand landmarks throughout the 34-frame
palm-down label-flip block, but only four of those frames pass unchanged depth
fusion.** Across the complete recording it provides 1,920 depth-supported
surface positions versus 1,906 baseline positions, with 28 gains and 14 losses.
The small net gain, crop losses and large reference disagreements require
separate assessment. Nothing is promoted to runtime or physical control.

## Configuration and complete input

The installed MediaPipe 0.10.14 legacy graph was instrumented to expose both
hands' 21 image/world landmarks, current hand-presence scores/flags, raw
handedness, final hand tracking ROIs, body wrist visibility and all 33 body
image/world landmarks. The graph has 202 expanded nodes and 19 requested output
streams. **Both hand-presence gates and both pose detection/presence gates are
0.8**; the separate original pose-wrist ROI visibility gate remains 0.1.
Model complexity is 1, pose smoothing/previous ROI tracking remain enabled,
segmentation and refined face landmarks are off. Existing bundled assets are
hashed; no install/download was performed.

This explicit variant selects the current **body-right branch** when current
body, accepted right-hand presence, and all 21 image/world points are available.
It does not use the standalone raw-label inversion and does not claim persistent
person identity. Raw handedness is retained. The pinned upstream detector file
is unchanged; its exact all-21 parsing, wrist centering, SVD orientation and
right MANO math are reused without constructing or calling a Hands detector.
World points remain model-relative; no camera-global XYZ is inferred from them.

All **2,857 source attempts** are retained: **2,849 chronological original RGB
frames**, seven initial capture failures, one unchanged corrupt final archive
(2856). Every readable archive/RGB hash matches the unchanged baseline input
before the one Holistic graph call. There are no internal gaps, omitted hard
frames, second inference calls, or cached missing hand points. Original capture
timestamps and saved baseline probes are retained.

## RGB availability and losses

| Readable-frame result | Count |
| --- | ---: |
| Original detector selected | 2,384 |
| Holistic body-right selected | 2,113 |
| Both selected | 2,057 |
| Newly selected by Holistic | 56 |
| Original selection lost by Holistic | 327 |
| Neither selected | 409 |

The recorded original selected count remains **2,385**, including the final
selected frame whose archive cannot be replayed. The fair independently
reprocessed denominator is 2,849 frames. Of **1,906** readable original frames
with a recorded depth-supported pose, **13** lose hand selection in Holistic.
The other 314 lost RGB selections previously lacked such a pose. This is why
327 versus 56 alone cannot settle the global-following tradeoff.

| Fixed recorded interval | Holistic selections |
| --- | ---: |
| Distractor-hand block 9–164 | 16/156: frame 9 and 150–164 |
| Palm-down label-flip block 2435–2468 | 34/34 |
| Long no-hand block 1550–1744 | 0/195 |

All **34 palm-down gains still carry raw hand label `Right`** (which the original
standalone Right-hand wrapper rejects). Body-side association, rather than an
improved raw handedness classifier or threshold relaxation, accounts for this
change. Across all 2,113 selected Holistic outputs the raw labels are 2,065 Left
and 48 Right. Representative overlays place the selected landmarks on the
visible target hand; this remains recorded evidence, not a general identity
guarantee through occlusions or crossings.

## Crop and geometry quality audit

All 327 losses have current body output, a current right-hand tracking ROI and
an admitted pose-hand ROI, but **right-hand presence is rejected**. Scores range
from **0.0000828 to 0.757294**, median **0.010408**, P95 **0.322895**. This does not
support lowering the threshold as a repair.

The original detector's 21 image points are useful comparison estimates for ROI
coverage, though they are not anatomical ground truth. For the 327 lost frames,
the final Holistic ROI contains a median **11/21** original landmarks; only
**8/327** contain all 21, and **228/327** exclude the original wrist. For the
2,057 common selected frames, median coverage is **21/21**, with **1,961** fully
covered. This is consistent with the crop failure described in the investigated
paper and directly visible in the saved examples.

All 13 previously pose-valid losses have some original landmarks outside the
ROI, and **10/13 exclude the original wrist**. Their sequences are 886, 959,
960, 964–969, 1956, 2345, 2520 and 2525. Examples 886/959/966/1956 show visible
hands truncated by the crop; 2345 also retains a difficult fingers-down pose.
These failures remain in every denominator. No pose range is narrowed.

The independent numerical audit reconstructed all **2,113** selected probes
from saved raw points using the pinned original math: image coordinates match
exactly, maximum local-coordinate difference is **2.9e-16 m**, and rotation
matrix difference **3.6e-15** under the separate NumPy test environment. Every
selected frame has 21 image/world points, current body output and accepted
presence; no left-branch substitution or threshold violation was found.

## Unchanged-depth A/B and reference ambiguity

`live-01-holistic-depth.jsonl` replays both original and Holistic probes through
the exact recorded version-3 `wrist_surface_fit`, original aligned depth,
SDK-deprojected intrinsics/distortion and explicit geometric wrist support mask.
Estimator/source/archive hashes are verified. **All 1,906 readable baseline
positions reproduce exactly**, with zero availability or coordinate mismatch.
All 2,857 attempts remain present; the source's 1,907 saved poses include the
unreadable final frame, which is not counted as a successful independent replay.

Holistic gives **1,920** accepted current-depth results: **1,892 common, 28 gains,
14 losses**. Of the gains, 23 occur where the original detector did not select
a hand, and five where the original detector selected a hand but its depth fit
failed. The fourteenth loss is 2536, where Holistic still selects the hand but
the unchanged depth estimator rejects its wrist ray.

- Distractor block: **16/156** depth-supported positions.
- Palm-down block: **4/34**, only 2444, 2446, 2465 and 2466. The other 30 remain
  unsupported by current depth despite the recovered RGB hand.
- Long no-hand block: **0/195**.
- Longest valid run: baseline **578 frames / 29.613 s**, Holistic **567 / 29.046 s**.
- Longest invalid run: baseline **419 frames / 20.876 s**, Holistic **420 / 20.942 s**.

Neither path exceeds the unchanged position jump condition
`step_m > 0.015 + 1.5 * actual_dt_s` on eligible adjacent valid pairs (baseline
1,880; Holistic 1,891). Holistic's maximum step is **98.61 mm** over **133.44 ms**.
These include actual motion; they are not precision/jitter errors or a control
replay. Full availability runs and every guard input are saved.

This fusion comparison isolates wrist position. It does not reproduce the
vision worker's `direct_joint_angle` gripper postprocessing. A future control
replay needs the saved online baseline aperture or an explicitly matching
adapter; these results do not validate live gripper/collision behavior.

Common-frame surface-position difference has median **0.063 mm**, P95 **2.232
mm**, but maximum **179.1 mm** at frame 2550. Inspection of 2541–2553 shows the
same visible right limb, not an opposite-hand/person swap. At 2550 the baseline
wrist pixel is **(272.6,353.5)**, on the proximal forearm; Holistic's is
**(417.4,307.6)**, near the hand base and body right-wrist prior
**(428.5,315.8)**. Both local surface fits pass, at Z **0.451 m** and **0.390 m**
respectively. They measure different surface locations. The visual evidence
supports investigating baseline wrist mislocalization, but there is no
anatomical ground truth proving either metric estimate accurate. Sustained
reference errors can pass both depth-support and adjacent-jump checks.

See `live-01-holistic.reference-difference.jpg` for these source-image overlays.
A baseline-first fallback would preserve the original reference on these frames;
availability gains alone would not repair that localization issue.

The named offline baseline-first candidate was also evaluated from these saved
results: **1,929** positions (+23, zero baseline losses), but the palm-down gains
remain isolated and one source switch jumps **156.33 mm**, above the existing
**115.04 mm** interval bound. Its longest valid/invalid runs are unchanged. See
`live-01-hybrid.report.md`; this does not justify runtime promotion.

## Timing, validation and artifacts

Holistic graph call median/P95/max: **32.310 / 34.776 / 104.129 ms**. Including
raw serialization and the original orientation adapter: **32.996 / 35.686 /
104.871 ms**. Total run **106.986 s**. Root's separate short virtual-time Bullet
check completed during preparation/early run; no clean benchmark or live
end-to-end latency claim is made. The original recorded-input Hands comparison
was 12.003/14.025 ms median/P95.

**16 focused pure tests passed in 0.03 s** (14 adapter/configuration, two complete
summary-denominator tests) before the run. Graph configuration validation
separately resolved all 19 stream types and checked the four modified gates
without running images. Full replay completed with exit 0. No devices, runtime
change, motor command, depth-threshold relaxation or physical validation occurred.
The separate unchanged-depth comparison completed in **15.29 s** with **five
focused tests passed** and no new model inference.

- `live-01-holistic.jsonl`: every attempt, both hands, body, scores, ROI,
  selected probe and complete baseline comparison.
- `.summary.json`, `.inputs.json`, `.log`: accounting, timing, source/model/input
  hashes and retained final archive error.
- `.analyzer.py` and `.adapter.py`: exact executed source snapshots.
- `.roi-audit.json`: all baseline-point coverage checks, including losses.
- `.examples.jpg`: **green** Holistic right hand, **magenta** original selected
  hand, **cyan** body right arm, **white** final hand input ROI. Colors describe
  estimates, not ground-truth correctness.
- `holistic-probe-configuration-audit.json`: installed graph/stream/gate audit.
- `live-01-holistic-depth.jsonl`, `.summary.json`, `.inputs.json`: all unchanged
  fusion observations/diagnostics, gains/losses, complete runs and guard inputs.

```sh
.venv-vision/bin/python tools/analyze_holistic_recording.py outputs/hand-follow/20261106-093702-636085eb/camera --baseline outputs/position-research/live-01-detector-reprobe.jsonl --out outputs/position-research/live-01-holistic.jsonl > outputs/position-research/live-01-holistic.log 2>&1
```
