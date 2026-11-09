# Offline body-pose association: live01

**The two principal identity failure blocks are separable in this recording's
body/image geometry. This is an association diagnostic, not a recovered-control
or global-position result.** No threshold was fitted and no hand was relabeled.

Installed MediaPipe Pose 0.10.14 ran once on every readable original unmirrored
RGB frame with model complexity 1, detection/tracking confidence 0.8, segmentation
off, and its default pose smoothing. Both saved hand-candidate runs were reused;
no hand inference, hand-confidence change, runtime edit, installation, or device
access occurred. Every current archive/RGB hash was checked against both saved
candidate runs before processing.

## Complete accounting

- **2,857 attempts retained:** 2,849 Pose calls, seven initial capture failures,
  one terminal corrupt archive (2856). No silent internal skips.
- **2,843 body outputs; six missing:** 16, 19, 25, 290, 291, 320.
- Original raw hand candidate absent on **269** readable frames; max2 candidate
  absent on **266**. Body plus at least one hand candidate exists on **2,577**
  original / **2,580** max2 frames.
- All **33 body image and world landmarks** are retained, including visibility.
  The public Pose wrapper clears presence; that unavailable field is explicitly
  saved as `null`. Hip-relative body world points are never camera-global hand
  measurements. Capture timestamps remain the originals; the legacy API's
  internal simulated 30 Hz and smoothing are disclosed in the manifest.

## Block results

| Recorded interval | Body available | Image association of original raw candidate |
| --- | ---: | --- |
| Distractor 9–164, 156 frames | 153; 3 missing | All 153 nearer body **left** wrist |
| Palm-down label flip 2435–2468, 34 frames | 34 | All 34 nearer body **right** wrist |
| Long no-hand 1550–1744, 195 frames | 195 | No hand candidate in either detector run |

For the **distractor block**, raw hand-wrist distance to body left wrist has
median **7.39 px**, versus **511.48 px** to the body right wrist. Every available
comparison favors left by at least **156.34 px**. Left-wrist visibility is
0.988–0.996. Inspected overlays 14, 86 and 164 connect this hand to the left arm
at image right; frame 16 explicitly has no body association. The max2 run's
15 newly separated target candidates are nearer the body right wrist.

For the **palm-down block**, candidate wrist-to-body-right distance is
**2.95–13.76 px**, median **5.12 px**, and median distance divided by projected
right forearm length is **0.079**. Right-wrist visibility is **0.983–0.991**.
The index/pinky digit-chain proximities are also recorded independently; they
are not combined into a tuned score. Overlays 2435, 2452 and 2468 place body right
wrist beside the actual target wrist despite the raw hand label remaining
`Right` (which the original standalone wrapper filters).

The other body wrist is outside the image throughout that palm-down block, so
its large distance is **an extrapolated model prior**, not a second observed
hand measurement. The useful positive evidence is the close, visible right-arm
association; relative distance alone must not be promoted into an identity
guarantee. Over the full body-available recording, right wrist is outside image
bounds in **19** frames and left wrist in **2,152**.

The **long no-hand block** remains unresolved. Body right-wrist visibility
ranges **0.251–0.622**, median **0.401**, with six wrist predictions outside the
image. There are no 21-point hand candidates in any of its 195 frames. Body
inference does not create observed hand geometry or registered depth evidence.

## Ambiguity and limits

All 2,384 readable original selected frames have body output and their candidate
wrist is nearer the body right wrist. That is recorded consistency, not
independent ground-truth validation.

For max2, **28 frames disagree** between nearest-right-wrist ranking and nearest
right-index/pinky-chain ranking; there are **zero exact distance ties**. Of the
35 duplicate expected-label candidate frames, **both candidates are nearer the
body right wrist in all 35**, and wrist/digit rankings disagree in **27**. Thus
body-side identity does not resolve duplicate estimates of one physical hand.
Examples 893, 2341, 2536, 2550 and 2553 show this ambiguity. No candidate is
accepted/rejected by these rankings, and no threshold is tuned to claim recovery.

The report uses pixel distances, projected forearm normalization, visibility,
and separate index/pinky-to-digit-polyline distances. Pose finger landmarks are
not assumed identical to hand MCP or fingertip joints. No pose-world to
hand-world global-distance comparison is made. Occlusion, crossing, wrong body
pose, and absent candidates remain potential failure modes.

## Timing, validation and next use

Pose-call timing: median **19.115 ms**, P95 **22.206 ms**, maximum **85.809 ms**;
whole run **67.156 s**. This would be additional computation alongside the hand
model. These are offline measurements, not a live scheduling/latency guarantee.

**Six pure geometric/serialization tests passed in 0.01 s** before this
complete replay. They cover unmirrored side geometry independent of the raw hand
label, missing body, degenerate forearm, exact candidate ties, digit-polyline
distance, all-33 preservation/unavailable presence, and distinct missing-body
versus missing-candidate denominators. Source review independently checked
input-hash matching and candidate-run provenance. No physical validation occurred.

The justified next comparison is an explicitly separate, instrumented Holistic
run with both hand-presence gates kept at 0.8. It can test whether body-guided
hand crops preserve the favorable association without worsening the known
palm-down ROI failure. Neither experiment addresses missing D455 depth by itself.

Artifacts:

- `live-01-pose-association.jsonl`: every attempt, full body outputs, both candidate analyses.
- `.summary.json`: all denominators, block counts and timing.
- `.inputs.json`: identical-RGB/archive hashes, model/source hashes and configuration.
- `.metrics.json`: continuous distances/visibility, missing-body and ambiguous sequences.
- `.examples.jpg`: **cyan** body right arm, **magenta** body left arm, **green/yellow**
  max2 raw candidate indices 0/1. Colors indicate algorithm outputs, not truth.
- `tools/analyze_pose_association.py` and `tests/test_pose_association.py`: reproducible
  offline analysis and pure tests.

```sh
.venv-vision/bin/python tools/analyze_pose_association.py outputs/hand-follow/20261106-093702-636085eb/camera --baseline outputs/position-research/live-01-detector-reprobe.jsonl --max2 outputs/position-research/live-01-detector-max2.jsonl --out outputs/position-research/live-01-pose-association.jsonl > outputs/position-research/live-01-pose-association.log 2>&1
```
