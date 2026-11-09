# Focused feasibility: body-guided hand identity with installed MediaPipe

Recommendation: **a small offline Holistic A/B is technically feasible without
installing packages or changing the upstream detector, but stock Holistic is not
a drop-in replacement and is not an established fix for the fingers-down poses.**
The first useful experiment would expose the installed graph's existing hand
world outputs and preserve the 0.8 hand-presence gates, then replay the complete
live01 sequence. An even narrower identity diagnostic can run Pose alone and
compare its right/left arm associations against the already-saved raw hand
candidates, without changing any hand inference.

No image inference, device access, installation, or runtime change was performed
for this feasibility check. The original full-sequence one-hand and max2 results
remain the authoritative measurements.

## What is installed and reusable

The installed version is MediaPipe **0.10.14**. Legacy
`mediapipe/python/solutions/holistic.py` publicly returns 21 normalized image
landmarks for each hand, plus pose/face outputs. Its only public world-coordinate
output is the **33-point body pose**, centered at the hips. There is no public
21-point hand-world output in that wrapper. Normalized hand `z` and hip-centered
pose world coordinates cannot substitute for the original adapter's 21 local
hand-world points or for camera-global wrist position.

However, read-only expansion of the bundled legacy graph found **202 nodes**
and the full 21-point hand-world output inside each hand subgraph:

```text
handlandmarksleftandrightcpu__handlandmarksfromposecpu_1__handlandmarkcpu__hand_world_landmarks
handlandmarksleftandrightcpu__handlandmarksfromposecpu_2__handlandmarkcpu__hand_world_landmarks
```

The first belongs to the left-body-hand ROI and the second to the right-body-hand
ROI. Both registered packet types are `::mediapipe::LandmarkList`; they follow
`WorldLandmarkProjectionCalculator`. An independent read-only feasibility check
constructed `CalculatorGraph` and successfully attached observers to both
streams, **without calling `start_run`, submitting any image, or running
inference**. `SolutionBase` accepts these registered internal outputs, so an
explicit research wrapper can expose them without rebuilding MediaPipe or
running a second hand inference. The required default `model_complexity=1`
models are already bundled. Lite/heavy pose assets are absent; their constructor
paths can download models and should not be used for this no-install experiment.

The installed **Tasks Holistic** API does explicitly expose both hand-world
lists and a separate hand-confidence option, but requires an external
`holistic_landmarker.task` bundle. None was found in the installed package or
project assets. Switching to Tasks is therefore not the smallest available
experiment. [Legacy graph](https://github.com/google-ai-edge/mediapipe/blob/v0.10.14/mediapipe/modules/holistic_landmark/holistic_landmark_cpu.pbtxt),
[hand model outputs](https://github.com/google-ai-edge/mediapipe/blob/v0.10.14/mediapipe/modules/hand_landmark/hand_landmark_cpu.pbtxt),
[Tasks API](https://github.com/google-ai-edge/mediapipe/blob/v0.10.14/mediapipe/tasks/python/vision/holistic_landmarker.py).

There is an additional installed Tasks-wrapper caveat: its image/video/callback
paths return an empty result when the face-landmark packet is empty. It should
not be assumed that a usable hand can survive missing face output in this exact
version. The detailed local graph/source fingerprint and no-inference observer
checks are saved in `holistic-installed-graph-audit.json`.

## Threshold trap and explicit adapter requirements

Calling legacy `Holistic(min_detection_confidence=.8,
min_tracking_confidence=.8)` sets **person detection and pose presence**, not
the hand-presence thresholds. Its two internal hand gates remain **0.5**. Our
standalone Hands baseline explicitly overrides its hand gate to **0.8**.
Therefore a plain public-API Holistic comparison would silently relax the hand
gate despite identical-looking constructor numbers.

A fair explicit wrapper must additionally set these calculator options to .8:

```text
handlandmarksleftandrightcpu__handlandmarksfromposecpu_1__handlandmarkcpu__ThresholdingCalculator.threshold
handlandmarksleftandrightcpu__handlandmarksfromposecpu_2__handlandmarkcpu__ThresholdingCalculator.threshold
```

Holistic also has separate pose-wrist visibility gates at **0.1**. These are ROI
admission checks, not evidence of reliable anatomical identity or measured wrist
depth. Log the pose visibility and associations; do not interpret an admitted
ROI as ground-truth identity. The public confidence options and original hand
gate semantics are visible in the pinned
[Holistic wrapper](https://github.com/google-ai-edge/mediapipe/blob/v0.10.14/mediapipe/python/solutions/holistic.py)
and [Hands wrapper](https://github.com/google-ai-edge/mediapipe/blob/v0.10.14/mediapipe/python/solutions/hands.py).

For an offline adapter, retain all 21 normalized and all 21 world points per
hand; subtract the hand wrist origin and reuse the original orientation/MANO
transformation math. Label the variant **body-associated Holistic**, because its
left/right assignment comes from body ROI identity rather than the original
standalone handedness filter. Do not silently re-label baseline output or carry
old points forward on missing output. Any current global XYZ must still pass the
unchanged recorded-depth estimator; body-world or normalized model depth is not
a replacement for registered metric depth.

Body-side mapping also differs from the standalone selfie-label convention:
legacy left-hand ROI uses pose indices 15/17/19 and right-hand ROI uses
16/18/20. Do not apply the standalone `Right -> raw Left` label inversion to
these body-side branches; verify the unmirrored association explicitly.

## Why this might help, and the exact pose-related risk

Holistic derives separate hand ROIs from the body pose on each frame, followed
by hand recropping and hand landmark inference. The official design specifically
uses body context to preserve semantic left/right association and improve
relocalization. That targets the observed distractor-hand occupancy and may
avoid relying on the unstable standalone handedness classifier. It tracks the
most prominent person, however; this is not a persistent person/hand identity
guarantee through crossings, occlusion, or body-pose failure. The full legacy
pipeline also runs pose and face models, so latency must be measured here.
The documentation's approximately 10% hand-recrop overhead is relative to the
hand model, **not** the total Holistic overhead or our end-to-end latency.
[Official Holistic design](https://chuoling.github.io/mediapipe/solutions/holistic.html).

The cited paper identifies a directly relevant weakness: the three-point 2D
pose-to-hand ROI can shrink or misorient when the palm plane is perpendicular
to the camera. It adds shoulder/elbow/thumb/depth features and evaluates tiny
MLPs on 1,912 training/846 test hand annotations. Reported ROI IoU improves
**57% to 63%**, while rotation error worsens **32.08 to 56.96 degrees**. The author
recommends learned center/size with the original rotation heuristic. This is
ROI-quality evidence, not demonstrated temporal hand tracking, identity safety,
metric wrist precision, or D455 fingers-down reliability.
[Paper](https://arxiv.org/html/2405.03545v2).

The installed-version source still uses a projected center weighted from index
and pinky, with box size twice the projected wrist-to-center distance, then a
2.7 scale and -0.1 shift. Foreshortening can therefore make the crop small even
when the body wrist is detected. Its hand path uses the pose ROI, recrop and
previous hand tracking; it does not invoke a standalone palm detector fallback
in that subgraph. The author's primary
[issue report](https://github.com/google-ai-edge/mediapipe/issues/5373) specifically
demonstrates the parallel-to-floor case.
[ROI calculator](https://github.com/google-ai-edge/mediapipe/blob/v0.10.14/mediapipe/modules/holistic_landmark/calculators/hand_detections_from_pose_to_rects_calculator.cc),
[ROI expansion](https://github.com/google-ai-edge/mediapipe/blob/v0.10.14/mediapipe/modules/holistic_landmark/hand_landmarks_from_pose_to_recrop_roi.pbtxt),
[hand path](https://github.com/google-ai-edge/mediapipe/blob/v0.10.14/mediapipe/modules/holistic_landmark/hand_landmarks_from_pose_cpu.pbtxt).

The paper's code is real and includes three small TorchScript weight files.
Its evaluation script compares ROI boxes; it is not a ready-made Holistic graph
patch. The published evaluator uses the learned rotation too, whereas the paper
recommends a hybrid. Importing that evaluator also loads its dataset and runs
evaluation. Porting the predictor into a live ROI graph would require explicit
engineering and validation. There is no reason to install its training stack
before showing that our actual failures have the predicted ROI geometry.
[Author implementation](https://github.com/sign-language-processing/mediapipe-hand-crop-fix/blob/main/mediapipe_crop_estimate/evaluate.py),
[weights](https://github.com/sign-language-processing/mediapipe-hand-crop-fix/tree/main/mediapipe_crop_estimate/mlp),
[training](https://github.com/sign-language-processing/mediapipe-hand-crop-fix/blob/main/mediapipe_crop_estimate/train_mlp.py).

## Minimal useful next test

Run one explicit offline instrumented legacy graph over **all 2,857 attempts**
with the same 2,849 readable RGB frames and retained seven capture failures and
one terminal corrupt archive. Keep 0.8 hand gates, full model complexity 1,
original RGB orientation and all 21 outputs. Record both hands, body landmarks,
pose visibility, raw hand-presence scores, pose/recrop/tracking ROIs, actual
capture timestamps and per-call timing. Missing outputs remain missing.

Judge the experiment separately on the 156-frame distractor block, 34-frame
palm-down label-flip block, 195-frame close/edge no-hand block, and all previously
valid baseline frames. Inspect explicit body-hand associations and duplicate
hypotheses, not just detection counts. The palm-down block can improve from
body-based assignment **or worsen from the documented ROI failure**; neither
outcome is established without replay. Neither Holistic nor the ROI paper fixes
the independent 478 recorded depth-support failures.

If only the identity hypothesis needs testing first, run Pose alongside the
saved raw baseline/max2 candidates and assess body-wrist/arm association as a
diagnostic. That retains their exact existing hand-world estimates and isolates
identity from a changed crop model. It must remain a separately named analysis,
not a silent bypass of original handedness selection. No proposed route is
ready for physical control without recorded evidence and subsequent validation.
