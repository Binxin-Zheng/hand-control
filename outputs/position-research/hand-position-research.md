# Hand-position research and offline comparator audit

2026-11-06. Scope: reliable correct-direction D455-driven SO101 translation;
metric precision is a later improvement. Research is separated from hardware
validation. This audit changed no repository files and accessed no devices.

## Recommendation

Retain the exact 21-keypoint upstream detector and preprocessing. Add a separate
current-frame palm-surface estimator to use the depth information already present
inside the palm, rather than requiring valid tiny patches at three anatomical
joint pixels. Give its output an explicit surface-reference identity. Keep it
separate from the existing model-wrist estimate; never switch reference identity
during an engagement. The saved recording supports testing this direction, but
does not establish absolute accuracy or successful live robot following.

The strongest objection to the current model-translation baseline is semantic:
`depth_point_i - model_joint_i` treats visible skin and a model joint center as
corresponding 3D points. Their difference is not necessarily the same translation
for all palm joints. Rotation, finger occlusion, hand-model scale errors and palm
curvature can change that difference. Relaxing a residual threshold cannot fix
the underlying correspondence assumption.

The RGB-only reprojection alternative is theoretically useful and inexpensive,
but the actual saved detector geometry is insufficiently consistent with the
2D observations. Independent tests below do not support using it to fill depth
gaps or drive the arm now.

## What the upstream systems establish

### AnyTeleop

Primary source: https://arxiv.org/html/2307.04577v3#S4.SS1
Project: https://yzqin.github.io/anyteleop/
Audited detector blob: `4867ad7d2e5ea3984002c1730b937d06b1fe97ee`.
Local file: `/home/bx/repos/dex-retargeting/example/vector_retargeting/single_hand_detector.py`.

AnyTeleop separates local finger geometry from camera-frame wrist pose. RGB-D
uses aligned depth at keypoints and intrinsics. RGB-only adds a weak-perspective
scale network. Its multi-view method selects relative motion from a view using
hand-shape consistency; it is not simply calibrated stereo triangulation. The
paper acknowledges self-occlusion and fast-motion failures. Its demonstrations
and task success support feasibility, not a general millimetre wrist-accuracy
guarantee. The public dex detector subtracts wrist keypoint 0 and returns local
points, 2D landmarks and rotation; global translation is absent.

### OAT: translation from model-to-image reprojection

Paper record: https://eprints.whiterose.ac.uk/id/eprint/238695/
Accepted PDF: https://eprints.whiterose.ac.uk/id/eprint/238695/1/ICRA26_0903_FI.pdf

Sections III-B/D estimate wrist translation by projecting MediaPipe's relative
3D joints into the image and minimizing 2D mismatch, with temporal regularization.
Metric scale relies on the model and intrinsics; hand-size and depth/focal-length
ambiguities remain. The paper calls the system open source, but its PDF and
record provide no repository link; title/author searches did not locate one.
Its best real-detector FreiHAND wrist error is 4.277 cm and 84.67% of images are
unrecognized. The near-zero-error row uses ground-truth joints and intrinsics.
Its accumulated position/orientation grasp compensation is a separate control
heuristic and should not be copied into this project's latched pose mapping.

Our comparator implements the translation-only geometric idea independently; it
does not claim to reproduce the unretrieved OAT implementation or its temporal
regularization and control heuristics.

### DexPilot and DART: fit surfaces to surfaces

Primary paper: https://arxiv.org/html/1910.03135v1
Actual DART repository: https://github.com/tschmidt23/dart
Audited commit: `37adc66580a15c5b438986d77621ebd8f5f450c3` (2019-04-06).
Audited kernel:
https://github.com/tschmidt23/dart/blob/37adc66580a15c5b438986d77621ebd8f5f450c3/src/optimization/kernels/obsToMod.cu

DART associates observed depth vertices with model signed-distance surfaces,
using distance/normal checks and Huber loss. It does not equate surface depths
with bone-joint coordinates. Its stack is C++/CUDA, Eigen3, matheval, JPEG/PNG,
tinyxml and an active OpenGL context; no modern-build/D455 compatibility was
tested. DexPilot used four synchronized calibrated D415s, segmentation and
learned pose priors because DART alone suffered local minima. Its complete
tracking stack was not established as publicly available: the coauthor
repository https://github.com/ankurhanda/dexpilot contains a paper and README.

Additional audited details are in `/tmp/hand-depth-research.md`.

### Lightweight palm geometry precedent

Sridhar et al., ICCV 2013, Section 5.2:
https://www.cv-foundation.org/openaccess/content_iccv_2013/papers/Sridhar_Interactive_Markerless_Articulated_2013_ICCV_paper.pdf

The authors isolate palm depth with morphology, fit a RANSAC plane to reject
occluding finger points, then use PCA and filtering. Their complete system has
multiple calibrated RGB cameras, depth and a person-specific hand model. This
supports the geometric technique, not a performance claim for our one-D455 setup.

## Additional actual public code inspected

### DepthAI hand tracker

Commit: `97731232fffd467e8de2c3a34ab8382962bb385e` (2023-01-14).
https://github.com/geaxgx/depthai_hand_tracker/blob/97731232fffd467e8de2c3a34ab8382962bb385e/HandTracker.py

`spatial_loc_roi_from_wrist_landmark()` uses an adaptive ROI around landmark 0;
`query_xyz()` sends the ROI to the device's spatial depth calculator. Local
21-point world landmarks remain separate. Its stereo depth is aligned to RGB.
Dependencies are OpenCV >=4.5.1.48 and depthai >=2.13, using OAK-specific APIs.
This is independent code evidence for measured-depth anchoring, not a D455
drop-in or measured anatomical accuracy. We should not replace our detector
preprocessing with its OAK model pipeline.

### egoBench MediaPipe + PnP comparator

Commit: `ec5a9ac0da412dca6ce75be33ec68510766db8dd` (2026-07-14).
https://github.com/stray128/egoBench/blob/ec5a9ac0da412dca6ce75be33ec68510766db8dd/egobench/models/commercial_safe.py

`_pnp_camera_joints()` calls OpenCV SQPnP on 21 model/2D correspondences and
returns `R @ model + t`. It is a small, inspectable implementation of the
geometric idea. It is not a safe implementation to copy wholesale: nearby
comments describe MediaPipe world origin inaccurately; its detector path falls
back from world landmarks to normalized landmarks; and it does not include the
residual, positive-depth and conditioning guards required by our control path.
The code uses NumPy/OpenCV; its complete benchmark has many unrelated dataset
and model dependencies. No benchmark claims were independently reproduced.

OpenCV's primary geometry reference:
https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html

### RealSense measurement semantics

https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0

SDK projection/deprojection uses the stream's intrinsics, distortion and depth
scale. A zero depth is missing data. Color/depth image and coordinate frames are
separate until alignment/extrinsic transformation. The offline tests below use
the saved distortion parameters with `rs2_deproject_pixel_to_point`, rather than
assuming the saved images are pinhole-rectified.

## Geometry and observability: our analysis

For camera-oriented model-relative joints h_i and undistorted observed rays
(a_i,b_i,1), translation-only perspective fitting minimizes

    sum_i || project(h_i + t) - pixel_i ||^2.

An algebraic initialization stacks

    t_x - a_i t_z = a_i h_iz - h_ix
    t_y - b_i t_z = b_i h_iz - h_iy.

Its units inherit the scale of h_i. Good reprojection cannot establish correct
physical scale when the model hand is too large/small. A numerical solver's
success flag cannot establish observability or correct tracking. Inconsistent
model/image shape can move the optimum toward extreme depth, and a tightly
clustered image configuration weakens depth estimation.

For a fitted surface plane n·p=d and a semantic image ray r, the surface point
is p=r d/(n·r). The plane contributes depth; the semantic ray anchors tangential
position. A small |n·r| amplifies noise. A plane's visible-depth centroid is not
a stable anatomical/material reference: the visible set changes with rotation,
occlusion and missing samples. A wrist ray may require extrapolation beyond the
interior fitted palm, while a palm-center ray is better supported but represents
a different control reference. Neither automatically removes skin/joint bias.

A dense estimator should preserve at least the existing consistency strictness:
80% valid support/inliers, 12 mm point-to-plane limit, 3 mm two-dimensional
coverage; require a fixed interior semantic region, reject edge-on geometry,
mixed surfaces and insufficient spatial distribution, and record all support
diagnostics. A 2D hull is only a support assumption, not proof of visibility or
hand segmentation. Depth holes must remain holes; no historical sample or
extrapolated temporal prediction counts as a present measurement.

## Complete saved-recording comparison actually run

Input: `/home/bx/repos/hand-control/outputs/depth-prep-01/frames.jsonl`.
All 900 attempts retained: 5 capture errors, 93 no selected hand, 802 detections.
802/802 detector rows supply unchanged full 21-point geometry to every solve.
No pose classes, difficult frames or solver failures were omitted.

Commands in existing isolated vision environment:

    .venv-vision/bin/python /tmp/probe_reprojection.py
    .venv-vision/bin/python /tmp/probe_pnp.py

Dependencies actually loaded: NumPy 2.2.6, SciPy 1.15.3, OpenCV 4.11.0 and the
existing pyrealsense2 installation. No dependency changes were made.

Translation-only comparator results:

| Quantity | Quadratic reprojection | Soft-L1 reprojection |
|---|---:|---:|
| RMS reprojection median / P95 | 38.77 / 58.22 px | 39.04 / 59.65 px |
| RMS reprojection >10 px | 716/802 | 721/802 |
| Jacobian condition >10,000 | 2/802 | 2/802 |
| Median z disagreement with palm-plane wrist intersection | -116.1 mm | -124.7 mm |
| Adjacent same-track step median / P95 | 8.53 / 44.59 mm | 7.97 / 44.25 mm |
| Steps >60 mm | 30/798 | 25/798 |

There are 592 frames shared with the parent's current `/tmp/palm-probe.json`.
The plane is another estimator, not ground truth. Two translation-only solutions
escape toward enormous positive depth despite solver convergence (quadratic max
371,909 m). They are retained and diagnosed, not removed from metrics. No linear
systems were rank-deficient, illustrating why linear rank alone is insufficient.

Full rotation-plus-translation SQPnP followed by LM still gives median/P95 RMS
30.67/50.91 px, median 33.2-degree orientation correction, and 2 solutions with
nonpositive transformed joint depth. Median z disagreement is -179.1 mm.
Adjacent same-track step P95 is 37.62 mm; 13/798 exceed 60 mm. It does not provide
a compelling fallback or justify silently changing the upstream orientation.

The translation-only implementation was independently checked using a synthetic
exact-projection case with a known translation: both losses recovered position
within 4.2e-8 m, RMS reprojection 8.95e-6 px. This verifies the comparator's basic
algebra, not real camera/model accuracy. Reprojection pixels here are SDK-ray
errors scaled by fx/fy in an undistorted equivalent image. Reported frame steps
contain real motion and variable capture intervals; they are not static jitter.

Artifacts:
- `/tmp/probe_reprojection.py`
- `/tmp/reprojection-comparison.jsonl`
- `/tmp/reprojection-comparison.summary.json`
- `/tmp/probe_pnp.py`
- `/tmp/pnp-comparison.jsonl`
- `/tmp/pnp-comparison.summary.json`

## Decision for implementation

Proceed with dense current-frame palm-surface support as an explicitly named
experimental reference, compare to both existing methods on every saved frame,
and exercise artificial missing-depth/background/edge-on cases. Preserve model
orientation, all keypoints, tracking thresholds, mapping signs, release and
re-engage behavior. Do not count RGB-only model reprojection as depth evidence.

Measure correct-direction gain, open/close coupling, rotation coupling, pauses
and latency on full recorded/live simulation sessions. Current saved data lack
independent motion ground truth and direction labels, so neither a low surface
residual nor a smoother trace can establish successful directional control or
millimetre accuracy. No hardware accuracy, physical stopping or robot reliability
was tested in this research subtask.

## Appendix: D455 depth-hole hypotheses after the complete second recording

The second recording is `outputs/hand-follow/20261106-093702-636085eb/camera`.
Its [complete audit](live-01-ab.report.md) retains all 2,857 attempts, including
2,385 upstream selected-hand frames, 465 non-selections and seven capture-clock
errors. Of the selected frames, 1,906 positions were independently replayed,
478 failed the surface fit, and one final archive was incomplete. The source
JSONL observation for that final archive is retained, but is not counted as
independently replayed depth evidence.

All **478 selected surface-fit failures** were checked for finite depth in the
existing 0.05–2 m envelope within four image pixels of the wrist ray:

| Available near-ray depth | Failed frames |
|---|---:|
| No valid sample | 359 |
| Residual sample median below 260 mm | 43 |
| Residual sample median at/above 260 mm | 76 |

These residual values are not ground truth at a missing wrist. In particular,
the 359 frames with no near-ray depth cannot be assigned a wrist distance from
this analysis. Surviving coherent patches in examples 373, 451, 904, 1745 and
1817 cluster around 245–262 mm, consistent with a near-range contribution.
Fully framed natural poses 2271 and 2469 retain near-ray samples around 410 mm
and 340 mm, so minimum range alone cannot explain every failure.

The subsequent **read-only, non-streaming** sensor inspection is preserved in
[current settings](d455-current-settings.json) and
[advanced settings](d455-current-advanced-settings.json). It reports firmware
5.15.1.55, emitter enabled, laser power 150 mW with supported maximum 360 mW,
auto-exposure enabled, disparity shift zero, depth clamp 0–65536,
second-peak delta 325, and texture-count/difference thresholds zero. No setting
was changed. The exposure value reported outside streaming is **not actual
exposure metadata for the recorded frames**.

Three specific diagnostic hypotheses follow:

1. **Stereo search range contributes to the closest failures.** The official
   D455 table gives Min-Z 260 mm at native 640×360, 200 mm at 480×270 and 180 mm
   at 424×240; 520 mm applies to 1280×720. This recording explicitly used native
   640×360 aligned to 640×480 color. Compare supported native depth modes while
   retaining the same RGB detector preprocessing and natural hand motions.
   Disparity shift is another documented experiment, but it reduces maximum
   range sharply and should be isolated from other changes rather than applied
   as a blanket fix. [Official D400 datasheet, Table 4-11](https://realsenseai.com/wp-content/uploads/2025/09/Intel-RealSense-D400-Series-Datasheet-October-2025.pdf),
   [official tuning guidance](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/)

2. **Stereo visibility or depth-to-color reprojection contributes to in-range
   failures.** Save synchronized native unaligned Z16 depth, IR1, IR2, RGB,
   aligned depth, intrinsics/extrinsics, timestamps and frame IDs. Native depth
   present but aligned support missing points toward projection coverage or
   viewpoint occlusion; native depth already missing calls for matching, range
   and exposure checks. This is a diagnostic distinction, not a diagnosis from
   the current RGB/aligned-depth files. Alignment can leave holes, and perfect
   calibration does not eliminate different camera viewpoints.
   [Official projection and occlusion paper, §§3.4–4](https://dev.realsenseai.com/docs/projection-texture-mapping-and-occlusion-with-intel-realsense-depth-cameras/)

3. **Exposure or projected texture contributes to stereo matching failures.**
   Inspect both IR views for dark, saturated or blurred wrist regions; log
   actual per-frame exposure and gain before comparing supported laser/exposure
   settings. Increasing laser power may improve weak texture or worsen
   saturation. The RGB and stereo exposure controls are separate. Displayed
   8-bit IR clipping is suggestive, not conclusive, because the ASIC has access
   to higher-bit-depth data. [Official tuning guidance](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/),
   [official HDR/exposure paper, §§2.1–2.4](https://dev.realsenseai.com/docs/high-dynamic-range-with-stereoscopic-depth-cameras/)

The immediate evidence-gathering priority is **native depth plus both IR
streams**, keeping the existing natural poses and all failed frames. These are
supported diagnostic experiments, not demonstrated fixes. No pose range was
narrowed, estimator threshold relaxed, or filled/predicted depth counted as an
observation. This appendix's research used saved data and primary documentation;
it did not access devices or change code/settings.
