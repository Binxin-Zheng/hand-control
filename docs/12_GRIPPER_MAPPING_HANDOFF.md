# Gripper mapping problem — handoff

## Desired behavior

Supervised lab demo, automatic single-joint SO101 gripper visualization. Thumb
against the palm means exactly 0 rad. Thumb comfortably away means the model's
maximum opening (~1.74533 rad / 100 degrees). Intermediate motion should produce
smooth proportional opening. Preserve natural fingers-down poses, rotations,
translations and all 21 upstream keypoints. No per-launch gesture calibration,
capture endpoints or engage button. The accepted UI is one window, gripper left,
cropped hand right, both displayed upward. Do not change that UI to fix mapping.
Physical robot commissioning is separate from this mapping problem.

## Current implementation and failure

`src/anyteleop_so101/gesture.py::thumb_palm_opening` computes minimum distance from
thumb tip (4) to finite triangles (0,5,9), (0,9,13), (0,13,17), divided by palm
width ||p5-p17||. It uses unvalidated fixed endpoints 0.15 and 0.85:

    opening = clip((distance_ratio - 0.15) / (0.85 - 0.15), 0, 1)
    q_gripper = opening * URDF_gripper_upper

This is a heuristic, NOT dex-retargeting optimization and NOT fitted from the
user's data. Model landmarks are not a measured skin surface. A folded thumb
does not necessarily lie on these triangles, especially with model errors or
occlusion. Increasing the endpoints blindly would not establish gesture semantics.

User reports almost always fully open, unable to close fully. Re-evaluating the
942 detected frames from the supplied shared recording gives:

- normalized distance min / P05 / median / P95 / max:
  0.1687 / 0.4349 / 0.7832 / 1.2236 / 3.1838;
- median opening: 0.9046;
- exactly fully open: 347/942 frames;
- exactly zero: 0/942 frames.

These numbers substantiate a failed mapping, not a successful calibration. Unit
tests previously checked mathematical endpoints/invariance, not real-gesture
classification. They did not establish that the selected feature/defaults fit
the user. Do not describe the current proximity rule as validated.

## Data and labels

`sample/hand-guesture/close.webm` and `open.webm` remain byte-identical, SHA-256
`3a4f63696e003689c7263887efcb98a10d2a8e0d32b9a65e4df590bbe94a934c`.
Both are 1,629,092 bytes. They cannot simultaneously label every identical pose
closed and open. Need distinct videos or explicit interval labels within the
shared video before evaluating endpoint separation; do not invent labels from
the current metric or delete difficult frames. A model-based prototype can be
developed independently, but its endpoint accuracy cannot be claimed from these
contradictory labels.

`outputs/gesture-analysis/shared.jsonl`: all 981 decoded frames over 35.1 s,
942 selected-right-hand detections and 39 unselected/no-detection frames, full
21-point local geometry, wrist rotation, pixel landmarks and timestamps.
Container frame-count metadata estimated 1,054; actual decoded count is 981.
`shared-annotated.mp4`, `shared-features.jsonl`, `feature-summary.json`, and
`ANALYSIS.md` are alongside it. Feature summary includes angle, thumb/index
separation and palm centroid/plane distances. Source detector blob is
4867ad7d2e5ea3984002c1730b937d06b1fe97ee; original preprocessing is preserved.
Additional full RGB/depth recording: `outputs/depth-prep-01` (900 attempts).

## What local dex-retargeting actually does

Source root: `/home/bx/repos/dex-retargeting`.

- `example/vector_retargeting/show_realtime_retargeting.py`: detector produces
  local 21-point geometry; constructs configured target vectors; calls
  `retargeting.retarget(ref_value)`; applies returned joint positions each frame.
- `src/dex_retargeting/configs/teleop/panda_gripper.yml`: vector mode, one target
  joint `panda_finger_joint1`, robot vector from `panda_leftfinger` to
  `panda_rightfinger`, human vector landmark 4 → 8, scale 1.5, LP alpha 0.2.
- `optimizer.py::VectorOptimizer`: compute robot FK and selected link vectors
  for candidate q; penalize vector mismatch with SmoothL1/Huber loss; use robot
  Jacobians and NLopt SLSQP with model joint limits. Previous q initializes the
  solve. The implementation adds a last-q regularization term to its gradient;
  do not claim a different idealized loss was literally implemented.
- `seq_retarget.py`: retain previous solution; handle model adaptor/mimic joints;
  optionally low-pass filter output. `LPFilter`: y += alpha * (x-y).
- `panda_gripper_dexpilot.yml` selects DexPilot instead. Its optimizer has
  pinch-distance projection/hysteresis and weights for fingertip pairs. This is
  not palm-contact recognition, nor the same as the plain vector configuration.

There is no universal human-joint-angle to robot-joint-angle equality here.
Robot model, selected frames, coordinate alignment, scaling and filtering are
explicit configuration. Smoothness alone does not establish intended contact.

## Adaptation direction to investigate

Reuse actual upstream vector/DexPilot machinery where appropriate, or a clearly
equivalent bounded 1D geometric solve for SO101's single revolute jaw. Define
actual fixed-pad and moving-pad/tip reference points in the official model and
verify their FK motion across the joint range. SO101's moving-jaw link origin is
at its joint: using that origin directly as the tip can give an uninformative
position target. Do not merely swap Panda's URDF path and retain its frame names.
Preserve official joint.origin, joint.axis and visual.origin/meshes.

The human control signal still must represent thumb-to-palm opposition, not
silently replace the requested gesture with thumb/index pinching. Evaluate
candidate contact/opposition features in a hand-local frame against labeled
intervals, including fingers-down, side views and rotations. Fixed defaults may
be learned once and shipped, with no runtime calibration. Separate gesture
recognition from geometric jaw-gap/joint-angle conversion. Do not promise that
an optimizer will recover palm contact absent from uncertain landmarks.

Evaluate endpoint false-open/false-close rates, output jitter within static
poses, intermediate monotonicity, lag and pose sensitivity on held-out temporal
blocks. Retain all failed detections in denominators and report them separately.
Do not lower tracking thresholds, discard difficult frames or label historical
predictions as current measurements to improve results.

## Current entry points

`tools/gripper_test.py`: core/PyBullet renderer; official full model with non-gripper
links hidden for presentation; simulation-only automatic following.
`tools/vision_gripper.py`: isolated vision worker, original detector, current
proximity feature, combined OpenCV UI. Protocol: `teleop.gripper_preview.v1`.
`src/anyteleop_so101/runtime/preview_ui.py`: presentation transforms only.
`tools/vision_rgbd.py`: separate full-arm input path; still uses the earlier
direct-angle default. Do not accidentally conflate these two mapping paths.

Launch from `/home/bx/repos/hand-control`:

    PYTHONPATH=src .venv-core/bin/python tools/gripper_test.py

Core tests: `PYTHONPATH=src .venv-core/bin/python -m pytest -q`.
Replay: `PYTHONPATH=src .venv-core/bin/python examples/replay.py outputs/readiness/offline/trace.jsonl`.
Keep core, vision and robot SDK environments isolated. No motor access is needed
to solve or evaluate the gripper visualization mapping.
