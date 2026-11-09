# Automatic gripper following

Close other camera applications, then run:

```bash
cd /home/bx/repos/hand-control
PYTHONPATH=src .venv-core/bin/python tools/gripper_test.py
```

The single window shows only the upright gripper on the left and the original full camera image on the right, with joints drawn on the hand.
The image is letterboxed to fit; there is no crop, zoom, background removal,
rotation or mirroring.

Show your right hand and open/close your thumb against the palm. The
SO101 gripper model follows immediately. There are **no calibration, capture,
load, save or engage steps**. Use `--hand Left` if needed. Q quits; Space is an
optional pause/resume control. Closing the camera preview with Q closes both.
Each launch writes a separate directory under `outputs/gripper-preview`.

This is a simulation-only kinematic preview, using the complete official SO101
follower model and its actual joint and visual transforms. The arm stays fixed.
The camera uses color only, so wrist depth cannot prevent finger following.
The original audited dex-retargeting detector, RGB preprocessing, handedness
handling and all 21 points are retained.

The preview now runs the **actual upstream `VectorOptimizer` and
`SeqRetargeting`**, with its low-pass filter (alpha 0.2). Only the SO101 `gripper`
joint is optimized. Palm orientation comes from wrist and MCP landmarks
0,5,9,13,17; finger tips cannot move the fixed reference. The thumb vector 2→4
is expressed in that palm frame and scaled to the robot jaw lever length.
The fixed-side pivot and moving-side distal marker are matched using official
URDF forward kinematics. Markers do not change the official joints or meshes.
The palm-forward direction corresponds to the closed joint limit; outward
thumb rotation opens the jaw. This is direction retargeting, not a measurement
of skin contact. Thumb movement outside the robot's one-joint opening plane
cannot be reproduced exactly.

Unlike the previous signed-angle formula, the joint value comes from upstream
constrained vector optimization. Its actual range includes the negative lower
limit (-10 degrees); 0% now means that lower limit, not zero radians.
No per-launch hand calibration is required. The UI displays the actual joint
angle and normalized opening.

The optimizer runs as a child process using the existing
`/home/bx/repos/dex-retargeting/.venv/bin/python`. Vision and core environments
remain separate; no packages were installed or changed. The camera retains
its full original image with the upstream joint overlay.

Offline verification: `tools/validate_dex_gripper.py` checks official FK
recovery, full-range synthetic hand binding, rigid rotation/scale invariance,
finger-tip independence, and every recorded landmark frame. Results are in
`outputs/readiness/dex-palm-thumb/summary.json`. Both uploaded clips are
byte-identical, so this is not independent labeled open/closed validation.
View-dependent detector shape errors can remain; this change does not prove
live viewpoint robustness. This implementation is scoped to the gripper
preview, not the full-arm or hardware controller.


Missing/unselected/stale input holds the last displayed pose; a fresh detected
hand resumes this visualization automatically. The preview uses its own
`teleop.gripper_preview.v1` protocol with `simulation_only=true`, not the physical
robot observation protocol. This behavior does not alter full-arm release/
re-engage requirements and never starts or commands the physical robot.

## Ordinary arm simulation also needs no gesture calibration

The RGB-D worker now defaults to its original direct-angle mapping for its aperture
channel. Optional legacy closed/open endpoint arguments are still supported,
with a distinct geometry fingerprint, but are not required. Wrist XYZ remains
necessary for **arm** movement.

Terminal A:

```bash
cd /home/bx/repos/hand-control
PYTHONPATH=src .venv-vision/bin/python tools/vision_rgbd.py \
  --detector-repo /home/bx/repos/dex-retargeting --hand Right \
  --display --mirror-preview --position-method surface_wrist --depth-size 640x360 \
  --out outputs/mirror-arm-02 --vision-file outputs/mirror-arm-02/vision.json
```

Terminal B:

```bash
cd /home/bx/repos/hand-control
PYTHONPATH=src .venv-core/bin/python tools/run_teleop.py --backend sim \
  --vision-file outputs/mirror-arm-02/vision.json --log outputs/mirror-arm-02/control.jsonl
```

E engages the full arm; R releases; X stops. After full-arm tracking loss, R then
E is still required. The initial mirror/first-person mapping is toward-camera →
robot forward (+X), up → robot up (+Z), mirror-right → robot right (-Y).

The previous raw recording showed depth holes, not only lost landmarks. Native
640×360 depth is an explicit near-hand option (color stays 640×480). Its improved
live availability is not yet established. The gripper-only preview above avoids
that dependency entirely, without presenting invented XYZ as measured depth.
