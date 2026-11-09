# Runtime: model download, simulation, RGB-D and hardware

The official SO101 model and D455 capture have now been exercised locally.
Physical hand-driven following remains unverified. Current procedures are in
`docs/09_D455_QUICKSTART.md`; current research/validation artifacts are under
`outputs/position-research/`. Earlier `outputs/runtime/VALIDATION.md` records
historical runtime tests. Numerical, camera-startup and hardware evidence remain
distinct.

## 1. Download the official model

Run these commands in a normal terminal with GitHub network access:

```bash
cd /home/bx/repos/hand-control
python3 tools/fetch_so101_model.py --out assets/so101
```

The downloader resolves one upstream commit, fetches the entire `Simulation/SO101` directory plus the repository license, verifies each Git blob, and writes `assets/so101/source-manifest.json`. It does not install software or access motors. It refuses to overwrite differing files; use a fresh destination for a different revision.

Expected layout (keep the nested `assets` directory):

```text
assets/so101/
  so101_new_calib.urdf
  so101_new_calib.xml
  scene.xml
  joints_properties.xml
  README.md
  source-manifest.json
  assets/
    ... official meshes ...
```

Source: https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101 . The simulator implemented here uses the official **URDF in PyBullet**. The downloaded MuJoCo scene is also retained, but this project's controller has no MuJoCo adapter. The small kinematics-only URDF is deliberately rejected by the runtime.

## 2. Simulator environment and startup

The existing `.venv-core` in this workspace can run the numerical tests and headless PyBullet adapter tests. For a normal reproducible install on your host, create a separate environment:

```bash
python3.12 -m venv .venv-sim
env -u PYTHONPATH .venv-sim/bin/python -m pip install -e '.[sim,test]'
env -u PYTHONPATH .venv-sim/bin/python -m pip check
PYTHONPATH=src .venv-sim/bin/python tools/check_model.py
PYTHONPATH=src .venv-sim/bin/python tools/run_teleop.py --backend sim --synthetic
```

`check_model.py` verifies mesh references, loads the complete URDF into Bullet, compares numerical FK position/orientation against Bullet, and reports initial-pose collision status. No physical robot connection occurs.

The simulator starts **released**. Type `engage` + Enter in the terminal, or press **E** in the simulation window. Synthetic motion is a small explicitly labeled test trajectory; no camera is required. **R** releases, **X** latches software E-stop, **C** resets E-stop only while released, and **Q** exits. Terminal equivalents are `release`, `estop`, `reset`, `quit`. After any tracking/collision/backlog failure, release then engage again; reappearance alone never resumes motion. Do not confuse the software E-stop with a physical stop device.

Use `--headless --duration 10` for an offscreen run. `--log` selects the JSONL control trace. The trace includes observations, measured joints/aperture, desired pose, actual commands, residuals and rejection reasons. A synthetic source is refused for the hardware backend.

`config/runtime.sim.json` supplies the full model path, fixed camera-to-base rotation, explicit translation scale, initial **simulated** joints, and table height. These are demonstration settings, not measured hardware calibration. The default endpoint is `gripper_link`, not a calibrated physical TCP. Five arm joints are controlled; gripper remains separate. Position has priority over orientation.

Collision checking uses a separate Bullet world, so trial configurations cannot reset the live plant. It checks self-collisions except directly connected neighbors and checks against a static table. Midpoint checks include a bound based on joint travel and a configurable reach bound; uncertain intervals are subdivided without reducing the required clearance. Actual clearance violations or exhaustion of the 1,000-check budget reject the motion. Collision-stop diagnostics identify the link pair and distance, or budget exhaustion. This is **not a certified continuous collision detector**. Full model geometry, adjacency exclusions, reach bound, table location, and actual obstacles require site validation. Unknown obstacles and dynamic people/objects are not modeled. Upstream notes some base collision geometry was removed in its model; inspect collision coverage rather than assuming the official model provides complete safety coverage.

## 3. RGB-D worker when the camera arrives

A RealSense source is implemented. Other RGB-D devices need an adapter providing the same registered depth, pixels and capture-time contract.

```bash
python3.12 -m venv .venv-vision
env -u PYTHONPATH .venv-vision/bin/python -m pip install -r requirements-vision.txt
env -u PYTHONPATH .venv-vision/bin/python -m pip check
PYTHONPATH=src .venv-vision/bin/python tools/vision_rgbd.py \
  --detector-repo /home/bx/repos/dex-retargeting \
  --display --record-images
```

The dependency file preserves the previously tested detector versions; the fresh Python 3.12/RealSense combination still needs installation and device verification. Do not modify the conflicting existing `hand` environment or system Python.

In another terminal start the simulator **without** `--synthetic`:

```bash
PYTHONPATH=src .venv-sim/bin/python tools/run_teleop.py --backend sim
```

Processes exchange an atomic latest-only file, `outputs/runtime/vision.json`, using the same host monotonic clock. Missing, malformed, old or future input pauses the controller. `--vision-file` changes that path in both processes. Optional localhost UDP is available with matching `--port` (vision) / `--vision-port` (controller); the file path does not require networking permissions.

The default gripper output directly follows finger opening as the model joint angle, with no per-user calibration. Optional legacy range mapping remains available with both `--closed-rad VALUE --open-rad VALUE`. These angles are the existing thumb-versus-four-fingers aperture metric, **not motor angles**. Do not substitute guessed values for acceptance. All 21 points remain in `outputs/rgbd/frames.jsonl`; optional NPZ files contain unmirrored BGR, aligned metric depth and timestamps. Source frame-number gaps are logged: these are all **acquired** frames, not a promise of lossless camera-rate recording.

The worker aligns depth to color, applies the sensor's depth scale, and uses RealSense deprojection with the aligned profile's distortion/intrinsics. It requires synchronized global/system capture timestamps; dequeue/inference completion time is not substituted. Bad clock domains, host clock jumps or RGB/depth skew stop the producer. Consumer expiry then stops control.

For recorded RealSense bags, pass `--bag PATH`. Replay rebases video time to the local monotonic clock and is simulation-only; it never produces hardware-approved geometry. Slow processing remains stale and pauses rather than pretending to be realtime.

The initial visibility method is explicitly a **palm-hull surface assumption**. Consistent skin depth is not proof that anatomical joints are visible. Surface bias, occlusion, metric translation, rotation-induced drift, timestamps and finger-down performance must be evaluated on the actual camera. `geometry_validated` defaults false. Hardware remains paused until `config/camera-acceptance.local.json` matches the emitted geometry SHA-256 and records measured validation. The hash binds camera serial/intrinsics, depth scale/profile, detector blob, selected hand, aperture mapping and support method. For `wrist_surface_fit` it also binds the estimator source hash/version and semantic reference. Signatures/dates are optional provenance, not startup prerequisites.

## 4. Physical robot path (implemented, commissioning still required)

The controller launches `tools/robot_worker.py` in a **separate SDK environment**, using pipes rather than importing LeRobot into the core. Only that worker opens the serial port. No imports, tests, setup commands, or simulator demos open devices, home joints, calibrate servos or enable torque.

The adapter targets the locally inspected LeRobot `FeetechMotorsBus` API (checkout HEAD `f2c548e24dfec216742c24d5934cac045885a416`, with local source changes possible). The SDK import/signatures were checked without opening a device. The worker uses raw reads/writes with `normalize=False`, never treats LeRobot normalized values as radians, never writes homing offsets and refuses non-position motor modes. It requests a bounded serial write timeout and packet timeout; actual timeout behavior must be verified with the installed SDK and unplugged-bus tests.

Prepare local configuration copies:

```bash
cp config/robot.example.json config/robot.local.json
cp config/runtime.robot.example.json config/runtime.robot.local.json
cp config/camera-acceptance.example.json config/camera-acceptance.local.json
```

Fill them from measurements, not assumptions:

- Each joint uses `raw = zero_raw + raw_per_unit * angle_rad`. A negative scale explicitly represents a reversed sign. Raw limits and physical joint limits must both match this robot and its current firmware offsets. Gripper uses `raw = zero_raw + raw_per_unit * aperture`, where 0 is closed and 1 open. Example IDs also need verification.
- `expected_servo_calibration` pins all six IDs, drive modes, homing offsets and raw ranges. The worker compares the live firmware calibration before its first goal or torque write. A changed homing offset cannot silently reuse an obsolete joint zero.
- `gripper_model_calibration` contains `zero_raw`, `raw_per_rad` and `angle_range_rad`. Its encoder endpoints must exactly agree with the aperture mapping. Rendering and collision checking use those same physical angles. Arm IK uses the intersection of calibrated operating limits and URDF limits.
- `tcp` is a fixed transform from `gripper_link` to the measured TCP: `position_m` and `quaternion_xyzw`. It cannot remain null for hardware. Re-engagement uses measured joints and re-anchors both position and orientation through this FK.
- Measure camera/base mapping, collision scene and workspace. Default simulation settings are not hardware calibration. Stop other robot applications before opening this worker. An advisory lock and kernel serial exclusion block cooperating/new owners; they do not evict an already-open driver.
- For this supervised lab demo, verify the software stop, command expiry, serial timeout, loss/disconnect, controlled hold, gripper-load preservation and full-pose re-engagement. A physical E-stop is not a prerequisite. Record who accepted these and when. Example acceptance flags deliberately remain false; merely changing flags does not perform the tests.

Only after local commissioning, the explicit launch is:

```bash
PYTHONPATH=src .venv-vision/bin/python tools/vision_rgbd.py \
  --detector-repo /home/bx/repos/dex-retargeting \
  --geometry-acceptance config/camera-acceptance.local.json \
  --display

# Separate terminal; starts released, no homing:
PYTHONPATH=src .venv-sim/bin/python tools/run_teleop.py \
  --backend robot --config config/runtime.robot.local.json --execute
```

Gripper control uses direct angular following by default; no gesture endpoint capture is required. Torque is not enabled by default. The worker refuses to proceed if motors are unpowered; `--enable-torque` is a separate explicit startup option after the startup-hold procedure has been physically validated. The ordinary run never recalibrates the arm or commands a home pose.

The driver has a local expiring-command loop independent of camera/controller execution. Commands are sequential, short-lived and close to measured positions. Expiry, malformed commands, buffering/backlog, controller EOF or a stop decision replaces the old **arm** target with current measured arm positions and latches release-required. Gripper goal is retained; torque is not disabled. E-stop requires reset followed by release/re-engage. On exit, it attempts the same hold before closing the serial port without disabling torque.

The installed SDK overwrites packet timeouts during reads. The worker now caps
both actual per-packet timeout setters at 20 ms and uses a monotonic receive
timer; this was tested against the installed SDK in memory. Physical bus timing
still needs measurement. **Limit:** holding measured position is an implemented
stop candidate, not a measured braking guarantee. It cannot protect against
driver SIGKILL, bus failure, a hung SDK call, or loss of power. The user accepts
software stop for this supervised lab demo. Configuration records measurements;
it does not establish untested stopping behavior.

## 5. Verification commands

```bash
PYTHONPATH=src .venv-core/bin/python -m pytest -q
PYTHONPATH=src .venv-core/bin/python examples/offline_demo.py --out outputs/runtime/offline
PYTHONPATH=src .venv-core/bin/python examples/replay.py outputs/runtime/offline/trace.jsonl
PYTHONPATH=src .venv-core/bin/python examples/replay_probe.py \
  outputs/video/right.jsonl --out outputs/runtime/video-safety.jsonl
```

The user recording is RGB-only, so the last command must produce no arm movement. Camera arrival, complete official geometry, and commissioning unlock the remaining integration tests; they do not retroactively make the numerical tests evidence of hardware accuracy.
