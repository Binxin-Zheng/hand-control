# First D455 recognition and simulation test

## Automatic visual preview

Run `.venv-core/bin/python tools/hand_follow.py --record-images` from this
repository. The launcher opens the camera preview and official-model simulator,
uses distinct existing vision/core environments, and creates a unique session
under `outputs/hand-follow/`. There is no physical-robot option in this launcher.
The visual simulator starts following usable tracking automatically. **Space**
pauses/resumes; **Q** exits. Missing tracking freezes the arm. When tracking
returns, it releases and re-anchors from the measured simulated pose before
following again, so it does not chase an old target. Collision, backlog, timing
faults and E-stop remain visible and require deliberate action.

This user-requested automatic recovery is a visualization behavior. For the
explicit control workflow used to validate physical-arm recovery, add
`--manual-control`: E engages, R releases, X stops, C resets while released and
Q exits. That mode starts released and requires release/re-engage after loss.
The physical controller cannot enable automatic preview.

The explicit selections are `--position-method wrist_surface_fit`, depth
640×360, unchanged color/inference 640×480, and `--control-mode position`.
Position mode uses the measured engagement orientation as a secondary IK
target; exact orientation holding is not promised for five joints. Movement
gain remains 0.5. Up maps to robot up, toward the camera to robot forward, and
mirror-right to robot right in the existing first-person convention.

For live evaluation retain the usual fingers-down poses. Include static holds,
left/right, up/down, toward/away, rotations without intentional translation,
and opening the fingers. Preserve every failed frame and pause. Record-image
compression adds acquisition load; source-frame gaps and capture age remain
visible. Automatic recovery must not be counted as physical recovery validation.
In manual mode, after a pause use R then E. A bounded startup check is
`.venv-core/bin/python tools/hand_follow.py --headless --duration 8`;
headless still opens the D455 and uses the selected automatic/manual behavior.

The new estimator uses only currently observed depth near the wrist image ray,
with explicit local support, foreground-consistency and surface-fit rejection.
It does not use model wrist-relative XYZ as global position. A surface reference
can still move with skin/landmark changes, so availability is not accuracy.
Original `model_translation` and `surface_wrist` paths remain selectable for
complete-recording comparison. The low-level camera worker default is unchanged.

For depth-hole diagnosis, optional `--record-stereo` also records **native Z16
depth and both infrared images** before color registration, together with their
actual frame timestamps/exposure metadata, intrinsics and transforms to color.
This implies raw RGB/aligned-depth recording. Each NPZ has `native_depth_z16`,
`infrared_1`, `infrared_2` and a Unicode `stereo_metadata_json` field. Native depth
is explicitly raw units with its recorded meters-per-unit scale. Missing streams
and unsupported metadata remain marked unavailable; no images are fabricated.
This diagnostic option adds streams and recording load, and is not the default
preview. The additional hardware stream combination still requires a bounded
device check before relying on a diagnostic recording. Do not launch another
camera worker while the existing preview owns the D455.

For the next near-range sensor comparison, `--depth-size 480x270` and
`--depth-size 424x240` are explicit alternatives; the default stays 640×360.
The D455 datasheet lists nominal minimum depths of 260, 200 and 180 mm for
640×360, 480×270 and 424×240 respectively, and supports their depth/IR streams
at 30 FPS over USB 3.1. Lower spatial resolution is a tradeoff, and these
minimum distances do not guarantee valid hand depth. Color inference stays
640×480 with the original upstream processing; the SDK supplies the selected
profile's actual calibration and registration. Source: [D400 datasheet,
tables 4-3 and 4-11](https://realsenseai.com/wp-content/uploads/2025/09/Intel-RealSense-D400-Series-Datasheet-October-2025.pdf).
The two added profiles have not yet been streamed on this device. A bounded
`--depth-size 424x240 --record-stereo --duration 15` camera/simulator test is
prepared for reconnection; any unsupported combination must fail explicitly.

To compare a complete raw recording without camera or motors:

```bash
PYTHONPATH=src .venv-vision/bin/python tools/analyze_rgbd_recording.py \
  outputs/depth-prep-01 --methods model_translation surface_wrist wrist_surface_fit \
  --out outputs/position-research/new-comparison.jsonl
```

Choose a new output filename; existing artifacts are not overwritten. The
summary includes all-frame and detected-frame denominators, timing, unavailable
spans, input hashes and potential jump-guard violations. Adjacent position steps
include real hand motion and must not be described as measured jitter.

Physical-arm launch still uses the separate explicit hardware entry point after
current pose/scene and integrated stopping checks. A reused calibration is now
derived from the older documented physical alignment and matched to this URDF;
firmware offsets/ranges are pinned and rechecked before writes. Shared physical
gripper angles drive both the encoder mapping and collision geometry. The
2026-11-06 read-only check found all torque off and matching calibration, but
elbow raw 3038 exceeds its recorded maximum 3037; this was not clamped or hidden.

The worker reads camera intrinsics and depth units from the D455; no hand-entered camera matrix is needed for the first diagnostic run. It aligns depth to color using the SDK, as in the [official RealSense alignment example](https://raw.githubusercontent.com/realsenseai/librealsense/master/wrappers/python/examples/align-depth2color.py). Keep the camera stationary during control.

Run from a normal desktop terminal with USB access. The restricted sandbox cannot initialize the RealSense udev monitor; an explicitly approved host diagnostic can access the camera. As of 2026-11-06, `assets/so101` and `.venv-vision` are installed, the model check passes, and a bounded live D455 test produced hand detections and fused wrist coordinates, with substantial depth rejection. See [readiness evidence](../outputs/readiness/VALIDATION.md). No physical arm was operated.

## Prepare the vision environment once

```bash
cd /home/bx/repos/hand-control
python3.12 -m venv .venv-vision
env -u PYTHONPATH .venv-vision/bin/python -m pip install -r requirements-vision.txt
env -u PYTHONPATH .venv-vision/bin/python -m pip check
env -u PYTHONPATH .venv-vision/bin/python -c 'import pyrealsense2 as rs; print([(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number)) for d in rs.context().query_devices()])'
```

The last command should list the D455. An empty list means USB access/device discovery must be fixed before the worker can run. Close RealSense Viewer or another camera application before streaming here. The installation needs network access, which is unavailable in the agent sandbox. If installation or `pip check` fails, preserve the error and resolve it before launching; do not force incompatible packages.

## Recognition and metric depth

```bash
PYTHONPATH=src .venv-vision/bin/python tools/vision_rgbd.py \
  --detector-repo /home/bx/repos/dex-retargeting \
  --hand Right --display --out outputs/d455-first
```

Use `--hand Left` for your left hand. With more than one RealSense device, select the desired one using `--serial SERIAL_NUMBER`.

The preview shows all 21 landmarks, aligned depth, the selected-hand detection state, global wrist XYZ in meters, valid palm-depth count, fusion residual, capture age, and finger opening angle. The depth color scale is fixed at 0–1.5 m for display only; it does not clip the data used by fusion. Status also prints once per second in the terminal. XYZ remains unavailable when depth fusion fails; the UI does not show a fabricated or stale position.

Camera coordinates: X right, Y down, Z away from the camera. Move in each direction, rotate your wrist, open/close your thumb, use your comfortable fingers-down pose, then remove/reintroduce your hand. Check that depth/pose availability is distinct from merely detecting landmarks. Observed depth fusion is still an estimate affected by surface/model bias, not demonstrated millimeter accuracy.

The gripper now follows the model-based finger opening angle directly, constrained by the model joint limits. No gesture calibration is required. For an automatic color-only preview that does not depend on wrist XYZ, run `PYTHONPATH=src .venv-core/bin/python tools/gripper_test.py`; see [automatic gripper following](11_GRIPPER_TEST.md).

Press Q or Escape in the camera window to stop. All processed-frame observations, including no-hand results, are logged at `outputs/d455-first/frames.jsonl`. Add `--record-images` to save color/depth arrays as NPZ for diagnosis; compression can reduce acquisition throughput, and source frame-number gaps remain logged. Initial clock synchronization rejections are logged without publishing them; a clock fault after successful synchronization stops the stream.

## Hand-controlled simulation

Download and check the model first, using a terminal with network access:

```bash
python3 tools/fetch_so101_model.py --out assets/so101
PYTHONPATH=src .venv-core/bin/python tools/check_model.py
```

Leave the camera worker running. Open a second terminal:

```bash
cd /home/bx/repos/hand-control
PYTHONPATH=src .venv-core/bin/python tools/run_teleop.py --backend sim
```

The existing `.venv-core` has the locally tested PyBullet extension. For a new machine/environment, follow the `.venv-sim` installation in `docs/08_RUNTIME.md`.

Do **not** add `--synthetic` for hand control: that option replaces camera input with a numerical test trajectory. Both processes use `outputs/runtime/vision.json` by default, independent of the diagnostic output directory.

Hold a steady hand with valid XYZ, then press **E** in the simulator window to engage. **R** releases, **X** latches software E-stop, **C** resets while released, and **Q** exits. Equivalent terminal commands are `engage`, `release`, `estop`, `reset`, `quit` followed by Enter. After losing tracking, press R then E; reappearance alone does not resume movement. The simulator's default scale is 0.5 and orientation is best-effort because the arm has five joints.

If `check_model.py` reports an initial collision, or the controller pauses, preserve the exact reason and logs. Do not disable collisions or increase tracking thresholds simply to make it move. The official-model synthetic runtime passed a 12-second run after adaptive collision checking was added; sustained live-hand simulation and physical commissioning remain outstanding.

## Physical robot

Use simulation for the first camera test. Physical-arm joint calibration, measured TCP, collision scene and local hold/watchdog/E-stop acceptance remain necessary. The example configurations intentionally do not authorize motor execution. Follow `docs/08_RUNTIME.md` for commissioning; do not turn acceptance flags on merely to bypass startup checks.
