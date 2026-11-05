# Runtime implementation validation — 2026-11-05

## Result

Implemented the remaining software paths that can be developed without the missing model/camera: full-URDF PyBullet plant, collision-check world, guarded controller, RealSense worker, latest-file/optional UDP observation transport, calibrated raw Feetech worker, independent command leases and explicit launch/configuration gates.

**Not yet established:** official SO101 simulation performance, real RGB-D capture/accuracy, physically verified stopping, collision completeness, hardware hand control or task success. No serial device was opened and no motor command was sent in this work.

## Tests actually run

- **101 passed, 1 skipped** with Python 3.12.3, NumPy 2.5.4, SciPy 1.18.1, pytest 8.4.2. Full test report: `pytest.xml`.
- The skipped test exercises optional UDP sockets: this sandbox rejects socket creation with `Operation not permitted`. Default atomic-file IPC was tested and passed.
- Three real PyBullet engine tests passed using intentionally artificial tiny tetrahedral meshes and the offline joint transforms: joint feedback/FK agreement, dynamic motor response, table-collision rejection, and a headless synthetic controller subprocess. **Those meshes are a test fixture, not a rendered SO101 or a validated robot collision model.** They are temporary test files, not production assets.
- SDK-independent tests cover calibration signs/units/ranges, duplicate/invalid IDs, future/expired commands, mandatory release after expiry, E-stop reset semantics, collision rejection/re-engagement, and preservation of the gripper on arm stop.
- The real driver worker event loop ran in a separate subprocess with a fake port. It expired an active command independently, issued a stop, rejected renewed active commands without release, and stopped/closed on controller EOF. This verifies software lifecycle, not physical braking or electrical watchdog performance.
- RGB-D tests deproject synthetic registered depth, verify global wrist translation, reject depth holes/misaligned grids, and retain the independent valid aperture channel.
- 1,200 synthetic offline cycles ran; all 1,200 recorded decisions reproduced. See `offline/metrics.json`. This remains ideal-plant numerical replay, not physics or camera accuracy.
- All 326 recorded RGB frames passed through the safety replay, across 4,335 controller ticks. It correctly issued zero motion commands without valid global position. See `video-safety.summary.json`.
- Launch parsers compiled and `--help` worked for simulator, camera and robot workers without opening devices.
- Existing LeRobot environment successfully imported `Motor`, `MotorNormMode`, `FeetechMotorsBus` and inspected raw read signatures without constructing a bus. Its `pip check` passed with inherited ROS PYTHONPATH removed. No motor I/O APIs were executed.

## Environment/provenance

- The workspace `.venv-core` uses isolated copied local NumPy/SciPy and cached pytest dependencies as documented in `../video/VALIDATION.md`; system site-packages remain disabled.
- The PyBullet native extension was copied into that environment from the existing ROS environment, rather than installing into or modifying ROS. Source package metadata reports PyBullet 3.2.5 (Conda build package 3.25), build timestamp October 21, 2025, API version 202010061. No model assets were copied into production from the test fixture.
- LeRobot API source reviewed at `/home/bx/lerobot/src/lerobot`; checkout HEAD is `f2c548e24dfec216742c24d5934cac045885a416`. Local changes may exist, so this is an API compatibility inspection, not a claim of a clean locked release or physically tested driver.
- The unchanged hand detector remains pinned to Git blob `4867ad7d2e5ea3984002c1730b937d06b1fe97ee`.
- `requirements-vision.txt` preserves baseline detector versions; its fresh Python 3.12 + RealSense installation/capture combination has not been executed here.

## Remaining integration work once resources arrive

1. Download the full official model, then run `tools/check_model.py`. Inspect collision coverage (upstream mentions omitted base collisions), mounting/table geometry, link exclusions and reach bound. Test synthetic control in the official model; do not reduce collision checks simply to make it move.
2. Run live RGB-D capture with timestamp checks, inspect full failure recordings and frame-number gaps, and measure depth/pose errors including fingers-down poses. The palm-hull mask is explicitly a surface-support assumption, not an anatomical occlusion detector.
3. Measure joint/raw calibration, rigid TCP and gripper endpoints; verify serial ownership and device failure handling. Robot and camera acceptance templates intentionally contain null/false fields and block physical launch.
4. Physically validate the implemented hold-current-arm-position stop strategy and independent command expiry. No physical deceleration/braking guarantee is made. Driver SIGKILL, bus failure and power loss require independent protection/manual E-stop.
5. Execute the full pick/transport/place acceptance sessions from `docs/05_ACCEPTANCE.md`.

Commands and configuration instructions: `docs/08_RUNTIME.md`.
