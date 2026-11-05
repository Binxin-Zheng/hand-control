# Hand recording integration — 2026-11-05

The project is **not yet complete**: no moving-arm simulation or hardware control has been validated.

Implemented:

- Upstream recorder v2: full-frame logging, video presentation timestamps, selected handedness, exact detector blob and input hash, dependency versions, and unmirrored annotated video. The separate display can be mirrored without changing inference geometry.
- Explicit upstream-to-core adapter: preserves 21 points and original orientation; can fuse supplied, aligned, visible depth points using the existing palm estimator. RGB alone yields unavailable global pose. No predictions or cached positions are promoted to observations. This adapter has numerical tests, not a tested depth-camera driver.
- Recorded-video safety replay with independent 100 Hz controller ticks and explicit video-clock rebasing. It holds clutch throughout and verifies that missing global pose never emits a command.

Actual verification:

- 77 tests passed with Python 3.12.3, NumPy 2.5.4, SciPy 1.18.1, pytest 8.4.2. JUnit: `pytest.xml`.
- 1,200 synthetic ideal-plant cycles and all 1,200 replay decisions reproduced. This is not physics simulation. Results: `../baseline/metrics.json`.
- Full `sample/output.mp4`: 326 frames, 1280×720, 7.5 FPS, 43.466667 seconds. Right-selected detector: 160 frames; Left-selected detector: 1 frame. Both passes retain all 326 frames. Both annotated videos also contain 326 frames.
- Original and updated recorder produced identical selected/not-selected decisions on all 326 frames. Every selected frame retains 21 points.
- Original detector blob matches audited `4867ad7d2e5ea3984002c1730b937d06b1fe97ee`. Default 0.8 detection/tracking thresholds and original preprocessing remain unchanged.
- The video safety replay processed all 326 frames and 4,335 controller ticks: 166 upstream-not-selected frames, 160 global-translation-unavailable frames, zero motion commands.
- Isolated core and existing dex-retargeting vision environments both pass `pip check` with inherited ROS `PYTHONPATH` removed. Exact inventories are adjacent text files.

Observed diagnostic events:

- Right selected from 7.0667 through 29.3333 seconds, with eight missing frames inside that interval: 85, 123, 127, 128, 135, 141, 142, 151.
- Frame 85 (11.3333 seconds) is selected as Left in the independent Left pass and rejected in the Right pass, indicating handedness instability.
- Idle time is retained. These are all-frame counts, **not recall or accuracy**. A contact-sheet inspection confirms idle periods and fingers-down poses; per-frame hand-presence/pose ground truth has not been completed. No pose-specific acceptance rate is claimed.

Environment investigation:

- Default system Python lacked numerical dependencies. System Python 3.12 with SciPy 1.11.4 failed nine baseline tests on read-only rotation arrays; that SciPy is below the project's supported range. No core workaround or threshold change was made.
- Existing `hand` environment has dependency conflicts and was not used. Existing dex-retargeting `.venv` passes dependency checks and ran the unchanged detector.
- `.venv-core` was populated offline from local Python 3.12 NumPy/SciPy distributions, cached pytest/pluggy, and pure-Python system packaging/iniconfig/pygments. System site-packages are disabled. No base environment was changed. Remove inherited ROS `PYTHONPATH` for environment checks; set it explicitly to `src` for core commands.

Remaining blockers:

- GitHub download failed with DNS resolution unavailable in this runtime. The local official follower URDF at `/home/bx/3d_model/calibration/sources/so101_new_calib.urdf` lacks its referenced mesh paths. See `model-check.json`. The kinematics-only extract is not a substitute for rendering or collision geometry.
- The MP4 supplies no calibrated depth or camera intrinsics. Live camera choice and a validated global wrist position source remain necessary. A future monocular model must be explicitly calibrated/evaluated and labeled model-derived; it cannot silently pass the current depth-evidence policy.
- Real TCP, joint calibration, collision checks, independent expiring-command watchdog and physical controlled-stop/E-stop verification remain required before hardware execution.

Reproduce from the repository root:

```bash
env -u PYTHONPATH .venv-core/bin/python -m pip check
PYTHONPATH=src .venv-core/bin/python -m pytest -q
PYTHONPATH=src .venv-core/bin/python examples/offline_demo.py --out outputs/baseline
PYTHONPATH=src .venv-core/bin/python examples/replay.py outputs/baseline/trace.jsonl
MPLCONFIGDIR=/tmp/hand-mpl /home/bx/repos/dex-retargeting/.venv/bin/python examples/upstream_probe.py --repo /home/bx/repos/dex-retargeting --camera sample/output.mp4 --hand Right --out outputs/video/right.jsonl --annotated outputs/video/right.mp4
PYTHONPATH=src .venv-core/bin/python examples/replay_probe.py outputs/video/right.jsonl --out outputs/video/safety-replay.jsonl
```

Repeat the recorder command with `--hand Left` and distinct output filenames for the second diagnostic pass. These commands never connect to motors.
