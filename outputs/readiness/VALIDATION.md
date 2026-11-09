# Readiness check — 2026-11-06

## Final offline comparison and next live action

The complete unchanged-depth Holistic comparison reproduces the original
readable baseline exactly: 1,906 depth-supported positions, zero coordinate or
availability differences. Holistic yields 1,920 (28 gains, 14 losses), and the
baseline-first hybrid yields 1,929 (23 gains, no baseline losses by construction).
All 2,857 attempts are retained, including seven capture failures and the one
terminal corrupt archive. The palm-down interval has 34 Holistic hand selections
but only **four** depth-supported positions, in runs of 1, 1 and 2 frames. The
hybrid's longest unavailable span remains unchanged, and its frame 2554→2555
source transition creates a 156.33 mm step exceeding the unchanged motion guard.
No runtime detector promotion is justified by these counts.

The worst 179.1 mm disagreement between two valid depth-supported estimates
occurs on the same visible right limb: the original wrist landmark is on the
proximal forearm, whereas the body-guided point is near the wrist. Both local
surface fits are supported by current depth, so depth residuals alone cannot
detect this semantic reference error. This is not anatomical ground truth or a
179 mm measured accuracy claim. The annotated evidence is
`outputs/position-research/live-01-holistic.reference-difference.jpg`. Full
reasoning and retained inputs are in `live-01-holistic.report.md` and
`live-01-hybrid.report.md`; the original detector remains unchanged in runtime.

The explicit launcher/worker now also accept 480×270 and 424×240 native depth
profiles for the next sensor comparison. Defaults remain unchanged, and color
inference stays 640×480. The official D455 table lists nominal minimum depths of
200 and 180 mm for these profiles; this is not a guarantee of usable skin depth.
No stream at either added profile has been started on this device. The prepared
bounded native-depth/IR diagnostic still needs the disconnected D455 to return.
See `docs/09_D455_QUICKSTART.md` for the source and exact options.

Final combined verification after the offline research adapters and profile
option were added: **353 passed, 3 skipped in 10.56 s**, JUnit
`outputs/position-research/pytest-research-commissioning-final.xml`. Skips are
the same two isolated SDK checks and sandbox loopback restriction. The complete
1,200-decision offline control replay reproduced exactly again, and
`git diff --check` passed. This establishes software checks only. Live camera
six-direction following and physical hold/movement checks remain unverified;
no physical motor was operated.

## Bounded hold commissioning and body-association diagnosis

Read-only host RealSense discovery after the disconnect returned an empty
device list (`[]`). No camera stream was started. Live following still requires
the D455 to be connected and an operator available for the direction sequence.

The acceptance dependency in the physical launcher is now addressed by a
separate, explicit hold-only commissioning mode. `tools/commission_hold.py`
supervises the same `tools/robot_worker.py` loop with `--commission-hold`, explicit
execution/current-pose confirmation and an independent worker duration of at
most 30 seconds. The ordinary physical launcher still requires every existing
acceptance field; no acceptance configuration was changed.

Only the exact five raw arm positions from the startup hold write may be sent
as active goals. The gripper is excluded from subsequent active writes and its
existing loaded goal is preserved. Expiry, stop, release after activation,
malformed input or another error permanently ends active eligibility in that
invocation. A new invocation obtains a new measured startup anchor. Complete,
in-range integer feedback, firmware calibration and position mode are checked;
commissioning also bounds completed-read duration and feedback age before a
write. The helper stops renewing commands on silent feedback. It records
cleanup failures rather than treating an observed stop message as overall
success. The controller-exit exercise requires the worker's actual terminal
`controller_eof` reason and normal exit, with remaining output drained.

The four explicit exercises are command expiry, software-stop command, release
and controller EOF. The operator can also request X/Enter software stop. All
four helper-to-real-worker-loop exercises passed with a fake port. This mode
does not admit a hand target, operate a trajectory, authorize normal
teleoperation, prove braking during motion or verify an actual table scene.
Physical use still needs current supported/clear pose confirmation and
in-range measured joints; the saved out-of-range elbow snapshot remains
unresolved. See the concrete command in `docs/10_DEMO_COMPLETION.md`.

Commissioning checkpoint suite:
`PYTHONPATH=src .venv-core/bin/python -m pytest -q -rs
--junitxml=outputs/position-research/pytest-hold-commissioning.xml`:
**332 passed, 3 skipped in 10.57 s**. Two skips are SDK checks in the intentionally
isolated core environment; the other is sandbox loopback access. Separate
no-I/O checks in the installed robot SDK confirmed integer raw encoder decoding
and the wrapped 20 ms packet timer, without importing pytest or opening serial.
`examples/replay.py outputs/position-research/offline-automatic-final/trace.jsonl`
reproduced all **1,200 decisions**. `git diff --check` passed. Independent source
review and tests addressed stale feedback, terminal-state re-engagement,
gripper preservation, cleanup failure and EOF evidence attribution. No motor
or serial device was accessed.

An additional six-direction isolation experiment used synthetic fresh hand
observations, the unchanged mapping/gain and position servo, official collision
guard, and the existing dynamic Bullet plant. Each 40 mm hand ramp produced
the expected 20 mm robot target. All six signs were correct with no paused
decisions; each trial retained its initial release, zero-jump engagement and
200 tracking decisions. A seventh stationary trial was retained too. Dynamic
response reached 90% of the target after 1.86–2.22 seconds, illustrating lag
independent of vision. This uses virtual control time and uncalibrated simulated
motors; it does not establish real-time deadlines, camera directions, physical
joint signs or hardware precision. Reproduction source, all 1,582 decisions and
metrics are `outputs/position-research/check_simulated_directions.py`,
`simulated-six-directions.jsonl` and `simulated-six-directions.summary.json`.

The complete offline Pose association analysis retained all 2,857 recording
attempts and made 2,849 readable-frame calls. Body pose was present on 2,843
frames. In the 34-frame target-hand label-flip block, every raw target wrist
was close to the body-right wrist (2.95–13.76 px, median 5.12 px; visibility
0.983–0.991). In the 156-frame distractor block, all 153 body-available raw
candidates were instead closer to the body-left wrist; three had no body pose.
The opposite body wrist lies outside the image throughout the label-flip block,
so it is not a trusted negative reference there. The positive right-wrist
association is the relevant evidence. The 195-frame raw no-hand outage still
contains no usable hand candidates. No association threshold was tuned, no
hand gate was relaxed and no recovered global XYZ is claimed. Report and
overlays: `outputs/position-research/live-01-pose-association.report.md` and
`live-01-pose-association.examples.jpg`. Runtime detector behavior is unchanged;
body-guided hand inference remains an offline experiment.

## Latest verification and remaining live work

The automatic session `20261106-102058-056c774b` has **ended**, with launcher
exit code 1 after the D455 reported `Device disconnected. Failed to reconnect`.
The terminal process result confirms both children were closed; the camera was
not automatically restarted. The original acquisition error was followed by a
cleanup error (`stop() cannot be called before start()`). Cleanup now attempts
all resource closures and preserves the acquisition error as the primary cause;
two regressions cover this behavior. The cause of the USB/device disconnect is
not established.

The last session exposed two separate controller interruptions. Sequence 5124
crossed the unchanged 60 mm measured-pose target-backlog limit (63.23 mm).
Sequence 5656 spent 46.14 ms in collision checking, producing a control overrun.
The simulated motor response also introduces substantial uncalibrated lag; its
dynamics have not been replaced with ideal execution. Automatic tracking
recovery does not hide these controller faults.

Collision checking now uses conservative per-pair rigid-tree motion bounds.
Shared ancestor transforms cancel from self-separation; remaining motion uses
loaded-shape enclosing radii and every intervening URDF origin. Unsupported
models retain the original bound. The official geometry, 5 mm clearance,
0.01 rad initial intervals and 1,000-sample rejection budget remain unchanged.
Seven-repeat, sequential offline benchmarks measured saved segment 1790 at
22.13 → 1.44 ms (180 → 12 samples), and reconstructed discarded segment 5656 at
46.09 → 1.46 ms (372 → 12 samples). Reconstruction provenance and exact hashes
are in `outputs/position-research/collision-pairwise-bound-validation.json`.
An independent source review found no actionable issue. These results prove
neither live deadline reliability nor physical actuator stopping.

The unchanged upstream detector was rerun on all 2,849 readable RGB frames of
the first recording: every saved selected probe, including all 21 keypoints,
reproduced exactly. Of 465 nonselections, 269 had no raw hand and 196 were
filtered by handedness. The latter include both a distractor hand occupying the
single-hand tracker and a separate target-hand label-flip interval; they are
not 196 proven classification errors. See
`outputs/position-research/live-01-detector-reprobe.report.md`. Detector
experiments remain offline; no runtime threshold or handedness rule was relaxed.

Optional `--record-stereo` now prepares native Z16 depth, both infrared streams,
actual capture metadata and source-to-color geometry for depth-hole diagnosis.
It is off by default, does not alter the estimator and has only fake-stream
tests so far. The additional hardware stream combination still needs a bounded
device check. Existing aligned-depth recordings cannot by themselves separate
stereo failure from registration/occlusion holes.

Latest complete suite:
`PYTHONPATH=src .venv-core/bin/python -m pytest -q -rs
--junitxml=outputs/position-research/pytest-automatic-preview-final.xml`:
**271 passed, 2 skipped in 7.71 s**. Skips: robot SDK absent from the intentionally
isolated core environment, and sandbox loopback sockets unavailable. The
collision change also passed 56 targeted tests and retained byte-identical
output for the complete 895-input, 44.28-second recorded replay. Fresh
`examples/offline_demo.py --out outputs/position-research/offline-automatic-final`
and `examples/replay.py outputs/position-research/offline-automatic-final/trace.jsonl`
reproduced all 1,200 synthetic decisions. `git diff --check` passed. None of these
offline checks establishes perception or hardware accuracy. Live six-direction
following and physical-arm acceptance remain incomplete. No motor was accessed.

## User-requested automatic visual simulator

The user explicitly chose automatic **visual simulation**: follow usable
tracking, freeze on missing tracking and resume when it returns. The default
`tools/hand_follow.py` now selects `--automatic-preview`; `--manual-control`
retains the earlier explicit E/R workflow. Main preview controls are Space
pause/resume and Q quit. No per-user gesture calibration is added.

The preview policy wraps the unchanged `TeleopSession`. Every acquisition or
recovery passes through a real release tick, then re-anchors position and
orientation from measured simulated state. No stale target is replayed and
missing/predicted input is never counted as observed. Missing/stale observations
and hand identity loss can recover automatically; malformed/order/future-clock
input, collision, backlog, timing faults and E-stop require deliberate action.
Typed receiver diagnostics preserve the prior error text and timing thresholds.

Automatic preview rejects a physical backend, non-position mode and UDP input
before reading configuration or constructing a world/driver. Its supported
latest-file IPC is the existing launcher path. Manual/physical behavior is
unchanged. AGENTS rule 18 records the user's specific visual-only exception.

Validation: **236 tests passed, 2 skipped** in the full suite; skips remain the
isolated robot SDK and sandbox loopback. JUnit:
`outputs/position-research/pytest-automatic-preview.xml`. A complete 895-input,
44.28-second ideal-plant recorded replay had 21 engagements, each immediately
preceded by an actual release tick and with `q_command == measured_q`; 20 tracking
losses recovered automatically, while a later target-backlog fault stayed
latched. Evidence: `recorded-automatic-preview-control-v2.jsonl` and companion
summary under `outputs/position-research/`. This is visual-policy verification,
not physical recovery acceptance or a live camera accuracy result.

The automatic camera/simulator session was explicitly opened at
`outputs/hand-follow/20261106-102058-056c774b` without a duration limit, as the
user requested continuous operation, and without raw-image compression. Startup
showed WAITING_FOR_HAND and actual camera observations; no initial selected hand
was observed. It subsequently ended on the device disconnect described above.
Its observations/timings are logged; raw RGB/depth archives are
not recorded in this session. No physical motor was accessed. The broader
required physical-following goal remains incomplete.

## Live following tests and timing fixes — latest update

**Basic physical following remains incomplete.** The user reported intermittent
immediate pauses after engagement, one brief forward movement, and inability to
test the other directions. The first two guided D455/simulator sessions did not
meet the required following behavior. No physical motor was accessed in these
sessions. The user also requested discussion of simpler, continuous simulator
interaction; that UI choice must remain distinct from physical recovery policy.

All live attempts are retained under `outputs/hand-follow/`:

- `20261106-093702-636085eb`: 2,857 camera attempts, 2,385 selected detections,
  1,907 online observed positions; 7,401 control cycles, 11 engagements and 450
  tracking decisions. Eleven unavailable-tracking stops included nine missing
  receiver observations and two unavailable poses. Timing faults also occurred.
  Longest ACTIVE interval was 4.584 s. One final NPZ was interrupted during
  compression; its original file/observation remain preserved and the A/B replay
  explicitly counts it as unreadable rather than dropping it. Consequently the
  independently replayed new-method count is 1,906, not 1,907.
- `20261106-094404-3ee8377c`: after earlier publication and added profiling,
  1,578 camera attempts, 1,113 detections, 1,021 observed positions; 4,401 control
  cycles. One engagement lasted 3.969 s before a control overrun. Profiling
  identified 38.685 ms collision work in that fault cycle; a preceding cycle's
  31.973 ms collision work plus 13.117 ms display work lengthened its successor.
  The subsequent invalid-dt event is the consequence of the same stall.
- `20261106-095730-fd89a10d`: after the collision/UI optimization, 1,637 attempts,
  1,618 detections, 1,441 observed positions; 4,419 control cycles. One explicitly
  agent-initiated simulation engagement lasted 2.500 s before unavailable depth
  caused a pause. No timing fault occurred (max dt 22.790 ms), but this was not a
  deliberate six-direction acceptance sequence and did not exercise the previous
  expensive jaw motion. It cannot establish reliable following or comparative
  live performance. No automatic re-engagement was used.

Concrete fixes and verification:

- Fresh observations are now published before synchronous diagnostic logging and
  image compression. A 25-repeat saved-frame microbenchmark measured compressed
  NPZ median 24.922 ms/P95 29.681 ms; that work previously delayed delivery.
  Capture timestamps and the 150 ms freshness limit remain unchanged. Separate
  camera `timing.jsonl` records acquisition/inference/fusion/publication/recording
  costs. The controller logs exact receiver rejection reasons and phase timings.
- Normal SIGINT/SIGTERM finishes the current camera frame before cleanup, and
  archives install atomically. Both later runs exited with both child return
  codes zero; their final archives decode completely. No worker remained running.
- Collision checking caches immutable pair/name data, rejects distant pairs with
  conservative AABB lower bounds, and uses direct distance queries without a
  redundant global contact-manifold rebuild. Invalid/nonfinite/inverted bounds
  raise instead of silently excluding a pair. Clearance, reach bound, adjacency
  exclusions, subdivision budget and stop behavior are unchanged. Display text
  updates at 10 Hz, immediately on state/reason changes; keyboard polling is
  unchanged.
- The exact saved 180-subdivision segment improved from median 32.22 ms to
  22.89 ms in five-repeat offline measurements, retaining all 180 samples and
  identical margins/results. Independent review checked immediate post-reset
  bounds on 128 official-model poses and 2,688 near-margin pair comparisons,
  finding no stale bounds or unsafe exclusions. Full 895-input recorded control
  replay remained byte-for-byte identical to its pre-optimization counterpart.
- Final full suite: `PYTHONPATH=src .venv-core/bin/python -m pytest -q -rs
  --junitxml=outputs/position-research/pytest-live-fixes.xml`: **208 passed,
  2 skipped** (robot SDK isolated from core; sandbox loopback unavailable).
  Fresh synthetic offline generation and replay reproduced all **1,200
  decisions**. These checks do not establish physical accuracy or stopping.

The complete first live recording's position A/B is in
`outputs/position-research/live-01-ab.report.md`: model translation 1,408/2,857,
earlier surface wrist 1,545/2,857, new fit 1,906/2,857 independently replayed.
Actual missing depth and visible-hand upstream non-selection remain, including
natural fully framed poses. A/B fuse-only observations do not include the
worker's subsequent direct-joint-angle gripper mapping: control/collision replay
must use the saved online gripper observations or an explicit matching adapter.
The earlier held-aperture replay did not reproduce this live jaw-motion workload.

Read-only, non-streaming D455 option inspection is saved in
`outputs/position-research/d455-current-settings.json` and its advanced-settings
companion. Emitter was enabled at 150 mW, automatic exposure enabled and disparity
shift zero. No setting or firmware was changed. Non-streaming exposure readback
does not establish actual exposure in the recordings. Native-depth/IR evidence
is still needed to distinguish stereo failure from color-depth alignment holes.

Machine-readable control evidence: `live-follow-01-02-summary.json`,
`live-follow-03-summary.json`, `live-01-recording-cost.json` and
`collision-performance-retest-02.json` under `outputs/position-research/`.

## Global hand XYZ research and implementation — current update

**The user's required physical hand-following outcome is still unverified.**
Implemented the selected current-depth wrist-surface estimator, explicit XYZ
control mode, isolated D455/simulation launcher, complete-recording analysis and
recorded-observation control replay. The original detector, all 21 points,
preprocessing, old estimator defaults and full-pose default are preserved.

Research inspected AnyTeleop, OAT reprojection, DexPilot/DART, palm-surface
fitting literature and actual pinned DepthAI/egoBench code. RGB-only translation
and full PnP comparisons on all 802 detected frames had large image reprojection
errors; they were not adopted as depth fallbacks. Whole-palm plane prototypes
also did not improve availability. The selected local wrist surface is explicitly
not an anatomical joint. Independent adversarial review caught background rings
and isolated-pixel support; those cases now reject. No historical depth is
promoted to current measurement. Details: `docs/01_RESEARCH.md`,
`docs/03_ALGORITHMS.md`, and `outputs/position-research/`.

Final complete-recording A/B, `depth-prep-01-ab-v4.summary.json`:

| Estimator | Available / all 900 attempts | Available / 802 detections |
|---|---:|---:|
| Original model translation | 78 / 900 | 78 / 802 |
| Earlier surface wrist | 616 / 900 | 616 / 802 |
| Current wrist surface fit, version 3 | 683 / 900 | 683 / 802 |

All 895 decoded rows produce byte-equivalent original-method observations and
diagnostics versus the earlier comparison; all five initial untimed capture
errors remain counted. New fitting P95 is 2.403 ms, excluding detector/capture/
logging. Longest available run is 201 frames / 9.7703 seconds. This is availability
and compute timing, **not recall, static jitter, metric accuracy or reliable live
following**. The original recording still has depth holes and pose-dependent
failures. Forty of 657 adjacent valid-position pairs trigger the original model
orientation jump criterion, motivating named XYZ mode rather than changing that
criterion in full-pose mode.

Exact verification performed on the final implementation:

- `PYTHONPATH=src .venv-core/bin/python -m pytest -q -rs --junitxml=outputs/position-research/pytest-final.xml`:
  **193 passed, 2 skipped**. Skips: isolated robot SDK absent from core, and
  sandbox loopback unavailable. Tests include six-direction geometry, fixed-point
  surface rotation, missing/mixed depth, wrong-background rings, isolated pixels,
  association masks, mode separation, stop/re-engagement, physical range/jaw
  consistency, SDK-timeout mocks and launcher lifecycle.
- Installed scservo/LeRobot `syncReadTx` additionally exercised in the separate
  robot SDK environment with mocked transport, never opening a port. SDK timer
  overwrite was 50.51 ms before the fix, bounded to 20 ms after; expiry checked
  at simulated 21 ms. Result: `actual-sdk-timeout.json`. Robot environment lacks
  pytest; the same assertions were executed directly without installing packages.
- `tools/check_model.py`: complete official meshes, Bullet import, named joints
  and FK agree; configured initial simulation pose is collision-clear.
- `examples/offline_demo.py --out outputs/position-research/offline`, then
  `examples/replay.py outputs/position-research/offline/trace.jsonl`:
  **all 1,200 decisions reproduced**. This remains a synthetic ideal plant.
- Eight-second full-model synthetic position-mode simulation:
  **400/400 cycles ACTIVE**, one engagement and 399 tracking decisions;
  maximum observed cycle dt 23.80 ms. `synthetic-position-sim.jsonl`.
- `tools/replay_hand_control.py` consumed **895/895 timestamped observations**
  across the full **44.2848 seconds**, counting five untimed startup errors
  separately. It used the official collision guard, actual supervisor/IK and an
  ideal kinematic plant. Explicit release/engage events at fixed times are saved
  in `replay-events.json`; no loss-triggered auto-restart was introduced. With the
  same events, position mode logged 914 tracking cycles and full-pose mode 516.
  Position mode still paused three times on missing tracking and once on target
  backlog. These replay cycles are not independent camera frames or hardware
  success rates. Final traces: `recorded-position-control-v4.jsonl` and
  `recorded-fullpose-control-v4.jsonl`. The complete 44.28-second side-by-side
  `recorded-position-replay.mp4` retains every interval, displays saved measured
  joints/aperture, and labels itself recorded/ideal-plant/not live robot. All 886
  encoded frames decoded successfully; source hashes and display-to-source
  timing are in its provenance JSON.
- Bounded D455 startup at 640×360 succeeded. The integrated eight-second
  camera/simulator launcher recorded 161 attempts (seven initial clock rejects),
  **zero selected hands**, and 346 released simulator cycles. Therefore this
  validates startup/IPC/cleanup only. Both child processes terminated; graceful
  KeyboardInterrupt handling was subsequently added to avoid shutdown tracebacks.
- A subsequent bounded GUI startup opened the NVIDIA GL simulator and camera
  preview and remained released. No selected hand was observed. The controller
  exited normally; the camera child needed the launcher's bounded SIGTERM
  fallback during cleanup. Both were reaped; no process was left running. This
  is GUI startup evidence, not a visual layout audit or live hand-following test.
- `git diff --check` passed. No dependency or system environment was changed.

Robot preparation and remaining live work:

- Reused the older documented physical alignment, explicitly transformed into
  the official model and current homing convention. Offline model difference is
  at most 0.5375 mm over the sampled configurations; this is **model comparison**,
  not measured robot accuracy. Local mapping/provenance contain the derivation.
- Driver validates pinned live firmware calibration before goal/torque writes,
  bounds actual SDK packet timers, intersects physical/URDF arm limits, and uses
  one physical jaw calibration for encoder mapping/rendering/collision geometry.
- Current **read-only** live inspection matched every designated offset/range,
  found all motors in position mode and torque off. Elbow raw **3038** still
  exceeds its recorded maximum **3037**; no bound was widened and no motor command
  or torque write was issued. Evidence: `robot-inspection-follow-01.json`.
- A guided live hand recording in the nearer-range profile, current arm pose/
  scene check, an in-range starting pose, and integrated hold/stop/expiry/direction
  tests remain. The source-derived mapping reduces repeat calibration work; it
  does not prove these live checks. The active goal remains incomplete.

The remaining sections below are historical updates, superseded where the
current update gives newer implementation or verification results.

Status: the reproduced full-model simulation blocker is fixed. Physical teleoperation is **not commissioned**. No serial device was opened, no physical motor was commanded, and no acceptance flags were changed.

## Fixed: conservative collision-envelope rejection

The original segment checker rejected the first synthetic tracking command after engagement. Gripper travel inflated the required search envelope from 5 mm to approximately 13 mm, while the wrist/moving-jaw separation was approximately 6.9 mm. Envelope overlap was treated as a clearance violation even though it did not establish one.

The checker now evaluates interval midpoints and subdivides ambiguous intervals. The required 5 mm clearance, configured reach bound, official collision meshes, adjacency exclusions, detector thresholds, and tracking thresholds are unchanged. The envelope includes the relative movement bound for two links. Actual clearance violations and exhaustion of a 1,000-check budget fail closed. Collision-stop logs include the closest link pair and distance or the budget failure reason. This remains a conditional geometric bound requiring site validation, not certified physical collision protection.

## Verification performed

- `PYTHONPATH=src .venv-core/bin/python -m pytest -q -rs --junitxml=outputs/readiness/pytest.xml`: 106 passed, 1 skipped (sandbox loopback sockets unavailable). Added regressions for the official-model gripper segment, collision between otherwise-clear samples, and refinement-budget failure.
- `PYTHONPATH=src .venv-core/bin/python tools/check_model.py`: complete meshes, Bullet import, joint mapping, and FK agree; initial pose collision-clear.
- Full official-model synthetic runtime, headless, 12 seconds, explicit simulated engagement: 600/600 cycles ACTIVE, one engagement and 599 tracking decisions. Maximum observed cycle dt approximately 20.36 ms. Trace: `synthetic-sim.jsonl`. This is simulation, not measured robot timing or dynamics.
- `PYTHONPATH=src .venv-core/bin/python examples/offline_demo.py --out outputs/readiness/offline`, followed by `examples/replay.py outputs/readiness/offline/trace.jsonl`: all 1,200 decisions reproduced. Synthetic ideal-plant results do not establish camera or hardware accuracy.
- Both core and vision `pip check` passed with inherited `PYTHONPATH` removed. Plain pytest with the inherited ROS path failed during unrelated ROS plugin import; using `PYTHONPATH=src` isolates the project successfully.

To reproduce the synthetic runtime without any robot access:

```bash
PYTHONPATH=src .venv-core/bin/python tools/run_teleop.py \
  --backend sim --synthetic --headless --duration 12 \
  --log /tmp/hand-control-simulation.jsonl <<'EOF'
engage
EOF
```

## Live camera evidence

Read-only discovery in the restricted sandbox failed with `could not initialize udev monitor`. An approved host-side discovery found the D455, and an explicitly bounded camera-only run completed. Its IPC output was isolated at `/tmp/hand-control-readiness/camera.json`; no controller consumed it.

```bash
PYTHONPATH=src .venv-vision/bin/python tools/vision_rgbd.py \
  --detector-repo /home/bx/repos/dex-retargeting --hand Right --frames 300 \
  --out outputs/readiness/camera \
  --vision-file /tmp/hand-control-readiness/camera.json
```

All 300 attempts are retained in `camera/frames.jsonl`:

| Outcome | Count |
|---|---:|
| Initial RGB/depth clock synchronization rejection | 5 |
| Remaining processed frames | 295 |
| Selected right hand detected | 83 |
| Valid fused wrist position | 31 |
| Detected hand, insufficient palm depth | 45 |
| Detected hand, inconsistent palm depth | 7 |
| No selected hand | 212 |

The original detector and all 21 points were preserved. Finger calibration was absent and camera geometry remained unaccepted. No claim is made about recall: hand presence was not independently annotated. Raw images were not recorded in this bounded check, so the reason for individual depth-patch failures cannot be established from this log alone. A guided raw RGB/depth recording is still needed, including the user's natural fingers-down poses and all failed frames.

## Physical blockers

1. No local robot runtime, joint/TCP calibration, or camera acceptance configuration exists in this repository. The examples intentionally do not authorize execution.
2. Existing calibration was found in `/home/bx/repos/so101-pick-battery/calibration/robot_profile.yaml` and `model_alignment.yaml`. It references a different URDF/xacro with different joint frames and axes. It is potential source evidence, not a verified calibration for this project's official follower model. The older September calibration YAML also differs from that profile; it must not be silently substituted.
3. The robot's identity, current calibration/firmware offsets, gripper endpoint geometry, fixed TCP, scene, and physical E-stop must be confirmed. Existing software acceptance flags cannot establish these measurements.
4. Physical command expiry, serial timeout, controlled hold, load retention, failure handling, and position/orientation re-engagement have not been exercised on this arm. No signed physical acceptance is present.
5. Sustained camera-driven simulation and depth accuracy/availability acceptance remain outstanding. A short run producing some XYZ observations is not sufficient.

Do not enable hardware by filling acceptance flags with guessed values. Continue with camera-depth diagnosis, model-specific calibration verification, and controlled physical commissioning.

## User-identified hardware and calibration follow-up

The user identified an SO101 at `/dev/ttyACM0` and calibration ID `my_follower_arm`. The file was located at `/home/bx/~/BZ/robot-records/calibrations/so101_follower/my_follower_arm.json` (a literal tilde directory), not `/home/bx/BZ/...`. Its offsets/ranges differ from both the old project calibration and the default LeRobot cache. It is now the designated source for preparation, but has not been compared with live registers.

Host-side filesystem inspection confirmed `/dev/ttyACM0` and the stable symlink `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14029749-if00`. This did not open the serial port.

Created ignored local drafts `config/robot.local.json`, `config/runtime.robot.local.json`, and `config/commissioning.local.json`. Only the port, motor IDs and recorded raw bounds were filled from the identified source. Provenance includes the exact source path, SHA-256 and original calibration data. Joint signs/zeros, conversion scales, physical limits and TCP remain unset; all acceptance flags remain false. Verified the draft is rejected by `validate_acceptance` before a connection can occur. These drafts supersede the earlier observation that no local files existed, but do not resolve physical commissioning.

The user is unsure whether a physical E-stop exists. Explained the distinction from the keyboard/software stop and requested identification of the physical controls. No motor writes, torque changes, live-register reads, or physical stopping tests were performed.

The user subsequently confirmed **only an ordinary power switch**. Recorded this in local commissioning provenance; physical-stop acceptance remains false. No dedicated E-stop is reported and the power switch's stopping/load-retention behavior has not been validated. Powered teleoperation remains blocked under repository rules 12–13. Camera diagnosis and offline calibration/model checks can continue independently.

### Superseding demo scope decision

The user explicitly accepts a software stop for this supervised lab demo and instructed that a physical E-stop must not be a prerequisite. Updated AGENTS.md rule 12, the worker acceptance gate, example/local config and active acceptance guidance accordingly. The earlier physical-E-stop blocker above is historical and no longer applies. `software_stop` replaces `physical_estop` in the worker's required checks; the local flag remains false pending actual verification, rather than fabricating a successful physical test. Regression coverage verifies that a software-stop declaration is accepted without a physical-E-stop declaration and that an unverified software stop is rejected. Calibration/TCP and depth-tracking issues remain unresolved independently of this scope decision.

### Preparation and live read-only inspection

The user subsequently reported that software stopping works. Recorded that provenance and set the local `software_stop` field accordingly; it is user-reported evidence, not an independent agent physical test. Other acceptance fields remain unresolved. The local runtime now explicitly controls `gripper_link` with an identity tool offset for the demo, rather than claiming a measured fingertip TCP.

Added `tools/inspect_robot.py`, an explicitly invoked read-only inspector with register-write methods blocked and torque-preserving disconnect. Confirmed no other process was reported by `fuser` before opening the port. The first inspection caught an SDK integration defect: the installed SDK uses `Operating_Mode`, not `Mode`. Corrected both inspector and hardware worker. The repeated bounded inspection succeeded; `robot-inspection.json` contains exact results. All six motors' offsets/ranges match the user-identified calibration, all six operating modes are position mode (0), and torque was off (0) on all six. Positions were read without motion commands. Elbow position was 3038, one tick above saved maximum 3037; no bound was changed to hide that finding.

The reusable procedure, responsibilities and commands are in `docs/10_DEMO_COMPLETION.md`. Model alignment, raw depth diagnosis, gripper mapping, scene verification, sustained live-hand simulation and first physical end-to-end validation remain pending. Existing saved synthetic replay is rerun after these changes; no physical motion is claimed.

### User recording and failed launch diagnosis

Analyzed all 900 attempts and raw NPZ images from `outputs/depth-prep-01` without
dropping failed frames. The original fusion results reproduce exactly: 5 clock
startup errors, 93 unselected-hand frames, 136 insufficient-depth frames,
577 model/depth inconsistency failures, 11 degenerate-coverage frames, and 78
valid fused positions. There were 802 selected-hand detections. Examined raw
color/depth images including natural fingers-down poses; many failed frames
have a coherent observed surface but incompatible model-relative palm geometry.

Added explicit `--position-method surface_wrist` as an alternative, retaining
the default `model_translation` and the original detector, 21 keypoints,
preprocessing, depth patch thresholds and rejection thresholds. This uses the
currently observed skin point at the wrist pixel, requiring valid wrist depth
and noncollinear palm-surface support. It never fills missing depth from history.
The reference differs from anatomical model translation and can still drift
with skin/occlusion changes; it is not validated hardware geometry. The geometry
fingerprint includes the alternative method, so baseline camera acceptance
cannot silently authorize it.

Full raw replay via `tools/analyze_rgbd_recording.py` produced 616 surface-mode
positions, 182 insufficient-support failures, 4 degenerate-support failures,
and the same 98 clock/unselected-hand frames. Longest consecutive valid streak:
199 frames versus 16 for the baseline. Trace and summary:
`depth-prep-01-ab-02.jsonl` and `depth-prep-01-ab-02.summary.json`. The earlier
`depth-prep-01-ab.jsonl` is an incomplete failed analyzer attempt; do not use it.
Availability improved but is not sufficient evidence of complete readiness or
metric accuracy. No thresholds were loosened and no failed frames excluded.

The simulator log `outputs/hand-sim-01/control.jsonl` contains 9,214 cycles:
177 released, then one `tracking_unavailable` at engagement, followed by 9,036
`release_then_engage_required`. Thus E was received, but tracking prevented
movement. Added visible simulator status/input availability/last fault, plus
explicit terminal hints and uppercase keyboard support. The GUI overlay itself
has not been visually inspected on the user's desktop; headless runtime passed.

Removed the hardware signature/date gate. Validation now lists unresolved
setup checks and missing joint mappings; it still rejects incomplete numeric
calibration before launching the worker. Updated the runbook to distinguish
read-only motor inspection from actual joint alignment. The command the user
ran was inspection and did not perform alignment; no repeat is needed merely
to verify the same stored calibration.

Verification: 110 tests passed, one sandbox loopback test skipped; 1,200 offline
decisions reproduced; five-second full-model synthetic headless run engaged
and tracked. Added tests for current-depth surface reference, rejection when
wrist depth is absent, and software-stop configuration without signatures.
Physical launch remains unfinished because the model mapping and remaining
functional checks have not been completed, not because a signature is missing.

### Mirror directions, depth diagnostics and single-joint interface

Latest `hand-sim-surface-01` inspection: 17,999 vision attempts; 5,219 surface
positions, 3,859 insufficient surface support, five degenerate support, 8,910
unselected hand, six clock errors. All aperture fields were null because no
hand endpoints were supplied. Controller logged 2,006 tracking cycles and seven
engagements, but repeated depth loss caused pauses. No claim of stable tracking.

Added `tools/gripper_test.py`: explicit simulation-only single-joint interface,
official follower model, capture-closed/open buttons and keyboard controls,
saved hand-angle calibration, soft takeover, stop and explicit recovery. It
uses current finger-angle observations independently of wrist XYZ, retaining
all 21 upstream points. `--start-camera` launches the existing vision worker in
its own environment, with mirror display only. No motor SDK is imported.
This is a kinematic preview, not physical collision/dynamics validation.

Changed the user-requested base-fixed mapping to toward-camera → +X forward,
up → +Z, mirror-right → -Y. Determinant remains +1. Simulator starts behind
the robot looking forward. Changed the example and local hardware mapping too;
this defines operator intent, not measured room extrinsics. Unmirrored inference
and geometry are unchanged. Saved hand endpoints can be loaded into the ordinary
vision worker with `--aperture-calibration`.

Added per-patch diagnostics without altering thresholds. Replaying all 900 raw
frames retained exactly the previous baseline and surface-mode counts. Among
the 182 surface-support failures, 633 palm patches failed for depth holes and
five for mixed depth. New trace: `depth-patch-diagnostics.jsonl`. No temporal
fill or invented observation was added.

Added optional 640×360 native depth aligned to unchanged 640×480 color input;
default original worker profile remains 640×480. D455 read-only profile query
confirmed both modes at 30 Hz. Manufacturer nominal Min-Z is 0.26 m versus
0.32 m respectively; the earlier recording's median valid palm depth is 0.343 m.
Near-range operation is a plausible contributor, not a proven explanation for
every dropout. The new launcher selects the near-hand mode explicitly. Live
availability improvement at this resolution is still unmeasured; preserve the
same required hand pose ranges when evaluating it.

Verification: 113 tests passed, one sandbox socket test skipped; all 1,200 offline
decisions reproduced. Gripper GUI opened and ran three seconds on the desktop
using NVIDIA GL without camera or motor access (not a screenshot-based visual
quality audit). Headless test ran. Separate four-second simulated input session
with no wrist pose captured .2/1.2 rad endpoints, saved the calibration and
followed opening to .80. Model direction tests verify forward/up/mirror-right
and unchanged orientation at re-anchor. No physical motion was performed.
Usage and exact commands: `docs/11_GRIPPER_TEST.md`.

### Superseding automatic gripper demo

The user rejected endpoint capture and engage controls for the single-joint
preview. Re-read upstream `show_realtime_retargeting.py` and its one-joint Panda
gripper configuration. Replaced the capture/soft-takeover interface with an
automatic color-only preview: launch → unchanged detector → existing finger
opening angle → official revolute gripper angle, clipped to model limits.
No user calibration, calibration file, capture step or engage action is needed.
The arm stays fixed. It is direct angular retargeting, not the upstream Panda
prismatic vector optimizer and not a measured human-to-robot fingertip match.

New `tools/vision_gripper.py` uses D455 color only and the pinned original
detector. Its distinct simulation-only preview protocol cannot be consumed as
the physical arm's observation envelope. Each normal launch creates new logs
and IPC. Missing input holds the displayed pose; fresh input resumes automatic
kinematic animation, matching the upstream visualization interaction. Physical
and full-arm re-engagement semantics are unchanged. Optional Space pause and Q
quit replace the setup panel. The launcher starts its camera by default.

The RGB-D worker also defaults to direct joint-angle aperture mapping when no
legacy endpoint arguments are supplied; this is recorded in the geometry
fingerprint. Mechanical motor calibration is not inferred from gesture mapping.
Updated AGENTS.md and current instructions so gesture calibration will not be
reintroduced as a demo prerequisite.

Verification: 113 tests passed, one sandbox socket test skipped; 1,200 offline
decisions reproduced. The entire original 326-frame video ran through the real
upstream detector and automatic preview without calibration or engage commands:
160 detected frames; the renderer had 105 following cycles with joint targets
spanning approximately 0.134–1.745 rad. Cycles and frames are different rates,
not detection recall. Evidence: `automatic-gripper-video/`.

An eight-second desktop test opened the official-model GUI and captured 204
live D455 color frames without depth or motor access. No selected hand was
detected during that bounded live test, so it validates capture/GUI startup,
not live gesture-following accuracy. The video test establishes the automatic
detector-to-renderer path. Evidence: `automatic-gripper-live/`. The bounded
shutdown initially printed KeyboardInterrupt from the child; graceful handling
was added. No physical motion or gesture calibration was performed.
