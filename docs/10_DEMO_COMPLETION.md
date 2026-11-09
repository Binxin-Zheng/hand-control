# Complete and run the supervised lab demo

The current goal is correct-direction XYZ arm following with the gripper. The
launcher below uses explicit position mode: hand translation moves the target,
and the measured robot orientation at engagement remains the secondary IK
target. The five-axis arm does not promise exact arbitrary orientation tracking.

This guide records remaining work. Live camera-to-physical-arm operation,
startup hold, local command expiry and integrated stopping have not been
physically verified with the current implementation. The user's reported
working software stop remains recorded as reported evidence. A physical E-stop
is not a prerequisite for this supervised lab demo.

## Already completed

- The official full follower model is available, numerical FK has been checked
  against it, and the collision-envelope fix has passed synthetic simulation.
- D455 capture and the unchanged upstream detector have run on this host. All
  21 landmarks, handedness and original inference preprocessing remain intact.
- The new `wrist_surface_fit` estimator uses current registered depth around the
  wrist ray. It rejects unsupported holes and contradictory near-wrist depth;
  predictions never count as observed tracking.
- Complete replay of `outputs/depth-prep-01` retained all 900 attempts, including
  802 selected-hand detections. Observed positions: original model translation
  **78/900**, older `surface_wrist` **616/900**, and `wrist_surface_fit`
  **683/900** (**683/802** detected frames). The original methods' observations
  and diagnostics stayed unchanged. See the [complete comparison](../outputs/position-research/depth-prep-01-ab-v4-report.md).
  These are availability results, not proof of metric accuracy or reliable
  physical following; the new method still has dropouts.
- `tools/hand_follow.py` launches isolated vision and simulation processes with
  fresh IPC and logs for every session. It uses `wrist_surface_fit`, 640×360
  native depth and explicit position mode. Its user-requested automatic visual
  preview is separate from manual control/physical recovery validation.
- Numerical joint signs, zeros, conservative operating ranges and the gripper
  mapping have been transferred into `config/robot.local.json` from the saved
  physical alignment, accounting for current firmware homing offsets and the
  official model's joint convention. Source hashes, derivation and remaining
  checks are in `config/commissioning.local.json`.
- The saved read-only inspection found matching calibration registers on all
  six motors, position mode and torque off. This verifies the saved register
  comparison; it does not establish current pose agreement or physical stopping.
- The selected demo control point is the `gripper_link` origin with an identity
  offset in `config/runtime.robot.local.json`. A precision fingertip TCP
  measurement is not required for this scope.

## 1. Run live-hand simulation and record the natural poses

Close other camera viewers/workers, then run in a normal desktop terminal:

```bash
cd /home/bx/repos/hand-control
.venv-core/bin/python tools/hand_follow.py --record-images
```

Use `--hand Left` for the left hand. Each launch creates a new directory under
`outputs/hand-follow/` and prints its path. It contains `session.json`,
`vision.log`, `controller.log`, `control.jsonl`, and the camera's `frames.jsonl`
and RGB/depth arrays under `camera/`. No prior recording is overwritten. The
launcher operates the camera and simulator only; it has no motor mode.

The visual preview follows usable tracking automatically. With its window
focused, **Space** pauses/resumes and **Q** quits. Missing tracking freezes the
arm; returning tracking causes a release tick and a new measured simulation
anchor before following resumes. Collision/backlog/timing faults and E-stop
still require deliberate action. The controller owns terminal input. Either worker exiting ends the
session and its companion; the launcher does not restart them automatically.
Optional `--duration 60` ends the session after 60 seconds. `--headless` disables
windows but still uses the camera.

The simulator shows input availability and the last pause cause. For validation
of the explicit physical-control workflow, add `--manual-control`: E engages,
R releases, X stops, C resets while released, Q quits. Manual mode starts released
and requires release/re-engage after loss. The physical backend rejects
automatic preview. The mirrored preview does not change measurement coordinates.

Spend roughly five seconds on each action, keeping your comfortable
fingers-down poses in the recording:

1. Hold the hand still in its natural control pose.
2. Move left/right and up/down slowly, checking the simulated arm's direction.
3. Move toward/away from the camera.
4. Rotate the wrist while keeping its location roughly fixed; watch for
   unintended translation. Position mode keeps the latched robot orientation
   target instead of following hand rotation.
5. Keep the hand still and open/close the thumb comfortably.
6. Remove the hand briefly and return it. Automatic preview should freeze then
   re-anchor without a jump. Separately use manual mode to verify that loss
   remains paused until explicit release/re-engagement.

Finger opening drives the gripper directly. No capture-closed/open procedure,
saved gesture calibration or gesture endpoint arguments are required. The
separate automatic gripper-only preview is described in
[the gripper test guide](11_GRIPPER_TEST.md).

The agent can inspect the session's complete logs and images locally. Preserve
failed attempts, capture gaps and difficult poses. Image compression can reduce
frame rate, so evaluate actual timestamps and input age. This sequence measures
availability and following behavior; millimeter accuracy needs an independent
position reference and is not a prerequisite for the directional-following goal.
The wrist-ray reference is a local visible surface, not the anatomical wrist
joint. Geometric association alone cannot identify a fully occluding object.

## 2. Confirm the transferred model mapping on the current arm

The numerical mapping is already populated; do not redo LeRobot calibration or
change firmware homing offsets as a preparation shortcut. The transfer assumes
the same robot and unchanged motor/horn assembly since the documented alignment.
Confirm that assumption, then compare the current measured pose with the
rendered official model. The agent handles encoder conversion and configuration;
the user does not need to calculate radians or set acceptance flags.

If the assembly or pose does not agree, investigate that discrepancy before
motion. A register match alone does not establish mechanical alignment. Retain
the selected `gripper_link` control point; checking its model convention is not
a demand for a new precision fingertip calibration.

The saved inspection reported elbow position **3038** against accepted raw
maximum **3037**. Recheck the current position and resolve this endpoint before
powered operation: with torque off, the user can place the arm comfortably
inside the accepted range during the guided pose check. Do not widen the limit
or clamp feedback. The previous
inspection is in
[`robot-inspection-follow-01.json`](../outputs/position-research/robot-inspection-follow-01.json).
A subsequent read-only snapshot must have exclusive serial ownership and a new
output path; it is not a substitute for current-pose comparison.

The physical gripper direction and bounded endpoints also need confirmation
against the transferred mapping. Driver aperture conversion and collision
geometry now use the same calibrated joint-angle endpoints. This mechanical
check is separate from the calibration-free hand gesture interface.

## 3. Complete the local physical checks

| Remaining check | Agent handles | User participates | Evidence needed |
|---|---|---|---|
| Current arm/model agreement | Compare measured joints and official-model pose with the transferred mapping; resolve the elbow endpoint | Confirm unchanged assembly and visible current pose | Matching pose, directions and bounded gripper travel |
| Camera directions and scene | Review complete live session, camera-to-base direction mapping, table location and existing collision guard | Move through the natural pose sequence; identify the actual tabletop relative to the base when needed | Correct XYZ direction and accounted-for tracking failures in the real setup |
| Startup and local stopping | Prepare and check the commissioning path; measure startup hold, command expiry, serial timeout, controlled hold and gripper-load preservation | Supervise small bounded checks at the arm and operate the software stop | Recorded current-arm behavior and timing; no unexpected startup movement |
| Release and reacquisition | Inspect state and feedback logs; verify measured-state re-anchoring | Release/re-engage and briefly remove the hand | No jump on re-engagement and no automatic restart after loss |

The chosen camera/base mapping and simulation table settings are not measured
hardware geometry. The agent must reconcile them with the actual setup and
prepare `config/camera-acceptance.local.json` for the emitted camera geometry
hash and current estimator. Keep the selected demo control point. Existing
acceptance fields remain incomplete; source files, synthetic tests and earlier
physical alignment do not make them current hardware acceptance.

The implemented stop candidate replaces the old arm target with current
measured arm positions while preserving the gripper goal and torque. It must be
measured locally for this arm, including command expiry, controller loss and
software stop. The SDK timeout cap has been tested in memory; actual bus timing
and physical hold response remain unverified. Software stop does not protect
against computer, driver-process or bus failure. A fixed-pose hold check alone
does not demonstrate stopping during movement.

## 4. Physical demonstration — pending those checks

The normal hardware entry point rejects incomplete acceptance, including the
hold, command-watchdog and serial-timeout checks, before opening the worker.
An explicit bounded **hold-only commissioning path** now exists to collect
initial evidence without authorizing normal hand-controlled motion. It leaves
all acceptance flags unchanged. It is not yet physically tested on this arm.

After confirming the current arm is supported, clear of obstructions and inside
the accepted limits, the agent can run the supervised first command-expiry check:

```bash
.venv-core/bin/python tools/commission_hold.py \
  --config config/robot.local.json \
  --robot-python /home/bx/miniforge3/envs/lerobot/bin/python \
  --execute --pose-confirmed --enable-torque \
  --exercise command-expiry --hold-seconds 1 --duration 8
```

This is a physical command, not a simulator. `--pose-confirmed` records the
operator's current-pose confirmation; the previously saved out-of-range elbow
snapshot does not supply that confirmation. `--enable-torque` is required only
when torque is off. The worker still validates every live calibration register,
position mode and measured raw range before its first write. It does not home,
alter calibration or disable torque at exit.

The worker captures its exact startup hold write as the only permitted arm
target for this invocation. Subsequent active writes contain only those five
raw arm goals, never the gripper. Initial startup initializes the gripper goal
only if its torque was off; an already loaded gripper retains its goal. Any
expiry, stop, release after activation or command error ends active eligibility
for this invocation. A new invocation must capture a new measured anchor.
There is no hand input, arbitrary trajectory or recovery to an old target.

The helper sends one release and a short stream of that fixed hold target, then
withholds commands to observe local lease expiry. Other explicit exercises are
`software-stop`, `release` and `controller-exit`. The software-stop exercise
tests the same driver stop command programmatically; the operator can also type
**X then Enter** at any time. A silent or stale feedback stream ends command
renewal. The worker has its own finite duration (at most 30 seconds), independent
of the helper, and retains the normal stop/cleanup behavior.

Each invocation writes every command and received feedback row, timestamps,
input hashes, worker errors and a summary to a new `outputs/commission-hold/`
directory. A reported protocol transition is evidence of that transition, not
proof of physical hold quality or braking while moving. Inspect actual behavior
and timing; a failed/forced worker exit is recorded as a failed diagnostic.
Serial/bus-failure behavior and subsequent small collision-checked movements
still need local verification through the ordinary guarded control path.

The eventual camera and controller must share the same fresh vision path and
use the reviewed camera geometry, `wrist_surface_fit`, explicit position mode
and local robot configuration. The robot driver exclusively owns the serial
port. Torque enabling remains an explicit startup action after hold preparation;
there is no implicit homing, recalibration or automatic restart. Hardware
commands are entered in its controller terminal followed by Enter.

Completion means actual small hand-controlled movements in all three axes,
gripper open/close, release/re-engage without a jump, tracking-loss hold without
automatic restart, and verified local software-stop behavior. Then perform the
complete pick–move–place demonstration and retain the logs. Hand orientation
following or precision fingertip calibration is not added to this XYZ demo's
completion requirements.
