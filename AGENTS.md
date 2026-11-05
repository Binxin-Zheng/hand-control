# Repository rules for coding agents

1. Read README and docs/01..07 before changing architecture. This is an independent AnyTeleop-inspired reference, not the original complete source release.
2. Hardware commands are NOT implemented. Never add implicit connect(), home(), torque enabling, live motor write, or background auto-start to import paths, tests, demos, or setup scripts.
3. All numerical interfaces use meters, seconds, radians; quaternion order xyzw; column vectors. `R_A_B` maps coordinates from B to A. Camera measurements are unmirrored; mirrored previews never change geometry.
4. Preserve upstream detector, all 21 internal keypoints, handedness handling and original inference preprocessing as the reproducible A/B baseline. A two-finger UI does not authorize deleting model keypoints.
5. Do not treat wrist-centered MediaPipe/dex points as global position. Explicitly carry depth/model/prediction provenance. Prediction must not count as observed tracking evidence.
6. SO101 arm has five controlled joints; gripper is separate. Never promise arbitrary exact 6D tracking or replace task-space IK with ad hoc XYZ-to-motor formulas.
7. Use actual measured robot state for re-engagement. Position and orientation are both re-anchored. Do not silently add orientation offsets after latching. Do not change base-fixed deltas into live hand-frame deltas.
8. No automatic restart after tracking loss, camera/identity change, target backlog, or E-stop. Require explicit release and re-engage. Preserve gripper load on dropout; never auto-open.
9. Existing public `handtrack.packet.v1/v2` consumers, if present in the user's repository, require explicit adapters and regression tests. New `teleop.hand_observation.v1` is NOT silently wire-compatible.
10. Preserve the user's comfortable fingers-down poses in acceptance recordings. Do not pass by narrowing pose ranges, dropping difficult frames, lowering thresholds, or labeling predictions as measurements.
11. Offline URDF extract is not a collision model or calibrated hardware TCP. Rendering/collisions must use official full follower URDF and meshes, honoring joint.origin, joint.axis and visual.origin.
12. Hardware integration requires joint name/unit/sign/zero calibration, real TCP, collision guards, a local expiring-command watchdog, manual E-stop, and signed-off acceptance. Driver owns the serial port exclusively.
13. `None` candidate command means trigger controlled stop, not keep the previous distant target alive and not blindly switch off torque. Physical stop behavior must be implemented and tested locally.
14. Do not claim a cuRobo adapter exists until it runs against a locked actual version. Do not mix v0.7 API snippets with current v0.8/v0.9 APIs.
15. Core, vision, motion-GPU and robot SDK environments remain isolated. Never `sudo pip`, change base conda, downgrade system NumPy, overwrite CUDA, or ignore pip dependency conflicts.
16. Run tests and offline replay after changes; report EXACTLY what was tested. Synthetic correctness does not establish camera/hardware accuracy or reliability.
