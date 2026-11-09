# Complete second live-recording position audit

Recording: `outputs/hand-follow/20261106-093702-636085eb/camera`. Source JSONL SHA-256: `1423ac3c7cc0e90f6a0d20ffe7cfc4395377a1b906e724d6dd2395a8dc825b85`. No camera or motor access occurred during this analysis. No estimator thresholds or code changed.

All **2,857 attempts** (sequences 0–2856) are retained: **2,385 upstream selected-hand detections**, **465 upstream non-selections**, and **7 initial capture-clock errors**. Captured timestamps span **147.063 s**. The last archive, `002856.npz`, is incomplete/corrupt: `depth_m` is missing and its color member fails decompression. It remains untouched. Its original JSONL observation/diagnostics are preserved in the replay-error row, but it does not count as independently replayed depth evidence. There are **2,849 replayable depth frames** and **one archive error**.

| Position method | New observed / all | New observed / detected | Earlier 900: all / detected | New longest valid span | New longest invalid span |
|---|---:|---:|---:|---:|---:|
| model_translation | 1408/2857 (49.28%) | 1408/2385 (59.04%) | 8.67% / 9.73% | 23.077 s | 21.009 s |
| surface_wrist | 1545/2857 (54.08%) | 1545/2385 (64.78%) | 68.44% / 76.81% | 19.976 s | 20.842 s |
| wrist_surface_fit | 1906/2857 (66.71%) | 1906/2385 (79.92%) | 75.89% / 85.16% | 29.613 s | 20.876 s |

The new method remains the most available of these three on the same recording, but **66.71% of all attempts is not reliable continuous following**. Its longest invalid run is **419 frames / 20.876 s** (1537–1955), comprising 201 upstream non-selections, 121 insufficient-surface-support frames, 92 unsupported-ray frames, and five degenerate-surface frames. The next long runs are 280–451 (172 frames / 8.537 s) and 0–165 (166 attempts; 8.236 s measured between known timestamps). Its longest valid run is 959–1536 (578 frames / 29.613 s). Spans are first-to-last sampled times; unknown startup timestamps do not establish an inferred outage duration.

Within selected-hand detections, new-method failures are **271 insufficient coherent patches, 192 unsupported wrist rays, 13 degenerate surfaces, one contradictory local depth, one excessive patch gap, and one unreadable archive**. Online JSONL recorded 1,907 observed positions; independent replay confirms 1,906 because the final archive is incomplete. The report does not silently promote the original final observation to replayed evidence.

The earlier recording selected a hand on 89.11% of all attempts; this recording does so on 83.48%. New-method detected-frame availability changes from 85.16% to 79.92%. The recordings contain different hand motions, framing and object interaction, so these differences do not isolate an estimator change or camera-profile effect. This recording explicitly logs native depth 640×360, aligned to 640×480 color; the older metadata does not record the native depth stream dimensions.

## Representative RGB-D failures

[Depth failure contact sheet](live-01-depth-failures.jpg) shows full RGB, wrist RGB crop, and raw registered depth with accepted coherent patch locations. [Numeric inspection](live-01-depth-inspection.json) preserves diagnostics for the inspected sequences.

- **373:** a fully visible open palm; 84.94% of the wrist ±24 px ROI is zero depth, plus 42 nonzero out-of-range points. Only 5/121 patches pass. There is no valid depth within 4 px of the wrist ray.
- **904, 1745, 1817:** visible hand near the lower/left image region; ROI zero-depth fractions are 94.62%, 94.36%, 93.19%. There is no valid near-ray depth, and only 2, 2, 6 coherent patches out of 121. This is missing depth, not evidence that relaxed plane thresholds would recover the correct location.
- **451:** only a narrow surviving depth strip; spatial coverage is 1.80 mm, below the unchanged 3 mm requirement.
- **2271 and 2469:** fully framed flexed/horizontal hand poses at near-ray depths approximately 0.410 m and 0.340 m. ROI depth is 56.51% and 64.58% zero. There are 35 and 31 coherent patches, but their separated/one-sided support does not surround the wrist ray. These natural poses remain part of acceptance coverage.
- **231:** a near-ray depth outlier disagrees with the fitted plane by 0.796 m, causing conservative rejection. This does not establish that the plane itself is incorrect. **233:** nearest coherent support is 6.176 px away, exceeding the added 6 px gap guard.
- **166** supplies a successful comparison: local depth about 0.411 m, 84% valid ROI depth and 99/121 coherent patches.

An independent RGB review inspected 44 stored frames across the session. Fully visible hands were upstream-unselected at **155, 164, 165** (open palm), **194** (more edge-on), and **2435, 2452, 2468** (extended, roughly horizontal/palm-down hand). Similar adjacent poses become selected at 166 and 2469. RGB alone does not distinguish detector failure from handedness/selection logic. Some other unselected frames (86, 302, 311, 1550, 1647, 1663) show the hand very near the camera and partly cropped; they cannot be called hand absent. Frame 2555 includes phone pickup with overlap/motion blur. These are specific inspected examples, not an aggregate labeled pose evaluation. No pose range was narrowed.

[Uniform RGB samples](live-01-uniform.jpg) and [selection-transition samples](live-01-detector.jpg) retain the visual evidence.

## Timing, motion and audit scope

New-method fusion wall time P95 is **3.975 ms**, excluding detector inference, capture, compression, publication and control. Offline availability does not mean the live controller received fresh usable input; publication/recording latency is being investigated separately. The new method has 1,880 adjacent observed pairs, motion-step median **1.205 mm**, P95 **10.552 mm**, maximum **52.374 mm**, and no steps over 60 mm. These include actual motion and capture intervals and are not precision/jitter measurements.

For coverage comparison, new and original-model methods both observe 1,395 attempts; the new method alone observes 511 and the original alone 13. New and older surface_wrist both observe 1,539; new alone observes 367 and the older surface method alone six. Their reference definitions differ, so these numbers do not authorize silently switching references or treating availability as accuracy.

All 2,849 replayable wrist poses, position/orientation evidence, identity and timestamps match the saved online output exactly. The analyzer replays `fuse_frame`; the worker’s later automatic gripper mapping is outside this comparison. Consequently full observations differ only in aperture/aperture_evidence, and diagnostics only in the direct-gripper-mapping label. This is a position audit, not a physical or gripper-control replay. A control replay must consume the saved online `direct_joint_angle` observations, or use an explicit matching gripper adapter, to reproduce the live jaw motion and collision-check workload. An earlier control replay that held aperture does not validate live gripper or collision timing.

Artifacts: `live-01-ab.jsonl` contains every attempt and complete replay observations/diagnostics; `.summary.json` contains denominators/distributions; `.inputs.json` hashes every existing archive and probe stream; `.periods.json` preserves every validity run; `.validation.json` contains source comparison and implementation hashes. The interrupted first analysis remains as `live-01-ab.incomplete.jsonl`.

The analyzer now records unreadable/missing depth archives explicitly instead of aborting or discarding attempts. **10 analyzer tests passed**, including incomplete/corrupt archive accounting and preservation of the original observation. No hardware SDK calls occurred in those tests. No sensor accuracy, metric ground truth, or real-arm reliability is established by this recording.
