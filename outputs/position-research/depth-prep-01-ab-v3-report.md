# Complete recording comparison after surface-support corrections

Input: `outputs/depth-prep-01/frames.jsonl`, SHA-256 `24858c57f840ce05fb1330e64cec75472aaa7606ce32090814504a288152c661`. All 900 attempts retained: 802 selected-hand detections, 93 upstream non-selections, 5 capture errors. The same saved detector outputs and RealSense SDK deprojection feed all methods.

| Method | Observed/all | Observed/detected | Longest valid span | Longest invalid span | Fusion P95 |
|---|---:|---:|---:|---:|---:|
| model_translation | 78/900 (8.67%) | 78/802 (9.73%) | 0.7337 s | 11.4380 s | 0.700 ms |
| surface_wrist | 616/900 (68.44%) | 616/802 (76.81%) | 9.7036 s | 2.2010 s | 0.755 ms |
| wrist_surface_fit | 683/900 (75.89%) | 683/802 (85.16%) | 9.7703 s | 2.1676 s | 2.378 ms |

Timing covers `fuse_frame` only, excluding detector, capture, image reads/hashes, and control. Durations are first-to-last timestamp spans within uninterrupted states, not inferred outage durations.

Surface-fit version 2 honors the supplied association mask in every 5×5 patch. The new standalone wrist ROI has a maximum 24 px half-width and is geometric association only, not segmentation. Existing 80% patch/inlier support, 15 mm patch spread, 12 mm plane residual, and 3 mm spatial coverage thresholds remain unchanged. Added conservative conditions require current depth within 4 px of the wrist ray, a coherent inlier patch within 6 px, and no local depth contradicting the plane by over 12 mm.

Adversarial tests reject a background ring surrounding a small nearer foreground patch, a near-center foreground patch with missing center, oversized surrounded depth holes, isolated valid depth surrounded by a large hole, and insufficient mask support. The runtime test proves all-False legacy visibility rejects the new method unless a separate explicit wrist association mask is provided. The existing small 7×7 depth-hole test still passes.

Four previously accepted recording frames are now rejected: 227, 228, and 329 had local depth contradictions of 40.83, 42.67, and 33.13 mm; 561 had nearest coherent support 6.24 px away. Frame 513 uses another supported sampling radius and moves 1.10 mm. Both original methods' complete observations AND diagnostics are exactly identical to the preceding run on all 895 decoded rows. No original baseline keypoints, handedness, or inference preprocessing changed.

For the new method, 657 adjacent observed pairs have motion-step median 2.759 mm, P95 16.126 mm, and maximum 84.437 mm; five exceed 60 mm. These contain actual hand motion and capture intervals, so they are not precision or jitter errors. The unchanged model orientation exceeds `0.12 + 8*actual_dt` on 40/657 of those pairs and 48/798 adjacent selected-hand pairs overall. This is a potential guard diagnostic, not an executed control replay.

Validation: `PYTHONPATH=src .venv-core/bin/python -m pytest -q tests/test_surface_position.py tests/test_rgbd_analysis.py tests/test_runtime_rgbd.py` — 44 passed. `PYTHONPATH=src .venv-core/bin/python examples/replay.py outputs/readiness/offline/trace.jsonl` — 1,200 recorded decisions reproduced, no motors or camera used. `git diff --check` passed.

Reproduce complete A/B with a fresh output filename: `PYTHONPATH=src .venv-vision/bin/python tools/analyze_rgbd_recording.py outputs/depth-prep-01 --methods model_translation surface_wrist wrist_surface_fit --out outputs/position-research/FRESH.jsonl`.

Artifacts: `depth-prep-01-ab-v3.jsonl` (complete observations and diagnostics), `.summary.json`, `.inputs.json` (all image and probe hashes), `-regression.json`, and `-implementation.json` (source snapshot hashes).

Limitations: this is a local visible-surface reference intersected by the detected wrist ray, not an anatomical wrist joint. A geometric ROI cannot identify skin or disambiguate a foreign surface completely covering the wrist; those cases need trustworthy segmentation, another view, or rejection evidence. A small bounded surrounded hole is local surface interpolation, not direct depth at the ray. No synthetic test or availability percentage establishes camera accuracy, correct real-world direction, or reliable physical teleoperation. The recording has no independent position ground truth.
