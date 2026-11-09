# Offline two-candidate detector experiment

**Result: limited recovery, with unresolved duplicate-candidate selection. Keep
this offline; no camera/runtime detector change is justified by this run.**

The explicit `--max-hands 2` experiment keeps the pinned upstream source,
original `detect()` and first-matching-handedness selection, all 21 keypoints,
unmirrored RGB preprocessing, temporal tracking, default model settings, and
both 0.8 confidence thresholds. It closes the original unprocessed one-hand
graph, then creates one isolated fresh graph differing only in
`max_num_hands=2`. The default one-hand path remains unchanged. This is an
experimental graph variant, not the original baseline.

## Complete accounting

Both runs retain **all 2,857 source attempts**: seven initial capture errors,
**2,849 readable RGB frames processed once each in original chronological
order**, and the same corrupt terminal archive at sequence 2856. No internal
gap, skipped hard frame, inference error, or additional inference call. The
source file remains unchanged; both executed-script snapshots match their
recorded SHA256 hashes.

| Readable-frame outcome | Original one-hand baseline | Two-candidate experiment |
| --- | ---: | ---: |
| Selected by unchanged handedness rule | 2,384 | 2,402 |
| Raw MediaPipe no hand | 269 | 266 |
| Raw hand(s) filtered by handedness | 196 | 181 |

All **2,384** original readable selections remain selected as booleans; **18**
additional frames are selected, **zero** original selections are lost, and 447
remain unselected. The recorded original denominator is still 2,385 selected
frames because its final selected frame cannot be independently replayed.
Boolean availability does not guarantee identical candidate geometry or identity.

Raw candidate counts in the experiment: 266 zero, 2,529 one, 54 two. The original
baseline reprobe previously matched all 2,849 saved probes exactly. The
experimental two-candidate run has 1,937 exactly unchanged selected probes plus
447 exactly unchanged unselected probes. Other differences are variant effects,
not demonstrated nondeterminism of the unchanged baseline.

## Recovery and identity audit

- **Single-hand starvation block 9–164:** recovered **15/156 frames**, precisely
  150–164. All 15 have candidate 0 with expected raw label `Left` and candidate 1
  with filtered label `Right`. Candidate 1 matches the original filtered hand
  closely: all-21-point image RMS difference **0.044–0.174 px**. The newly
  selected candidate is spatially separate: **184.1–273.0 px** RMS from that
  original hand. Representative overlays at 150 and 164 show the selected hand
  at image left and the original distractor at image right. This establishes
  potential recovery of the desired second hand in these recorded examples
  without selecting the original distractor. The other **141** frames remain
  filtered; at representative frame 86 the desired hand is near/crossing the
  image edge and only the other hand is returned. No pose range was narrowed.
- Other recovered frames are **8, 165, and 2555**. Overlays show the intended
  visible hand selected, but these three examples are not a general identity
  guarantee or proof of metric depth availability.
- **Palm-down label-flip block 2435–2468:** **0/34 recovered**. All 34 remain
  filtered. Increasing candidate count does not solve this target label flip.
- **Long no-hand block 1550–1744:** **0/195 recovered**. The previous longest
  global-position outage (1537–1955) still has the same 218 selected, 199 raw
  no-hand, and two filtered frames. Independent depth holes remain unresolved.
- **35 frames return two candidates both labeled `Left`.** The unchanged
  first-match rule chooses candidate 0; candidate 1 is closer to the original
  selected landmarks in every such frame, and exactly equals those landmarks
  in 34 cases. Representative overlays at 278, 893, 2340, 2536, 2540, and 2553
  show overlapping hypotheses on the same physical hand rather than a distant
  second person's/other hand. This is still meaningful estimator ambiguity:
  across common selected frames, all-21 image RMS difference reaches **68.77
  px**, and wrist-image difference reaches **151.86 px** (2550). We have no
  anatomical ground truth establishing which hypothesis is correct. Candidate
  ordering is not persistent identity, and this experiment does **not** prove
  identity-safe selection over arbitrary two-hand or overlapping-hand scenes.

All candidate coordinates, labels, scores, selected index, and associations are
retained. Diagnostic image distances are not control thresholds, global metric
accuracy, or a proposed substitute for measured identity handling. The next
design issue would be explicit candidate association and duplicate suppression;
neither is implemented or authorized for runtime by this report.

## Timing and validation

Original detect-call median/P95: **12.003/14.025 ms**. Experimental median/P95:
**25.445/27.003 ms**; maximum 48.326 ms, total run 82.097 s. These are sequential
offline timings after the separate clean collision benchmark, not live
end-to-end latency measurements.

**16 pure/fake tests passed in 0.02 s** on the final script. They include exact
original method-return transparency, unchanged default construction, explicit
two-candidate graph replacement after closing the unprocessed original graph,
unchanged confidence settings, all-21 serialization, no-hand/filter distinction,
complete corruption accounting, no temporal skipping, and diagnostic failure
handling. `git diff --check` passed. No devices, vision worker changes, physical
commands, threshold changes, or new depth-fusion experiment occurred.

## Artifacts and field clarification

- `live-01-detector-max2.jsonl`: complete raw experiment and original-probe comparison.
- `live-01-detector-max2.summary.json`, `.inputs.json`, `.log`: counts, provenance,
  input hashes, timing, and retained terminal archive error.
- `live-01-detector-max2.script.py`: exact script used for this run, matching its
  input-manifest hash; `live-01-detector-reprobe.script.py` similarly preserves
  the original baseline replay script.
- `live-01-detector-max2.comparison.json`: complete selection differences and
  duplicate-label cases; `.associations.jsonl`: every frame's image association.
- `live-01-detector-max2.compare.py`: reproducible association analysis script.
- `live-01-detector-max2.examples.jpg`: selected candidate **green**, other
  candidate **yellow**, original selected wrist **magenta cross**. Colors
  describe algorithm selection, not ground-truth correctness.

The preserved raw run uses a legacy field `selected_count`: this is **literal
upstream return `n`**, meaning total returned raw boxes when one matching hand
is selected, otherwise zero; it never means two hands were selected. Likewise
its legacy note says “fresh original graph”; the explicit variant and
`graph_settings` correctly identify the experimental two-candidate graph. These
original evidence files were preserved. The current reusable script fixes this
wording and uses `upstream_return_count`, `selected_hand_count` (0/1), and
`raw_candidate_count`, and explicitly states frame-local candidate-index semantics.

Command used:

```sh
.venv-vision/bin/python tools/reprobe_recorded_detector.py outputs/hand-follow/20261106-093702-636085eb/camera --out outputs/position-research/live-01-detector-max2.jsonl --max-hands 2 > outputs/position-research/live-01-detector-max2.log 2>&1
```
