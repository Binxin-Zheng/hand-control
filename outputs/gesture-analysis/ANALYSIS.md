# Closed/open gesture input analysis

The two supplied files are byte-for-byte identical:

- `sample/hand-guesture/close.webm`
- `sample/hand-guesture/open.webm`

Both contain 1,629,092 bytes and have SHA-256
`3a4f63696e003689c7263887efcb98a10d2a8e0d32b9a65e4df590bbe94a934c`.
Binary comparison also confirmed equality. Mapping one recording to zero and
the identical recording to maximum would assign contradictory labels to every
pose. No thresholds or control mapping were changed.

Processed the shared recording once with the unchanged audited upstream detector
(Right hand, selfie=False, original RGB preprocessing, all 21 points). OpenCV
decoded 981 frames spanning presentation timestamps 0–35.1 s: 942 selected-hand
detections and 39 unselected/no-detection frames. All decoded frames were retained,
including failures. Container frame-count metadata estimated 1,054 frames; that
metadata estimate is not the actual decoded-frame count. Counts are not annotated
hand-present recall.

Artifacts:

- `shared.jsonl`: full landmarks and timestamps for every decoded frame.
- `shared-annotated.mp4`: complete decoded recording with joint overlays.
- `shared.summary.json`: source hash, detector hash and environment versions.
- `shared-features.jsonl`: candidate signals with every failed detection retained.
- `feature-summary.json`: candidate distributions; no fitted closed/open thresholds.

Candidate signals: existing thumb/four-finger angle, thumb/index angle,
thumb-to-index tip distance normalized by palm width, thumb-to-palm-centroid
distance, and distance to the estimated palm plane. Plane distance alone does
not measure contact with the finite palm surface, and none of these is proven
to classify the desired gesture from contradictory labels.

Next input needed: replace the mistakenly duplicated clip with the actual other
gesture, or supply closed/open timestamp labels within this recording if it was
intended as one combined video. Once labels are distinct, compare features on
temporal held-out portions and report all missed/ambiguous frames before choosing
a fixed proportional mapping. Closed maps to exactly 0 radians, open to the
official gripper joint maximum; no runtime user calibration is required.
