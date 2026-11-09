# Experimental proximity mapping and split preview

Implemented the requested gripper-test experiment, not a fitted closed/open
classifier. The supplied clips remain byte-identical. Provisional defaults:
distance from thumb tip to the finite triangulated palm, normalized by MCP palm
width, maps [0.15, 0.85] to [0, URDF maximum] with clipping. The old angle remains
logged for comparison. This does not establish actual skin contact or classify
the duplicate clips correctly; distinct endpoints are still needed to validate
the rule. No learned/user-calibrated thresholds are claimed.

UI now uses one window: official gripper model on the left, cropped mirrored
hand on the right. The gripper camera uses its local -Z extension as screen up;
hand presentation rotates wrist-to-middle-MCP upward. Only presentation changes:
the unchanged full-frame detector and all 21 internal points are retained.
Full official URDF/meshes are loaded; other links are hidden/masked for display,
not deleted or used as a new collision model. Core offscreen rendering and vision
UI remain in separate existing environments, exchanging atomic image files.

Visually inspected `split-preview/preview.png` rendered from the user's recording:
gripper left, hand right, both upward, no visible arm or room/person outside the
hand cutout. The hand mask is landmark-hull based, not semantic segmentation;
small background regions within the hull can remain. User can Q to quit or
Space to pause. No calibration steps or motor access.

Verification: 114 tests passed, one sandbox socket test skipped; 1,200 offline
decisions reproduced. Added tests for exact endpoint outputs, rigid-rotation
and scale invariance of proximity, and valid preview protocol mapping. Ran the
new detector/render/UI path against recorded video. Preview output is in
`split-preview/`; it is not evidence that the provisional thresholds match the
user's intended closed/open endpoints.
