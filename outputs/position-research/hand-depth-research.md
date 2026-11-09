# RGB-D surface fitting research audit

Read-only audit, 2026-11-06. No installation, camera access or repository changes.

## DART: actual code inspected

Repository: https://github.com/tschmidt23/dart
Audited master: 37adc66580a15c5b438986d77621ebd8f5f450c3 (2019-04-06).
Commit verified via GitHub API: https://api.github.com/repos/tschmidt23/dart/commits/master

Code:
- https://github.com/tschmidt23/dart/blob/37adc66580a15c5b438986d77621ebd8f5f450c3/src/optimization/kernels/obsToMod.cu
- https://github.com/tschmidt23/dart/blob/37adc66580a15c5b438986d77621ebd8f5f450c3/src/optimization/optimizer.h
- https://github.com/tschmidt23/dart/blob/37adc66580a15c5b438986d77621ebd8f5f450c3/README.md

The inspected CUDA kernel associates valid observed depth vertices with an articulated model's signed-distance fields. It rejects excessive distance, optionally checks normal agreement, and uses Huber loss (static const LossFunctionType lossFunction = HuberLoss). This is observed surface-to-model-surface fitting; it does not equate a raw skin depth with a joint center. Model geometry and kinematic frames are distinct.

Dependencies documented by repository: C++, CUDA, Eigen3, GNU libmatheval, libjpeg, libpng, tinyxml; optional Pangolin/OpenNI/DepthSense/Assimp/gtest. Tracker requires an active OpenGL context. README's legacy dependency commands are not an installation recommendation; no current CUDA compatibility was tested. No ready D455 Python drop-in was established.

## DexPilot: why DART alone is insufficient

Primary paper: https://arxiv.org/html/1910.03135v1
Sections III, IV, V and VI: four calibrated/time-synchronized RealSense D415 cameras; articulated hand surface model; DART nonlinear optimization. Authors report local-minimum failures every few minutes without good initialization. Segmented hand point clouds and learned pose priors constrain tracking, including preventing the hand model from sliding onto the forearm. Their PointNet++ pipeline was trained from a glove-assisted annotation phase. Whole-system latency was about one second. These are the authors' setup/results, not results for our D455.

Coauthor repository: https://github.com/ankurhanda/dexpilot
Inspected root contains README.md and dexpilot_main.pdf, with README stating it contains the paper. It is not the complete tracking implementation.

Transferable principle: use current-frame semantic hand detection to restrict dense depth association and use a surface model appropriate to the depth measurement. Do not attempt to install the historical entire stack for a first directional-control improvement.

## Lightweight palm surface precedent

Primary paper (ICCV 2013), Section 5.2:
https://www.cv-foundation.org/openaccess/content_iccv_2013/papers/Sridhar_Interactive_Markerless_Articulated_2013_ICCV_paper.pdf

Sridhar et al. remove fingers from a hand depth mask using erosion/dilation, reject occluding finger points using RANSAC palm-plane fitting (their consensus threshold: 5 mm), then estimate orientation using PCA and Kalman filtering. Their complete system uses multiple calibrated RGB views and a depth sensor and person-specific hand-model fitting. This is useful geometric precedent, not validation of our single-D455 estimator.

## Concrete recommendation and limits (our inference)

For a small CPU estimator, retain unchanged full MediaPipe/dex detection and preprocessing. Use a conservative interior palm image region to collect current aligned depth, reject background/edges/multimodal or poorly conditioned support, and fit a robust local palm surface. Carry depth provenance and validity from the current frame.

Crucially: never use the mean/centroid of whichever depth pixels happen to survive as a fixed wrist reference. Occlusion or hand rotation changes the visible subset and therefore that centroid even without desired translation. Intersect a stable semantic anchor's camera ray with the fitted surface, or estimate a canonical wrist/palm anchor using the model's fixed local geometry plus the current surface constraint. Surface offsets need explicit semantics: skin surface is not the skeleton joint center. Any offset/thickness assumption creates residual rotation-dependent bias and must be tested, not hidden.

A plane only constrains translation normal to that plane; semantic 2D landmarks/reprojection must provide the tangential anchor. At edge-on orientations or with insufficient spatial coverage, reject instead of inventing depth. Previous frames can initialize optimization but are not current observed evidence. Count every unavailable frame and retain comfortable fingers-down poses in evaluation.

Existing 5 mm plane-inlier thresholds in a paper must not be adopted as a camera-accuracy guarantee. The first milestone is repeatable correct XYZ direction and fixed-reference behavior on complete recordings, followed by independent metric validation.

