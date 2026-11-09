# 01 · AnyTeleop 技术调研与开源审查

审查日期：2026-11-05。来源链接和上游文件标识见文末与 `SOURCES.json`。

## 1. “Arm Controls Arm” 到底指什么

项目页明确说明：手指姿态控制机器人手，**手腕位姿控制机械臂**。这里的 arm 不是人体肩、肘到机器人关节的一一对应。[S1]

系统关注的是手腕在固定参考系里的位置与方向。机器人怎样弯肘、怎样组合关节去到这个位置，是机器人运动生成器的问题。对 SO101，也无需采集操作者的肩部和肘部骨架。[S1；后一句为本项目设计推论]

## 2. 论文说明到什么程度

论文把识别、视角融合、手部重映射和机械臂运动生成分开。机械臂部分采用 cuRobo：接收低频末端目标，生成更高频的关节轨迹；论文给出的两个频率是 25 Hz 与 120 Hz，后续由机器人阻抗控制器执行。RGB 路径还使用额外的弱透视尺度估计；RGB-D 可使用深度定位。多视角处理包含初始化和置信度选择，不等同于我们之前讨论的双目关键点三角测量。论文也记录了快运动丢失、自遮挡失效。[S2，IV-A/B/D、VIII]

**边界：** 这些是论文中的系统描述，不等于所有细节和权重都在当前公开示例里，也不意味着 SO101 的位置舵机具有论文中机器人同样的控制能力。

## 3. 实际开源了什么

| 部分 | 本次核查结果 | 采用方式 |
|---|---|---|
| `dexsuite/dex-retargeting` | 源于 AnyTeleop；核心是人手到机器人手的优化重映射 [S3] | 灵巧手可直接研究；SO101 单自由度夹爪不必强行跑完整优化 |
| `example/vector_retargeting/single_hand_detector.py` | MediaPipe 包装、局部 21 点和腕部旋转；没有完整全局腕部平移 [S4] | 保留未改动基线；通过明确适配器接入 |
| `NVlabs/sim-web-visualizer` | 项目页链接的浏览器可视化代码 [S1] | 可参考，但不是机械臂控制器 |
| AnyTeleop 完整 arm-hand 相机到真机系统 | 本次检查的官方链接和当前仓库中，未找到能直接复现整套效果的完整实现 | 不以“已经开源完整系统”为工程前提 |
| 独立 `NVlabs/curobo` | 提供机器人模型、运动学、碰撞和运动生成能力 [S7] | 可作为后续真实运动后端，不是装上就自动支持 SO101 |

当前仓库 issue #27、#73 仍有人询问机械臂控制代码；#62 中作者 Yuzhe Qin 推荐 BunnyVisionPro 的 arm control 示例。这是“应补 arm 控制集成”的直接线索，但不据此声称互联网任何地方都不存在相关代码。[S5]

## 4. 最容易误接的源码细节

审查文件：`single_hand_detector.py`，Git blob `4867ad7d2e5ea3984002c1730b937d06b1fe97ee`。[S4]

`detect()` 返回四项：检测数、`joint_pos`、二维关键点、`mediapipe_wrist_rot`。其中三维点已经减去了腕点，然后转到腕部/MANO 相关坐标。**腕点为零是坐标局部化的结果，不是“手在相机原点”。**

因此仅在这个函数后面加一个机械臂 IK，仍然缺少真正的相机空间平移。若把局部点直接当绝对坐标，换朝向和开合动作就可能错误地驱动末端移动。

本包 `decode_dex_local()` 的逆变换严格对应所审查的代码；它只恢复“相机朝向下的相对骨架”，没有凭空生成全局位置。

另一个必须检查的地方是左右手与自拍语义：上游包装在非自拍输入下会反转检测类别匹配。输入图像又自行翻转、BGR/RGB 搞反、缩放后不更新内参，都可能破坏基线。这些是**排查假设**，不是对旧项目失败原因的已证实判断。[S4]

## 5. 作者推荐的替代参考：BunnyVisionPro

作者在 #62 明确推荐该项目。它已公开 XArm7 和 Ability Hand 的真实控制示例，但输入是 Vision Pro，不是普通相机。可借鉴“全局手腕目标 → 机器人运动学/驱动”的组织方式，不需要为了借鉴控制层去购买头显。[S5、S6]

其当前公开 README 仍把碰撞避免和机械臂奇异性避免列为未完成项目。因此不能把这个仓库当作已经提供了论文级全部安全能力。[S6]

## 6. 适配 SO101 的另外一个直接参考

NVIDIA 的 SO101 retargeter 文档描述了 full-pose clutch、按下时重设参考点及五自由度姿态折中。它证明“相对位姿输入 + 末端 IK”适合这一机型，不代表我们已经运行过该集成。该系统以 XR 为输入，输入层与本项目不同。[S8]

本包采用**按实际机器人位置和朝向一起重新接管**的策略，是本项目明确的交互选择；不声称逐项复制了 NVIDIA 实现中的 home 保留策略。

## 7. cuRobo 版本问题不能忽略

核查时官方文档列出 2026 年的 v0.8 重写与 11 月 4 日 v0.9 更新。不能把 2023 论文、旧版 `MpcSolver` 教程和当前 API 随意拼起来。工程化必须固定具体版本、运行官方 reactive-control 示例、再建立 SO101 配置。[S7]

本包没有伪造一个未经运行的 cuRobo 后端。提供的是通用后端接口、CPU 位置优先伺服基线，以及明确的后续集成任务。

## 8. 对旧方案应怎样纠偏

最值得保留的是“末端相对移动”的产品目标。最应该改变的是工程顺序：先复现上游识别与坐标语义，再分别验证手腕全局定位和末端控制，最后集成。

换成 AnyTeleop 架构不自动解决识别率。其公开示例本身仍使用 MediaPipe。应以同一批完整原始视频做 A/B：上游识别率、适配后的识别率、有效空间定位率、控制暂停原因分别统计。没有旧代码和录像时，不应直接断定旧项目到底错在模型、相机角度还是控制映射。

## 2026-11-06 · D455 global-position implementation research

The user clarified the first required outcome: continuous arm following in the
intended direction; high metric precision is a subsequent improvement. Position
availability, correct-direction motion and precision must be measured separately.
The existing base-fixed relative mapper and position-priority IK already provide
the control structure. Constant position bias cancels at engagement, but
rotation-dependent bias and missing observations do not.

Sources and actual open-source code inspected:

- AnyTeleop §IV-A/B separates wrist-relative geometry from global wrist pose;
  RGB-only adds a scale estimator and RGB-D adds measured depth. Its public
  detector blob remains `4867ad7d2e5ea3984002c1730b937d06b1fe97ee`.
- [OAT accepted paper](https://eprints.whiterose.ac.uk/id/eprint/238695/1/ICRA26_0903_FI.pdf)
  estimates translation by matching projected model joints to image landmarks.
  The paper's real-detector results are centimetre-scale, and its ground-truth
  rows are not camera-accuracy evidence. No linked implementation was located
  in its paper/university record, despite its open-source description.
- [DexPilot](https://arxiv.org/html/1910.03135) fits an articulated surface using
  DART, supported by learned priors and multiple depth cameras. The inspected
  [DART kernel](https://github.com/tschmidt23/dart/blob/37adc66580a15c5b438986d77621ebd8f5f450c3/src/optimization/kernels/obsToMod.cu)
  associates observed depth vertices with model surfaces using robust losses.
  This is a relevant geometric precedent, not a tested modern D455 dependency.
- [DepthAI hand tracker](https://github.com/geaxgx/depthai_hand_tracker/blob/97731232fffd467e8de2c3a34ab8382962bb385e/HandTracker.py)
  obtains global XYZ from an adaptive wrist depth ROI, separately from local
  world landmarks. It requires OAK-specific APIs, so its architecture was
  studied without replacing the current detector or installing its stack.
- [Sridhar et al., ICCV 2013](https://www.cv-foundation.org/openaccess/content_iccv_2013/papers/Sridhar_Interactive_Markerless_Articulated_2013_ICCV_paper.pdf)
  uses robust palm-plane fitting as part of a larger calibrated tracking system.
  A surface fit requires adequate coverage, visibility and conditioning.
- [RealSense D400 datasheet](https://realsenseai.com/wp-content/uploads/2024/10/Intel-RealSense-D400-Series-Datasheet-October-2024.pdf)
  lists nominal D455 minimum depth of 320 mm at 640×480 and 260 mm at 640×360.
  The saved recording operates near that boundary. Nearer-range mode is a
  sensor-profile experiment, not proof that all missing depth will recover.

Independent full-recording reprojection comparisons retained all 900 attempts
and all 802 selected-hand detections. Translation-only fitting gave median/P95
reprojection RMS 38.8/58.2 px; SQPnP plus refinement gave 30.7/50.9 px. Numerical
convergence sometimes produced impossible/extreme depths. An exact-projection
synthetic case verified the translation solver algebra, so the observed mismatch
cannot be dismissed as a demonstrated solver-algebra error. These results do
not support filling missing depth with that monocular model on this recording.
Artifacts and pinned code details are in `outputs/position-research/`.

Dense whole-palm prototypes also failed to improve availability. The selected
implementation instead fits current local wrist-surface depth with explicit
association, local support, foreground-consistency and conditioning checks.
The control anchor remains the landmark-0 image ray, never the moving centroid
of available depth. Skin/landmark bias remains a limitation. Independent review
found a background-ring counterexample; acceptance now requires local depth
evidence agreeing with the fitted surface, not convex-hull containment alone.

The saved model orientations also contain abrupt changes. The implementation
therefore exposes a named `position` mode for the user's XYZ milestone, while
leaving `full_pose` behavior and its default unchanged. This is not an automatic
fallback, a threshold reduction, or an orientation offset applied after latching.
All position/identity/timing/collision/stop rules remain active.

The research does not establish physical following, absolute precision, or
reliability in every required pose. Full live D455 hand-motion and physical-arm
tests are still required; successful startup or synthetic geometry is not a
substitute.

## Recorded follow-up: semantic wrist errors and depth holes are separate

The complete first live recording now has an exact upstream raw-result replay,
Pose association analysis, instrumented Holistic comparison and unchanged-depth
A/B. All 2,857 attempts remain accounted for. The original 2,849 readable-frame
selected probes reproduce exactly; the terminal corrupt archive is retained
as unavailable rather than discarded.

Body context distinguishes the target-hand handedness flip from the distractor
hand occupying the original single-hand tracker. The installed Holistic graph
can expose all 21 local world points, but both internal hand-presence gates must
be explicitly set to 0.8: the public confidence arguments configure pose, not
those hand gates. The stock pose-derived hand crop loses many original hand
detections. This is consistent with the ROI failure described by
[Moryosseff, 2024](https://arxiv.org/html/2405.03545v2), whose crop-quality results
do not establish metric hand-position accuracy or a ready-made live fix.

With the unchanged depth estimator, readable-frame global positions were 1,906
for the baseline and 1,920 for Holistic (28 gains, 14 losses). A named offline
baseline-first hybrid produced 1,929, but recovered only four isolated depth
positions in the 34-frame natural palm-down interval. Its longest unavailable
span did not improve, and switching references at frame 2555 creates a 156 mm
step exceeding the existing motion guard. Neither experiment is promoted to
runtime. Full reports: `outputs/position-research/live-01-holistic.report.md`
and `live-01-hybrid.report.md`.

A separate image audit found the original wrist landmark on the proximal
forearm during phone handling, while the body-guided estimate stays near the
visible wrist. Both depth fits are supported by real current surfaces, yet the
two XYZ estimates differ by up to 179 mm. This is an estimator disagreement,
not an independently measured anatomical error. It demonstrates why a valid
surface fit cannot establish that the selected pixel is actually the wrist.
See `outputs/position-research/live-01-holistic.reference-difference.jpg`.
Future body/hand consistency work must preserve a stable physical reference;
blindly switching estimators can itself move the target.

The next live sensor comparison retains the original 640×480 color inference
while testing explicitly selected nearer-range depth profiles and recording
native depth/IR. Those profiles are prepared, not hardware-validated. Better
depth availability and correct wrist localization must both be demonstrated;
neither can be inferred from the other's confidence or availability count.

## Original sources

- [S1] 官方项目页：https://yzqin.github.io/anyteleop/
- [S2] 论文 v3：https://arxiv.org/html/2307.04577v3；RSS PDF：https://www.roboticsproceedings.org/rss19/p015.pdf
- [S3] 官方重映射仓库：https://github.com/dexsuite/dex-retargeting
- [S4] 审查文件：https://github.com/dexsuite/dex-retargeting/blob/main/example/vector_retargeting/single_hand_detector.py
- [S5] 官方仓库讨论：https://github.com/dexsuite/dex-retargeting/issues/62；另见 `/issues/27`、`/issues/73`
- [S6] 作者推荐的示例：https://github.com/Dingry/BunnyVisionPro
- [S7] cuRobo：https://nvlabs.github.io/curobo/latest/；Reactive Control：https://nvlabs.github.io/curobo/latest/getting-started/reactive_control.html；模型构建：https://nvlabs.github.io/curobo/latest/getting-started/build_robot_model.html
- [S8] SO101 末端映射参考：https://nvidia.github.io/IsaacCapture/main/references/retargeting/so101.html
- [S9] MediaPipe：https://developers.google.com/mediapipe/solutions/vision/hand_landmarker/python
- [S10] RealSense 投影：https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/
- [S11] OpenCV PnP：https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
- [S12] 官方 SO101 follower 模型：https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
