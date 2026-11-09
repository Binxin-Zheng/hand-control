# 03 · 算法定义与实现边界

这是可直接指导工程化的本项目算法定义。与原论文不同之处均在下文标明。

## 1. 手部识别不重写

首先直接运行所审查的上游 detector，并保留完整 21 个点。四指并拢与拇指的两线抽象放在后处理/UI，不删模型输入输出、不用两条指线取代腕部朝向估计。

上游输出采用行向量处理：

`joint_pos = (world_points - world_points[0]) @ wrist_R @ operator2mano`

因此本包还原：

`relative_camera_model = joint_pos @ operator2mano.T @ wrist_R.T`

得到的是相机朝向下的**相对骨架**。`relative_camera_model[0] == 0`，没有全局位置。`wrist_R` 保留为相机参考中的模型朝向，使用前必须固定版本和左右手/镜像约定。[源码依据：01_RESEARCH S4]

## 2. RGB-D 全局腕部平移：参考实现

### 2.1 像素到三维

在去畸变、深度已经对齐的像素网格上：

`X=(u-cx)*Z/fx; Y=(v-cy)*Z/fy`。

`RectifiedIntrinsics` 只接受这个假设。真实 RealSense 有自己的畸变模型、depth scale、彩色与深度外参；工程实现应优先使用驱动/SDK 的正确反投影和流配置，不能把本工具套在未经校正的原图上。[S10]

深度区域采样先排除空洞、越界、混合深度边缘与不可信可见性。深度单位必须转换到米。`depth_patch_median()` 是一致性工具，不会识别被手掌遮住的关节。

### 2.2 平移估计

优先取掌部点 `0,5,9,13,17`。对每个确认有有效深度对应的点，令：

`candidate_t[j] = observed_surface_point_C[j] - relative_camera_model[j]`。

取候选平移的中位数，按残差剔除离群值，要求至少三个有效掌点覆盖非共线区域。实现：`wrist_translation_from_depth()`。这使拇指开合不会直接改变腕部位置锚点，也不强求每个指尖都有有效深度。

**重要限制：** 深度看到的是皮肤表面，模型点近似的是骨骼位置；模型手尺寸也不等于实测尺寸。因此该算法有系统偏差，尤其旋转时偏差可能变化。它是可验证的 RGB-D 参考融合器，不是已经证明毫米级的真实腕骨定位器。需用完整录制数据标定/评估表面偏置、尺度和旋转引起的伪平移；必要时使用可见表面模型拟合，不可仅靠放宽残差门限。

### 2.3 朝向策略

第一条 A/B 主线保留上游 RGB 模型朝向，深度只修正平移。这是本项目为了减少对识别基线的改动而作的设计，不宣称是原论文逐行实现。可选的 `rigid_fit_3d()` 用于可信的 3D—3D 配准比较；不在没有实测证据时切换默认路径。

论文在 RGB-D 段落中同时提到三维对应点和 PnP；工程上必须区分：**PnP 是 3D—2D 配准**，**Kabsch/刚体配准是 3D—3D 配准**。不能把两个三维点集直接传给 `solvePnP`。本包没有照搬这一表述上的歧义。[S2、S11]

## 3. 手腕 → 末端增量映射

### 2026-11-06: explicit `wrist_surface_fit` alternative

The new alternative estimates a **visible wrist surface**, not an anatomical
joint. It preserves the original detector, all 21 points, RGB preprocessing and
both earlier estimators. Landmark 0 supplies a fixed image ray. Bounded local
neighborhoods around that ray supply current-frame depth patches. Patch support
still requires 80% valid samples and a 15 mm depth spread limit. Depth-connected
components separate nearby background; robust surface fitting uses the existing
12 mm residual limit, at least six patches and 3 mm two-dimensional spatial
coverage. The ray must be locally supported after rejection, not merely inside
the convex hull of distant background. Current near-ray depth must agree with
the fitted surface. Excessive holes, conflicting surfaces and grazing views
produce unavailable position; no previous depth is substituted.

For a surface `n·p=d` and SDK-deprojected wrist ray `r`, the reference is
`p=r*d/(n·r)`. The current SDK intrinsics/distortion are retained. Output records
the method version, `wrist_surface_ray` reference, patches, residuals and current
depth provenance. The geometric wrist ROI is explicitly an association
assumption, not a segmentation or anatomical-visibility claim. The separate
support mask is honored; an empty mask cannot establish observed hand position.
The local demo rejects depth outside 0.05–2 m, including implausible saturated
depth, without clipping values or removing frames from evaluation.

This construction avoids subtracting model bone positions from skin points.
It still depends on the 2D wrist estimate and can have rotation-dependent skin
bias. Small fitting residuals do not establish anatomical or millimetre accuracy.
The original `model_translation` remains the low-level worker default. The
explicit `tools/hand_follow.py` experiment selects the new estimator and the
D455 640×360 depth mode while retaining 640×480 color input.

### Explicit XYZ control mode

`TeleopSession(control_mode='position')` / `--control-mode position` follows
the same base-fixed position deltas and uses the measured robot orientation at
engagement as the secondary IK target. Five-joint hardware cannot guarantee
exact orientation holding. Both anchors are renewed from measured state on
re-engagement. This named mode does not use model orientation evidence or its
angular-jump gate; position evidence, position-jump, age, identity, backlog,
collision, stop and release/re-engage rules remain in force. `full_pose` remains
the default and retains its original orientation requirements. There is no
automatic switch between these modes.

接管瞬间：人手 `(p_H0, R_H0)`，实测机器人末端 `(p_G0, R_G0)`。固定坐标旋转为 `C=R_B_C`，平移倍率 `s`。

`p_G_des = p_G0 + s*C@(p_H - p_H0)`

`R_G_des = C@(R_H@R_H0.T)@C.T@R_G0`

本包 `RelativeMapper` 实现这两个公式。

首帧 `p_H=p_H0`、`R_H=R_H0`，结果严格等于机器人接管状态。旋转差在固定参考系左乘；没有反复相加的欧拉角，也没有把当前位置差旋进当前手掌局部系。平移 `s=1` 才是 1 cm 对应 1 cm；倍率不偷偷影响旋转。

此处控制点可以是刚性 TCP。若定义两指尖中点，它随单侧活动指开合变化，需要显式使用 `T_tip_TCP(gripper)`；本版不会假装它是固定偏移。默认测试用 `gripper_link` 原点，只用于运动学验证。

## 4. 夹爪映射

完整关键点中，四指方向分别使用 MCP→TIP，取单位向量的平均并归一化。拇指方向采用点 2→4。开合量为：

`alpha = acos(clamp(dot(fixed_fingers_direction, thumb_direction), -1, 1))`。

这对整只手的刚性旋转不敏感。四指要保持并拢相对稳定；拇指轴向扭转仍是这个手势的歧义，不能声称被一个角度完全解决。

标定舒适闭合与张开角：`a=clip((alpha-alpha_closed)/(alpha_open-alpha_closed),0,1)`。本包统一 **0 闭、1 开**，与某些第三方 API 的 closedness 符号相反；由驱动适配。

该默认是舒适范围映射，不是严格手角=机器人电机角。需要几何一度对一度时应另选显式模式并标定机器人夹爪几何；不得混称。

夹爪在接管时先保持当前实测开度。用户开度接近或跨过它时才解锁跟随，再按最大变化率运动。跟踪失败保持，不自动打开。

## 5. 机械臂：位置优先微分 IK

五个关节为 `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll`。`gripper` 独立，不用于给机械臂补第六个位姿自由度。[官方模型 S12]

令 `Jp` 为 3×5 位置 Jacobian，`Jr` 为 3×5 角速度 Jacobian。目标速度：

`v*=cap(kp*(p_des-p_actual), v_max)`

`omega*=cap(kr*Log(R_des@R_actual.T), omega_max)`。

### 一级：受约束的位置速度

求 `v1 = argmin ||Jp*v-v*||² + lambda²||v||²`，并施加关节速度、加速度以及下一步软限位边界。用 `scipy.optimize.lsq_linear` 实现。

阻尼随最小奇异值降低而增大。它减轻奇异附近的不稳定，但不保证穿越所有奇异点，也不实现奇异回避规划。

### 二级：不破坏一级线性任务的姿态跟随

计算 `N=null_space(Jp)`，限定 `v=v1+N*z`，再在相同边界下优化姿态速度误差。

由于 `Jp*N=0`，姿态优化不改变一级的**线性化位置速度**。实际一步积分的 FK 仍会有二阶误差，所以输出实际位置/姿态残差并由监督层判断。

这不是把位置权重随意设大后声称“严格优先”，也不是承诺不可达的全六维姿态能精确跟随。`primary_leak_m_s` 用于测试二级任务是否污染一级。

### 每步输出

`q_next=q_measured+dt*v`。必须基于当前实测状态，而不是不断叠加旧的未实现命令。超出规定 dt、软限位或数值失败时拒绝输出。CPU 参考实现不提供碰撞检测、力矩约束、动力学或硬实时保证。

## 6. 防止目标积压

即使位置目标始终相对最初参考点计算，也可能落到机械臂无法到达的位置。若目标与实际位置差超过设定阈值，进入 PAUSED，并要求重新接管。用户不应把手往回拉很久，机器人却还在追赶积压的目标。

本版采用明确暂停，而非偷偷改变倍率/参考点；后续可设计带用户反馈的工作空间投影，但需单独验证。

## 7. 真正接近 AnyTeleop 运动生成能力时，还缺什么

运动后端需支持：实时目标刷新、上一时刻解的 warm start、全连杆/自碰撞、真实桌面模型、关节约束、末端跟踪和未达目标诊断。cuRobo 的 reactive-control 路线比逐帧重新调用一次完整离线规划更贴合这个任务。[S7]

工程化应先在官方全模型上验证 CPU 基线，再决定引入 cuRobo 的实际价值。选择 GPU 后端不免除 SO101 欠驱动、伺服器标定、碰撞模型、夹爪几何和驱动 watchdog；也不能把原论文阻抗控制接口直接当成 SO101 舵机接口。
