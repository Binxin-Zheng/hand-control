# 05 · 验证与验收

## Current user priority (2026-11-06)

The required first outcome is a physical SO101 end effector that follows hand
movement reliably in the intended directions. Improved small-motion precision
is the next objective. A camera startup, isolated valid XYZ frame, replay, or
synthetic simulator success does not complete that outcome. Preserve the user's
natural fingers-down, rotation and open/close pose range and all failed frames.

The current explicit XYZ experiment uses depth-supported wrist-surface position,
the unchanged direction mapping/gain, and position mode with the measured
engagement orientation as a secondary target. Validate all six translation
directions, stationary holds, rotation/open-close coupling, interruptions,
delay, stop and measured-state re-engagement end-to-end. The existing proposed
precision thresholds below remain visible; they are not claimed achieved or
silently weakened. A basic-following milestone must be identified separately
from precision acceptance.

## 1. 本次实际运行了什么

当前 `outputs/VALIDATION.md` 是最终运行记录，以该文件和 JUnit 为准。数值测试包括坐标变换、左右手适配数学、深度离群值、开合不随全手旋转变化、FK/Jacobian 有限差分、受约束微分 IK、重新接管零跳变、夹爪软接管、数据过期/身份变化/急停。

连续回放使用 1,200 个控制周期，合成视觉 25 Hz、数值控制 100 Hz。故意包含丢失、恢复但仍按住、明确释放重接管、急停与复位。每条输入和输出保存在 JSONL，已再次回放检查一致性。

**不属于本次实测：** 真摄像头识别率、真实深度误差、真实 TCP 精度、实际机器人可达范围、碰撞、动态力学、电机频率、真实端到端延迟。离线理想执行器的跟随误差不能填到上述项目里。

## 2. A/B 评估必须保存分母

对同一份完整录像同时运行原版 detector 与新系统。统计原始帧总数、上游有手帧数、全局位置可用数、朝向可用数、夹爪可用数、控制可用数、每类拒绝原因和最长中断时长。

检测率与空间有效率分别报告。`predicted` 和由历史维持的数据不计入当前观测有效率。不能先删掉失败帧再计算成功率。

## 3. 建议的第一版指标（待设备实测确认，不是已达成承诺）

| 维度 | 建议验收目标 | 测量方式 |
|---|---|---|
| 必需姿态可用率 | 每种必需姿态单独达到 95%，不能只报整体平均 | 完整会话统计；包括指尖向下、张合、侧转、前后移动 |
| 空间位移 | 10/20/50 mm 已知位移测试；误差 P95 不超过 5 mm | 外部滑轨/标记夹具或独立计量，不仅使用模型自证 |
| 只转手不平移 | 参考点伪平移 P95 不超过 5 mm | 固定旋转中心的夹具或独立追踪 |
| 开合解耦 | 拇指开合时腕部参考点不超上述容差 | 固定掌部完整录像 |
| 接管 | 首帧目标与接管实测位姿一致；不得自动张开夹爪 | 日志断言 + 仿真 + 低速真实验证 |
| 延迟 | 端到端 P95 目标不超过 150 ms | 采集时间、处理时间、发送/反馈及外部视频核对 |
| 异常恢复 | 丢失后不补跑、不自动复位；释放重接管后无突跳 | 故障注入、断线、相机重启、身份切换 |
| 真实动作 | 抓取—搬运—放置至少 20 个完整回合，公开成功率及失败原因 | 所有回合保留，不只挑成功视频 |

这些门槛用于立项讨论和实测设计，不是以门槛替代结果。若不满足，先定位误差来源，再由负责人明确改变范围/硬件/指标，禁止编码代理为了“通过”自行放宽。

## 4. 真机之前的硬性放行条件

完整 follower URDF 和 mesh、真实固定 TCP、关节名/单位/符号/零位、实际机械限位、活动指几何、全连杆/自碰撞检查、桌面/底座环境模型、命令有效期、独立本地 watchdog、软件停止、通信失败停止策略均已验证。本项目是有人监督的实验室 demo，按用户要求不以安装物理急停为放行前提。

确认同一串口只有一个所有者；UI 卡死和相机退出不能让机械臂继续追逐旧目标。若这些条件缺一项，交付只能标为离线/仿真，不得改名“可安全真机运行”。

## 5. 验收姿态集

保留用户自然的指尖向下姿态；测试同一姿态下张合和移动，避免只在摊平张开的手上验收。覆盖左右旋转、倾斜、侧面、局部遮挡、距离边缘、速度变化和恢复过程。若单视角确实不可观测，应报告并评估额外视角，而不是让用户长期保持不自然姿势。
