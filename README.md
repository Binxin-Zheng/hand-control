# AnyTeleop 路线 → SO101：调研、设计与离线核心交付

**离线核心 + 显式运行时适配 · 2026-11-05**

这不是 AnyTeleop 官方完整系统，也不是已验证的真机产品。它是基于论文、官方项目页和实际源码审查整理的独立实现交付包。核心代码可以离线运行；导入、测试和仿真不会访问电机。新增的 RealSense/PyBullet/Feetech 运行时均需显式启动；真机仍需实测标定与安全验收。

## 当前运行入口

See [runtime setup and commands](docs/08_RUNTIME.md) for the official-model downloader, synthetic-input simulator, RGB-D worker, and explicitly gated hardware worker. Actual test evidence and remaining integration limits: [runtime validation](outputs/runtime/VALIDATION.md).

```bash
# Run in a terminal with GitHub access. Keep the nested mesh directory.
python3 tools/fetch_so101_model.py --out assets/so101
# With simulator dependencies installed (see runtime guide):
PYTHONPATH=src .venv-core/bin/python tools/check_model.py
PYTHONPATH=src .venv-core/bin/python tools/run_teleop.py --synthetic
```

The simulator starts released: **E** engage, **R** release, **X** software E-stop, **C** reset while released, **Q** quit. This does not operate the physical arm. The runtime paths have tests with artificial geometry/fake motors; official SO101 dynamics, real camera capture, and physical stopping are pending local verification.

## 结论先读

AnyTeleop 的 “Arm Controls Arm” 是**人的手腕位姿 → 机器人末端目标 → 机械臂运动生成**，不是复制人的肩、肘关节角。现有 dex-retargeting 手部示例不等于完整 arm-hand 系统。我们要补的是**固定空间中的手腕位置、末端控制映射、运动生成、机器人适配与安全闭环**。

建议主线：保留上游完整 21 点识别作为基线；RGB-D 补充有尺度的手腕位置；手腕控制末端；手指角度控制单自由度夹爪；五轴 SO101 采用位置优先、姿态尽量跟随。不要再把两条指线当成整个手部姿态检测器，也不要把手的 XYZ 直接变成几个电机角度。

## 先看哪些文件

| 文件 | 作用 |
|---|---|
| `docs/01_RESEARCH.md` | AnyTeleop 实现原理、确切开源边界、源码核查、可借鉴项目 |
| `docs/02_ARCHITECTURE.md` | 模块划分、坐标系、交互状态机、进程与控制环 |
| `docs/03_ALGORITHMS.md` | 手腕定位、位姿映射、夹爪映射、位置优先 IK 的数学定义 |
| `docs/04_CODEX_HANDOFF.md` | Codex 接手顺序、任务边界、不可擅改的约束 |
| `docs/05_ACCEPTANCE.md` | 软件测试结果、完整实拍评估和真机放行条件 |
| `docs/06_ENVIRONMENT.md` | 隔离环境、上游依赖与版本管理 |
| `docs/07_LIMITATIONS.md` | 真实完成度、已知限制、必须继续验证的部分 |
| `AGENTS.md` | 给编码代理的仓库级规则 |
| `SOURCES.json` | 检索来源、审查日期、上游文件 Git blob 标识 |

## 已实现与未实现

**已实现并测试：** SE(3) 坐标工具；官方 detector 返回值的坐标解码；有效深度的掌部平移融合；三维刚体配准工具；手指夹角和开合标定；相对位姿映射；夹爪软接管；URDF 串联链 FK/Jacobian；受关节速度、加速度和位置约束的位置优先微分 IK；追踪丢失/身份改变/过期数据/急停/重新接管状态机；严格 JSON 输入；离线生成与确定性回放。

**新增运行时代码、尚待设备验收：** RealSense RGB-D 输入、PyBullet 全模型仿真与采样碰撞守卫、Feetech 原始单位标定边界、独立命令过期 worker、键盘接管控制。**尚未完成：** 真实 TCP/关节标定、物理停止测试、完整官方模型与真实相机联调、多相机融合、cuRobo、正式 UI。

**特别说明：** `assets/so101_kinematics_only.urdf` 是官方 SO101 follower 文件的运动学裁剪件，保留关节几何和限位。它没有 mesh/collision/inertia。测试端点是 `gripper_link` 原点，不是假称已校准的抓取 TCP。

## 初始离线交付验证（历史记录）

**72 项自动化测试通过；1,200 个合成控制周期运行并确定性回放通过。** 摄像头、真机、cuRobo 和碰撞检测尚未验证。详细结果与边界见 `outputs/VALIDATION.md`。

## 离线运行

在项目根目录，使用独立 Python 3.11+ 环境：

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-core-tested.txt
python -m pip install -e . --no-deps
python -m pytest -q
python examples/offline_demo.py --out outputs/demo
python examples/replay.py outputs/demo/trace.jsonl
```

Python 版本命令应替换为本机实际安装的 3.11+ 解释器。核心验证实际运行环境是 Python 3.13.5；这里不声称对每个 Python/OS 组合都已验证。不需要 GPU、MediaPipe、RealSense、ROS、LeRobot。

包内已含本次生成的 `outputs/demo/metrics.json` 和完整 `trace.jsonl`，方便首次接手直接检查。测试细节见 `outputs/VALIDATION.md`。合成轨迹数值不能作为摄像头精度、识别率或真机表现。

## 开发入口

把 `docs/04_CODEX_HANDOFF.md` 末尾的任务提示交给 Codex。**第一项工程工作是复现未改动的上游识别基线并录制完整失败片段，不是连上电机试跑。** 本交付没有拿到旧项目源码和完整失败录像，因此没有给旧项目做逐行根因诊断。

## 实拍视频接入进展（2026-11-05）

已用未改动、Git blob 校验一致的上游 detector 完整处理 `sample/output.mp4`（326 帧），新增视频时间戳、21 点标注视频、显式 RGB-D 观测适配器及实拍安全回放。77 项测试及 1,200 周期合成确定性回放通过。完整结果、复现命令、环境和未完成项见 [实拍验证记录](outputs/video/VALIDATION.md)。

查看 [右手标注视频](outputs/video/right.mp4)。全片保留无手空闲时间，检测计数不代表有手帧召回率。RGB 视频没有可直接使用的全局深度，因此安全回放不发运动命令。**尚未完成机械臂运动仿真或真机手控验收**；需要完整 follower meshes、有效全局腕部位置输入及硬件放行条件。
