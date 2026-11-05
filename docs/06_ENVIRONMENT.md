# 06 · 环境与依赖隔离

## 核心环境

核心仅依赖 NumPy 与 SciPy，pytest 用于测试。`requirements-core-tested.txt` 记录本次真实使用的版本；`pyproject.toml` 给出 API 兼容范围。pip resolver 通过不等于所有版本已验证。应在本机验证后提交实际锁文件。

推荐 Python 3.11+ 的独立 venv 或新的 conda 环境，不使用系统 Python 安装全套机器人依赖，不修改 conda base。环境中使用 `python -m pip`，执行 `python -m pip check`。命令见 README。

本次验证实际为 Python 3.13.5 / NumPy 2.3.5 / SciPy 1.17.0 / pytest 9.0.2。测试运行环境不包含已验证的摄像头/GPU/ROS 组合。

## 视觉环境

上游示例使用 legacy `mp.solutions.hands`。不要在没有验证的情况下换成另一套 MediaPipe Tasks API 后继续称为“原版复现”。先使用与本机已跑通上游相同的独立环境、记录 `pip freeze` 和源码版本，再做 API 迁移 A/B。

当前上游 README 的 NumPy/MediaPipe 说明不能代替本机依赖解析。若 resolver 冲突，应选择可共存的明确版本，或拆进程；不使用 `--no-deps` 强行装一堆冲突包、不忽略错误。核心本身不需要导入 MediaPipe。

`examples/upstream_probe.py` 故意不依赖核心库，可以直接放在上游环境运行。它不控制机器人。全局平移字段明确标为不可用。

## 运动 GPU 环境

cuRobo 单独环境/容器，固定版本与 GPU 架构支持，按所选版本的官方文档安装。当前 API 已不同于早期教程，先跑官方最小 reactive-control，再写适配。不要因本项目改动系统显卡驱动、CUDA 或其他已有机器人环境。必要的系统变更必须单独说明影响并获得授权。

## 机器人环境

尽量复用用户已稳定的 SO101/LeRobot/ROS 控制环境。用本地消息协议与 core 交互，不要求把 MediaPipe、cuRobo、ROS、LeRobot 全装进一个 Python 环境。机器人进程独占串口。

## 源码获取

`tools/fetch_audited_files.py` 可下载两个审查过的精确 Git blob 并验证 SHA。该可选脚本本次未执行网络下载，不影响离线测试。它不下载 mesh、不安装软件、不改变环境。完整可视化/碰撞模型仍需从官方仓库取全。

```bash
python tools/fetch_audited_files.py so101 --out vendor_audit
python tools/fetch_audited_files.py detector --out vendor_audit
```

README/模型 LICENSE 应随实际再分发保留。核心源码许可与裁剪模型的第三方许可见根目录 LICENSE 和 THIRD_PARTY_NOTICES.md。
