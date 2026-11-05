# 最终验证记录

版本：0.1.0；验证日期：2026-11-05。

## 本次实际执行

| 项目 | 结果 |
|---|---|
| 自动化数值 / 协议 / 状态机测试 | **72 passed**；0 failures，0 errors |
| Python 编译检查 | `src`、`examples`、`tools` 全部通过 |
| 合成理想执行器连续运行 | **1,200 个控制周期**，25 Hz 合成观察 / 100 Hz 数值控制时钟 |
| 确定性回放 | **1,200 条决策全部复现**，校验状态、原因、关节及开合指令 |
| 摄像头 / MediaPipe 实测 | **未执行** |
| 真机 / 电机 / 物理动力学仿真 | **未执行** |
| cuRobo / GPU / 全连杆避障 | **未执行，后端未实现** |

测试环境：Python 3.13.5，NumPy 2.3.5，SciPy 1.17.0，pytest 9.0.2。`OPENBLAS_NUM_THREADS=1` 用于限制本次计算环境的线程数；不代表在用户设备上验证过实时频率。

## 可复现命令

独立环境安装后，在仓库根目录执行：

```bash
python -m pytest -q --junitxml=outputs/pytest-junit.xml
python -m compileall -q src examples tools
python examples/offline_demo.py --out outputs/demo
python examples/replay.py outputs/demo/trace.jsonl
```

本次运行时使用 `PYTHONPATH=src`，与安装 editable package 后导入同一份源代码等效。仓库中保留 pytest stdout、JUnit、demo stdout、replay stdout、metrics.json 和完整输入/输出轨迹。

## 合成运行内容与解释

状态计数：ACTIVE=1100，PAUSED=40，IDLE=40，ESTOP=20。脚本注入了追踪中断、显式释放后重新接管、急停及复位事件。

重新接管时最大关节目标跳变为 0.0 rad。次级姿态任务对主位置任务的线性速度泄漏最大为 1.766e-18 m/s。

日志中的位置跟随误差 P95=3.602 mm，**只是合成、理想执行器、特定轨迹、gripper_link 原点的数值结果；不是实际相机精度、机器人精度或 TCP 定位精度，不能用它对外宣称毫米级遥控。** 该轨迹以已知运动学生成，因此不构成独立的机器人标定验证。

本次 1,200 周期计算 wall time 约 16.99s，仅是本容器的一次运行，不是实时性能验收。

`assets/so101_kinematics_only.urdf` 无 mesh/collision/inertia。这里测试的端点是 `gripper_link` 原点，没有谎称抓夹中心已经标定。候选关节目标没有发送到任何设备。

## 已覆盖的关键边界

坐标系旋转与反射拒绝；左右手上游局部坐标逆变换；越界像素不裁剪成合法深度；深度空洞/混合表面/离群点/退化掌面；全手旋转时夹角不变；只动拇指不改变掌部平移；FK/Jacobian 有限差分一致性；关节位置/速度/加速度数值约束；姿态任务不改变主位置速度；身份变化、倒序数据、过期数据、主机时钟回退、错误 frame、预测冒充观测、夹爪软接管和急停锁存。

**软件测试通过，只能证明这些输入条件下实现满足断言；不能代替完整原始录像和真机测试。** 后续放行要求见 `docs/05_ACCEPTANCE.md`。
