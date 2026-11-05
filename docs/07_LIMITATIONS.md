# 07 · 完成度与已知限制

## 运行时更新（2026-11-05）

`docs/08_RUNTIME.md` and `outputs/runtime/VALIDATION.md` supersede the implementation-status statements below. Explicit RealSense, PyBullet and Feetech-worker code now exists, including local command expiry. Full-model/device tests, measured calibration and physical stop acceptance remain outstanding. The following section records the original offline delivery's boundaries; it is not a claim that those adapters are still absent.

## 初始离线交付边界（历史记录）

核心能读取规范观测、计算目标、运行五轴数值控制、处理接管和异常、离线记录/重放。它不访问串口，也不声称自动恢复识别失败。测试和 replay 已运行，真实传感器与机械臂未运行。

## 具体限制

1. 识别器没有替换或训练。对上游 MediaPipe 的遮挡局限不会凭架构改变而消失。可执行 probe 未经本机摄像头测试。
2. RGB-D helper 假定输入已正确注册、正确去畸变/反投影并有可信可见性。没有实现 RealSense 实时驱动和遮挡推理。皮肤表面与骨骼模型存在偏差，模型尺度亦需评估。
3. 本包没有多视角融合，也没有原论文单目额外弱透视网络/SMPL-X 置信度模型。普通单目局部骨架不被当成有尺度的全局位置。
4. FK/解析器只用于支持的串联 fixed/revolute/prismatic 链。mimic、continuous、floating 明确拒绝，不静默近似。生产可换成熟运动学库，保持测试语义。
5. CPU 二级 IK 保证的是线性化任务优先，不是全局最优、完整姿态可达、无奇异或无碰撞。朝向残差必须显示。
6. 速度/加速度限制作用在离散数值候选上，不保证真实舵机响应、冲击、动力学/力矩或断电行为。停止分支没有实现物理减速。
7. 没有真实 TCP、driver 标定、完整 mesh、碰撞检测和 cuRobo 后端。运动学裁剪件不能作为硬件可用性依据。
8. 默认 full-pose 模式位置或朝向不可用时暂停。夹爪独立失效时保持；若要位置模式，应显式加模式及测试，不悄悄降级。
9. 本版命令年龄/跳变等阈值是参考配置，不是经过用户设备标定的最终门槛。实测前不能承诺识别率、精度或实时性。
10. `deployment.example.json` 目前是工程配置清单，不被运行时代码自动加载。它保留 null 字段是为了突出未标定部分，不是假装硬件配置已完成。
11. 本轮没有获取用户旧源码/全量失败录像。交付是重新组织路线和可复现基线，不是已证明修复了旧项目所有 bug。

## 明确不做的承诺

不会宣称“可运行参考代码=已完成 AnyTeleop 全系统复现”；不会把这次合成回放的误差当真机毫米精度；不会把关节限位叫避障；不会把源码能 import 叫相机识别已验证；不会把整体检测成功率隐藏各关键姿态失败。
