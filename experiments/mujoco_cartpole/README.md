# MuJoCo 小车倒立摆：LQR 控制实验

与参考图相似的侧视环境：深色背景、紫色小车、细杆、橙色摆球、导轨刻度、
金色目标位置虚线和水平力箭头。物理由 MuJoCo 推进，窗口使用 GLFW + MuJoCo 渲染。

**默认启用离散 LQR 控制。** 开始后小车会移向目标位置，摆杆保持竖直向上。
控制器完整实现在 `controller.cc`，可以直接修改 Q、R，比较不同权重的效果。

## 构建与运行

在包含 `algorithm/` 的工作区根目录执行，沿用项目现有 MuJoCo、Eigen、GLFW、OpenGL 依赖：

```bash
cmake -S algorithm -B build -DMUJOCO_ROOT=/opt/mujoco
cmake --build build --target mujoco_cartpole mujoco_cartpole_verify -j
./build/experiments/mujoco_cartpole/mujoco_cartpole
```

窗口初始暂停，先显示 `x = 0.2 m, theta = -0.05 rad` 的初始姿态。

| 操作 | 效果 |
|---|---|
| 空格 | 开始 / 暂停 |
| R | 重置模型与控制器，并暂停 |
| N | 暂停时前进一个物理步（默认 2 ms） |
| 按住左 / 右方向键 | 在控制器输出上叠加 −2 / +2 N，用于手动控制或扰动实验 |
| Esc / 关闭窗口 | 退出 |

固定正交侧视相机，屏幕右侧为世界坐标 +X，上方为 +Z。HUD 显示状态与实际限幅后的力。
暂停时不施加新的输入，箭头显示上一仿真步使用的力。默认力为零时不画箭头。
小车驶出画面会提示按 R 重置，不会自动传送或修改物理状态。

无图形环境可运行，输出 CSV 到标准输出：

```bash
./build/experiments/mujoco_cartpole/mujoco_cartpole --headless --seconds 5
./build/experiments/mujoco_cartpole/mujoco_cartpole --headless --seconds 2 --force 1
./build/experiments/mujoco_cartpole/mujoco_cartpole --target 0.3
./build/experiments/mujoco_cartpole/mujoco_cartpole_verify
```

`--force` 是叠加在控制器上的恒定水平力，仅用于无窗口模式；`--seconds` 也仅用于无窗口模式。
无窗口仿真以完整物理步推进，时长取最接近的步数，默认每 50 步打印一次，并包含初末状态。
CSV 的 `force_N` 是上一物理步施加的限幅后输入（第一行初始化为零）。
`--target` 改变 LQR 跟踪的参考位置与虚线，默认回到原点。
模型复制至构建目录，因此程序可从任意工作目录启动。修改 XML 后重新构建即可同步。

## 控制器接口

从 `controller.hh` / `controller.cc` 开始：

```cpp
double Controller::compute(State const& state,
                           State const& reference,
                           double time,
                           double dt);
```

| 索引 | 状态 | 单位 / 约定 |
|---|---|---|
| 0 | 小车位置 x | m，向右为正 |
| 1 | 小车速度 x_dot | m/s |
| 2 | 摆角 theta | rad，竖直向上为零，向右倾斜为正 |
| 3 | 摆角速度 theta_dot | rad/s，绕 +Y 轴 |

`reference = [target_x, 0, 0, 0]`，`time` 是仿真秒数，`dt` 是物理步长。
返回值是小车的水平力，单位 N，向右为正。小车电机 `gear = 1`，没有摆杆电机。
仿真先叠加手动输入，再按 XML 中的范围限幅（默认 ±10 N）。
非有限力会明确报错。状态中的摆角保持 MuJoCo 的连续角度，不做 ±π 折返。

注意：**接口状态顺序与 MuJoCo 原始数组不同**。原始 `qpos = [x, theta]`，
`qvel = [x_dot, theta_dot]`。构造 A、B、Q 或增益时请保持接口的状态顺序一致。

控制器每个物理步调用一次，该步内输入保持不变；RK4 的内部阶段不会重复调用控制器。
当前实现是离散 LQR，启动时按模型的 `dt` 计算增益，每步仅做状态反馈。

## LQR 实现与调参

`Controller::initialize(model)` 在独立的 MuJoCo 状态上计算，不改变仿真初始姿态：

1. 在竖直静止平衡点、零输入附近，对实际 `mj_step` 一步映射做中心差分
   （扰动量 `1e-6`），得到 `A_d` 和 `B_d`。这包含当前 XML 的质量、惯量、
   阻尼、步长与积分器；不是忽略摆球惯量的质点近似。
2. 设 `Q = diag(25, 2, 120, 4)`、`R = 0.3`，迭代求解离散代数 Riccati 方程，
   相对收敛阈值为 `1e-12`，并检查 `A_d - B_d K` 的特征值模小于 1。
3. 每步执行 `u = -K (state - reference)`，仿真层叠加手动扰动并限幅。

默认模型得到的增益约为：

```text
K = [-8.996175, -10.586576, -68.858572, -17.600921]
```

这里 K 的负号与当前摆角定义有关，控制律仍为 `u = -K e`。
Q 的四个对角项分别惩罚位置、速度、摆角和角速度误差；R 惩罚控制力。
在 `controller.cc` 中修改权重后重新编译；修改 XML 后重新构建，程序下次启动会重新计算增益。
按 R 只重置状态，保留已经计算的增益。当前控制器没有积分项或其他历史状态。

默认初态在约 2.1 秒后同时满足位置误差 < 5 mm、速度 < 0.01 m/s、
摆角 < 0.005 rad、角速度 < 0.01 rad/s，并保持至 8 秒验证结束。
实测默认初态的最大控制力约 1.64 N。
按住方向键短暂扰动后松开，可观察扶正和回位；持续恒力可能留下位置稳态误差，
因为此版本没有积分补偿。控制针对竖直附近，不包含倒下后的自动起摆。

MuJoCo 一步映射及有限差分背景见
[官方 API 文档](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mjd-transitionfd)。

## 物理参数与边界

参数集中在 `cartpole.xml`：

| 参数 | 默认值 |
|---|---|
| 小车总质量 | 1 kg（包含装饰车轮） |
| 摆球质量 | 0.15 kg |
| 铰点到摆球质心距离 | 0.7 m |
| 摆球半径 | 0.05 m |
| 摆球质心转动惯量 | 每轴 0.00015 kg·m²，即实心球 `2mr²/5` |
| 重力加速度 | 9.81 m/s² |
| 阻尼 / 摩擦 / 电机附加惯量 | 0 |
| 时间步长 / 积分器 | 0.002 s / RK4 |
| 初始状态 | `[0.2, 0, -0.05, 0]` |

采用无质量刚性杆和有转动惯量的刚性摆球，动力学不是均匀杆模型。
XML 使用显式 `inertial`，装饰几何不会额外贡献质量。
线性化时如将摆球近似为质点，会与本模型存在小的惯量差别。

小车只有一个水平滑动自由度，摆杆只有一个绕 Y 轴的转动自由度。
车轮仅是装饰，不模拟滚动。导轨仅是标尺，没有机械端挡；所有接触关闭，
摆杆倒下后可以穿过小车或导轨并自由旋转。这样保留理想倒立摆动力学，
方便先在竖直平衡点附近实验。LQR 的局部稳定与大角度起摆是不同任务。

## 文件与验证

- `cartpole.xml`：模型、视觉样式、物理参数、初始姿态。
- `controller.hh` / `controller.cc`：模型线性化、Riccati 求解和 LQR 状态反馈。
- `simulation.hh` / `simulation.cc`：状态映射、控制调用、限幅、步进和重置。
- `viewer.cc`：交互窗口、刻度、目标虚线和力箭头。
- `main.cc`：命令行和无窗口运行。
- `verify.cc`：动力学检查，包括竖直平衡、重力失稳、正向施力、执行器限幅、
  无输入能量守恒、状态重置和非法输入拒绝。动力学部分直接调用 MuJoCo，
  不依赖控制器输出；另有默认初态、正负目标跟踪、镜像初态、±2 N 短时扰动恢复、
  力限幅及控制器重置的闭环检查。

MJCF 字段说明参考 [MuJoCo 官方 XML 文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html)。
