# RDrive / moteus 三环 FOC 仿真环境

该目录根据 `source_files/dummyx2_rdrivec1/moteus/fw/` 和其测试目录中的电机模型搭建，控制结构为：

```text
位置轨迹 -> 位置 P 环 -> 速度 PI 环 -> Iq 给定
                                      |
Id*=0, Iq* -> D/Q 电流 PI -> 电压限幅 -> 逆 Park/Clarke -> SVPWM
                                      |
                           SPMSM 电气与机械模型
                                      |
                      Id/Iq、速度、位置闭环反馈
```

## 运行

在当前目录执行：

```powershell
python simulate.py
```

运行测试：

```powershell
python -m unittest -v test_simulation.py
```

只依赖 NumPy，不需要 Matplotlib。运行后生成：

- `output/simulation.csv`：全部采样数据，可导入 MATLAB、Excel 或 Python。
- `output/summary.json`：超调、电流、电压饱和和负载恢复指标。
- `output/three_loop_response.svg`：位置、速度、电流、电压和转矩曲线，可直接用浏览器打开。
- `output/plots/*.png`：五张独立的测试结果图。

重新生成独立 PNG 图：

```powershell
python plot_results.py
```

运行复杂工况压力测试：

```powershell
python stress_scenarios.py
```

压力测试会自动生成 8 组仿真，包括基准、有/无运动前馈、高速大行程、强负载、惯量失配、摩擦失配、多段反向和正弦扰动负载。输出包括：

- `output/stress/stress_summary.csv`：每组工况的最终误差、峰值误差、RMS 误差、峰值 Iq、电压利用率和通过标志。
- `output/stress/stress_metrics.json`：同一组指标的 JSON 版本。
- `output/plots/07_stress_position_error.png`：用柱状图比较各工况位置误差。
- `output/plots/08_stress_current_voltage.png`：比较峰值 Iq 和 SVPWM 电压利用率。
- `output/plots/09_stress_reverse_tracking.png`：多段反向工况的位置、位置环采样误差和负载跟踪细节。
- `output/plots/10_stress_sine_load_tracking.png`：正弦扰动负载下的位置、位置环采样误差和负载跟踪细节。

指定其他配置或输出目录：

```powershell
python simulate.py --config config.json --output my_output
```

## 与固件资料的对应关系

- 电机参数来自 `fw/test/spmsm_motor_simulator.h` 中的 MJ5208 参数。
- 电流环默认 `Kp=0.065, Ki=120`，对应源码仿真夹具中的约 400 Hz 电流环。
- 控制频率为 30 kHz，与 `BldcServoControl::RateConfig` 的最大中断频率一致。
- D/Q 电压使用固件同样的“D 轴优先”圆形限幅。
- SVPWM 使用固件中的共模注入算法，而不是显式扇区判断。
- 轨迹规划使用固件相同的 `v²/(2a)` 制动距离思想。
- 轨迹速度沿 `sqrt(2*a*距离)` 制动包络运行，并在离散跨越目标时精确吸附，避免参考轨迹自身超调。
- 速度环包含低通负载扰动观测器，将估计负载转换为 `Iq` 前馈，减小加载与撤载时的位置偏差。
- PMSM 使用 D/Q 动态方程，包含电阻、电感、反电动势、惯量、摩擦和外部负载。
- 电机模型增加库仑摩擦，速度环加入 `J*a + B*w + Fc*tanh(w/eps)` 运动模型前馈，用来补偿加减速惯量和摩擦转矩。

## 三个环的调参入口

全部参数位于 `config.json`：

1. 位置环：`position_kp_rad_s_per_rad`
2. 速度环：`speed_kp_a_per_rad_s`、`speed_ki_a_per_rad`
3. 电流环：`current_kp_v_per_a`、`current_ki_v_per_a_s`

负载观测器由 `enable_load_disturbance_observer`、`disturbance_observer_bandwidth_hz` 和 `disturbance_observer_torque_limit_nm` 配置。真实编码器存在噪声时应降低观测器带宽，并先验证速度微分噪声。

惯性与摩擦补偿由 `enable_inertia_feedforward`、`enable_friction_feedforward`、`inertia_feedforward_kg_m2`、`viscous_friction_feedforward_nm_per_rad_s`、`coulomb_friction_feedforward_nm` 配置。当前默认值等于仿真电机参数，相当于“已经准确辨识出 J/B/Fc”的理想补偿。

建议按“电流环 -> 速度环 -> 位置环”的顺序调节。内环带宽必须显著高于外环；当前多速率配置为电流环 30 kHz、速度环 3 kHz、位置环 1 kHz。

## 默认实验场景

- `0.1 s`：位置从 0 跳变到 0.25 圈。
- 轨迹限制：2 圈/秒、12 圈/秒²。
- `0.55～0.8 s`：施加 0.08 N·m 负载转矩。
- `0.8 s` 后撤掉负载，观察位置恢复能力。

修改 `scenario` 字段即可改变目标位置和负载阶跃。

复杂工况脚本额外使用 `target_events_rev`、`load_events_nm` 和 `load_sine_nm` 描述多段目标、分段负载和正弦扰动。仿真主循环会把这些命令写入 `command_position_rev`、`load_nm`，便于检查轨迹规划前后的差异。

## 模型边界

本模型适合控制算法学习和参数初调，但不包含死区、MOSFET 压降、采样噪声、编码器量化、齿槽转矩、母线动态和热网络。上板前仍需降低电流/电压限值并重新辨识实际电机参数。
