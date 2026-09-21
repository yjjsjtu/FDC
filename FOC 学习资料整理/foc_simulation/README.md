# FOC Simulation

离线电机控制建模与仿真项目，覆盖三相 FOC、SVPWM、电机模型、R/L 辨识、摩擦/惯性辨识和前馈补偿验证。

本项目只依赖 Python 标准库和 `numpy`。绘图输出使用内置的轻量 PNG 绘图工具，避免依赖 `matplotlib`。

## 快速运行

```bash
cd "/home/ruihu/Desktop/yjj/FDC/FOC 学习资料整理/foc_simulation"

python3 examples/run_svpwm_demo.py
python3 examples/run_velocity_foc.py
python3 examples/identify_rl.py --input data/rl_step.csv
python3 examples/identify_friction_inertia.py --input data/friction_sweep.csv
python3 examples/run_compensation_compare.py
python3 examples/run_control_optimization_compare.py
python3 examples/run_position_three_loop.py
python3 examples/run_rdrive_three_loop.py
python3 examples/run_rdrive_stress_scenarios.py
python3 -m unittest discover -s tests
```

## 主要输出

- `data/*.csv`：仿真和辨识用数据。
- `outputs/*.png`：曲线图。
- `outputs/*.json`：辨识参数和补偿指标。
- `docs/*.md`：原理、辨识和补偿说明。

## 目录结构

```text
src/
  foc_math.py          坐标变换和电角度计算
  svpwm.py             SVPWM 扇区、矢量作用时间和占空比
  motor_model.py       PMSM/BLDC 简化 dq 模型
  controllers.py       PI/P 控制器和限幅
  simulator.py         闭环 FOC 仿真主循环
  trajectory.py        速度参考轨迹规划
  tuning.py            电流环/速度环 PI 初值计算
  estimators.py        编码器量化和速度估计
  identification.py    R/L、摩擦、惯性辨识
  compensation.py      前馈补偿和误差指标
  plotting.py          无第三方依赖 PNG 绘图
examples/
  run_svpwm_demo.py
  run_velocity_foc.py
  identify_rl.py
  identify_friction_inertia.py
  run_compensation_compare.py
  run_control_optimization_compare.py
  run_position_three_loop.py
  run_rdrive_three_loop.py
  run_rdrive_stress_scenarios.py
tests/
docs/
data/
outputs/
rdrive_three_loop/  从 three_loop_sim 迁移来的 RDrive/moteus 风格三环仿真
```

## 控制性能优化

`examples/run_control_optimization_compare.py` 会同时运行基础控制和优化控制：

- baseline：速度命令直接进入速度 PI，使用原始经验参数，无电压前馈。
- smooth：加入轨迹规划、参数化 PI 整定、d/q 电压前馈解耦、模拟编码器测速，优先降低电流尖峰。
- precision：不重设目标速度，直接跟踪原始速度命令，优先提高速度跟随精度。

新增输出：

- `outputs/control_optimization_compare.png`：命令速度、baseline 实际速度、precision 实际速度对比。
- `outputs/trajectory_compare.png`：原始命令速度、smooth 轨迹层参考速度、smooth 实际速度对比。
- `outputs/current_decoupling_compare.png`：baseline/precision 的 d/q 电流对比。
- `outputs/control_optimization_metrics.json`：命令速度 RMS、目标速度 RMS、Iq 峰值、Id RMS、电压饱和比例等指标。

## 位置三环控制

`examples/run_position_three_loop.py` 会运行完整的位置环、速度环、电流环级联控制：

- 位置 P 环：`position_error -> target_velocity`
- 速度 PI 环：`velocity_error -> target_iq`
- 电流 PI 环：`id/iq error -> vd/vq`

新增输出：

- `outputs/position_three_loop_response.png`：目标位置和实际位置对比。
- `outputs/position_three_loop_velocity.png`：位置环输出的目标速度和实际速度对比。
- `outputs/position_three_loop_current.png`：目标 Iq、实际 Iq、实际 Id 对比。
- `outputs/position_three_loop_metrics.json`：位置 RMS、速度 RMS、Iq 峰值等指标。

## RDrive/moteus 固件参考仿真

`rdrive_three_loop/` 是从仓库根目录 `three_loop_sim/` 迁移进来的三环仿真，参数和结构更贴近 RDrive/moteus：

- 30 kHz 电流环，多速率速度环和位置环。
- 位置 P、速度 PI、d/q 电流 PI 级联。
- D 轴优先电压限幅、共模注入 SVPWM。
- 负载扰动观测器，用来减小加载/卸载位置偏差。
- 惯性与摩擦前馈，用 `J*a + B*w + Fc*tanh(w/eps)` 折算成 q 轴电流补偿。

运行：

```bash
python3 examples/run_rdrive_three_loop.py
```

复杂工况压力测试：

```bash
python3 examples/run_rdrive_stress_scenarios.py
```

压力测试会覆盖基准、有/无惯性摩擦前馈、高速大行程、强负载、惯量失配、摩擦失配、多段反向和正弦扰动负载。新增输出位于：

- `rdrive_three_loop/output/stress/stress_summary.csv`：每组工况的误差、电流、电压利用率和是否通过。
- `rdrive_three_loop/output/stress/stress_metrics.json`：同一组指标的 JSON 版本。
- `rdrive_three_loop/output/plots/07_stress_position_error.png`：复杂工况位置峰值/RMS 误差。
- `rdrive_three_loop/output/plots/08_stress_current_voltage.png`：复杂工况峰值 Iq 和电压利用率。
- `rdrive_three_loop/output/plots/09_stress_reverse_tracking.png`：多段反向目标下的位置、位置环采样误差和负载跟踪。
- `rdrive_three_loop/output/plots/10_stress_sine_load_tracking.png`：正弦负载扰动下的位置、位置环采样误差和负载跟踪。

说明文档见 `docs/RDrive三环仿真迁移说明.md`。
