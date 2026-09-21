# RDrive 三环仿真迁移说明

`rdrive_three_loop/` 是从仓库根目录的 `three_loop_sim/` 迁移到本机 `foc_simulation` 环境中的固件参考仿真。

这份仿真参考了解压后的 `acuator-release/source_files/dummyx2_rdrivec1/moteus/`，控制结构更贴近 RDrive/moteus：

```text
位置轨迹 -> 位置 P 环 -> 速度 PI 环 -> Iq 给定
                         -> d/q 电流 PI -> 电压限幅 -> 逆 Park/Clarke -> SVPWM
                         -> SPMSM 电气/机械模型 -> 位置、速度、电流反馈
```

## 解压位置

参考源码已从：

```text
FOC 学习资料整理/acuator-release/source_files/dummyx2_rdrivec1.tar.gz
```

解压到：

```text
FOC 学习资料整理/acuator-release/source_files/dummyx2_rdrivec1/moteus/
```

该目录作为本地参考源码使用，已加入 `.gitignore`，不会重复提交到 GitHub。

## 运行方式

在 `foc_simulation` 目录下运行：

```bash
python3 examples/run_rdrive_three_loop.py
```

复杂工况压力测试：

```bash
python3 examples/run_rdrive_stress_scenarios.py
```

输出目录：

```text
rdrive_three_loop/output/
```

主要输出包括：

- `simulation.csv`：位置、速度、电流、电压、占空比等全部采样数据。
- `summary.json`：最终误差、超调、负载恢复、电流峰值、电压饱和比例等指标。
- `three_loop_response.svg`：汇总曲线。
- `plots/*.png`：独立 PNG 图，若系统安装了 Pillow 会自动重新生成。

压力测试额外输出：

- `stress/stress_summary.csv`：复杂工况指标汇总。
- `stress/stress_metrics.json`：复杂工况指标 JSON。
- `plots/07_stress_position_error.png`：各工况位置峰值误差和 RMS 误差。
- `plots/08_stress_current_voltage.png`：各工况峰值 Iq 和电压利用率。
- `plots/09_stress_reverse_tracking.png`：多段反向目标的跟踪细节。
- `plots/10_stress_sine_load_tracking.png`：正弦扰动负载下的跟踪细节。

## 与本机教学仿真的区别

`rdrive_three_loop` 更像硬件固件参考仿真，参数来自 RDrive/moteus 方向，包含 30 kHz 电流环、多速率位置/速度/电流级联、D 轴优先电压限幅、共模注入 SVPWM、负载扰动观测器，以及惯性/摩擦前馈补偿。

惯性与摩擦补偿使用：

```text
tau_ff = J_hat * acceleration_ref
       + B_hat * velocity_ref
       + Fc_hat * tanh(velocity_ref / eps)
Iq_ff = tau_ff / Kt
```

其中 `J_hat`、`B_hat`、`Fc_hat` 当前取自 `config.json`，默认等于仿真电机模型参数。第 6 张输出图 `06_friction_inertia_feedforward.png` 会显示惯性前馈、摩擦前馈、合计前馈以及折算后的 q 轴电流。

## 复杂工况验证

`stress_scenarios.py` 用同一套三环控制器连续跑 8 类情况：

- 基准有前馈：验证默认位置阶跃和负载阶跃。
- 基准无运动前馈：作为对照，观察没有 `J/B/Fc` 前馈时的位置误差变化。
- 高速大行程：目标位置更远、速度和加速度限制更高。
- 强负载：负载转矩提高到接近电流余量的工况。
- 惯量失配：真实模型惯量变大，但控制器仍使用原辨识参数。
- 摩擦失配：真实模型摩擦变大，但控制器仍使用原辨识参数。
- 多段反向：目标位置多次正反切换，同时叠加正负负载。
- 正弦扰动：负载转矩包含连续周期扰动。

脚本的通过条件是峰值位置误差、电流限制和电压饱和都没有越界。该测试不是证明真实硬件一定稳定，而是检查当前控制器在更复杂输入下是否仍有合理的跟踪余量。

原有 `src/` 与 `examples/` 更偏教学和周报展示，覆盖 Clarke/Park、显式 SVPWM 扇区、电阻电感辨识、摩擦惯性辨识、补偿验证和控制优化对比。

后续如果要接真实硬件，建议以 `rdrive_three_loop` 作为三环控制性能基准，再把 `src/identification.py` 和 `src/compensation.py` 中的辨识/补偿流程迁移到同一套参数上。
