# FOC 建模与仿真说明

## 1. 控制链路

本仿真项目的主链路是：

```text
目标位置/目标速度
  -> 位置 P 环（位置模式）
  -> 可选轨迹规划（速度模式）
  -> 速度 PI
  -> 目标 q 轴电流 Iq*
  -> d/q 电流 PI
  -> 可选电压前馈解耦
  -> Ud/Uq
  -> 反 Park 变换
  -> Ualpha/Ubeta
  -> SVPWM
  -> A/B/C 三相占空比
  -> 电机 dq 模型
  -> 位置/速度/电流反馈
```

默认 `Id* = 0`，表示不主动励磁，把电流尽量用到产生转矩的 `Iq` 方向。

位置三环模式下，控制链路为：

```text
theta_target - theta_feedback
  -> position P
  -> target_velocity
  -> velocity PI
  -> target_iq
  -> current PI
  -> vd/vq
```

位置环输出会被 `position_velocity_limit_rad_s` 限制，避免位置误差较大时给速度环一个过大的速度目标。

smooth 优化模式下，上层命令速度不会直接进入速度环，而是先经过轨迹规划器：

```text
command_velocity
  -> acceleration/jerk limit
  -> target_position / target_velocity / target_acceleration
```

这样可以避免硬阶跃速度造成不真实的电流尖峰。速度反馈也可以从“理想真实速度”
切换成“编码器角度量化 + 低通测速”，用于模拟真实硬件闭环。

precision 优化模式下，不改变上层速度命令，而是通过更强的速度环参数、电流环参数和
电压前馈解耦来提高对原始目标速度的跟随精度；代价是 q 轴电流峰值会明显升高。

## 2. 坐标变换

三相电流或电压先经 Clarke 变换得到静止坐标：

```text
alpha = A
beta  = (A + 2B) / sqrt(3)
```

再用电角度做 Park 变换：

```text
d =  cos(theta_e) * alpha + sin(theta_e) * beta
q = -sin(theta_e) * alpha + cos(theta_e) * beta
```

电角度来自编码器机械角度：

```text
theta_e = pole_pairs * theta_m - zero_electric_angle
```

## 3. SVPWM

SVPWM 把目标电压矢量 `Ualpha/Ubeta` 分解成相邻两个有效矢量和零矢量的作用时间：

```text
T1 = sqrt(3) * sin(sector*pi/3 - angle) * modulation
T2 = sqrt(3) * sin(angle - (sector-1)*pi/3) * modulation
T0 = 1 - T1 - T2
```

以第 1 扇区为例：

```text
Ta = T1 + T2 + T0/2
Tb = T2 + T0/2
Tc = T0/2
```

`Ta/Tb/Tc` 就是 A/B/C 三相 PWM 占空比。

## 4. 电机模型

当前采用表贴式 PMSM/BLDC 简化 dq 模型：

```text
Ld = Lq = L
Te = Kt * Iq
J * domega/dt = Te - B*omega - Fc*tanh(omega/eps) - load_torque
```

电流方程：

```text
dId/dt = (Vd - R*Id + omega_e*L*Iq) / L
dIq/dt = (Vq - R*Iq - omega_e*L*Id - Ke*omega_m) / L
```

这个模型用于教学和算法验证，不替代真实硬件标定。

## 5. 控制性能优化

### 5.1 PI 参数化整定

电流环按 RL 一阶对象估算初值：

```text
wc = 2*pi*current_bandwidth
Kp_i = L * wc
Ki_i = R * wc
```

速度环按二阶闭环期望估算初值：

```text
wn = 2*pi*velocity_bandwidth
Kp_v = (2*zeta*wn*J - B) / Kt
Ki_v = J*wn^2 / Kt
```

这不是最终调参结论，而是比纯经验参数更有物理依据的初始值。

### 5.2 电压前馈解耦

优化模式在电流 PI 输出上叠加模型前馈：

```text
Vd_ff = -omega_e * L * Iq
Vq_ff = R * Iq_target + omega_e * L * Id + Ke * omega_m
```

其中 `Vd_ff` 抵消 d/q 交叉耦合，`Vq_ff` 提前补偿电阻压降和反电动势。
这样电流 PI 不需要完全靠误差积分来“追”电压，Iq 跟随会更稳。

### 5.3 限幅和抗积分饱和

电压矢量超过 `voltage_limit_v` 时，会按比例缩小 `Vd/Vq`。
如果发生整体电压限幅，电流 PI 会回退本周期积分，避免解除饱和后出现明显过冲。

## 6. 运行方式

```bash
cd "/home/ruihu/Desktop/yjj/FDC/FOC 学习资料整理/foc_simulation"
python3 examples/run_svpwm_demo.py
python3 examples/run_velocity_foc.py
python3 examples/run_control_optimization_compare.py
python3 examples/run_position_three_loop.py
```

输出：

- `outputs/svpwm_sector.png`
- `outputs/velocity_response.png`
- `outputs/current_loop.png`
- `outputs/control_optimization_compare.png`
- `outputs/trajectory_compare.png`
- `outputs/current_decoupling_compare.png`
- `outputs/position_three_loop_response.png`
- `outputs/position_three_loop_velocity.png`
- `outputs/position_three_loop_current.png`
- `data/velocity_foc.csv`
