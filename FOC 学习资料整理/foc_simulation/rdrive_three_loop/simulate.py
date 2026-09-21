#!/usr/bin/env python3
"""Position-speed-current three-loop FOC simulation for the RDrive/moteus data set."""

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np


TWO_PI = 2.0 * math.pi


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


@dataclass
class PIController:
    kp: float
    ki: float
    integral_min: float
    integral_max: float
    integral: float = 0.0

    def update(self, error: float, dt: float, feedforward: float = 0.0) -> float:
        candidate = clamp(
            self.integral + self.ki * error * dt,
            self.integral_min,
            self.integral_max,
        )
        raw = self.kp * error + candidate + feedforward
        return raw

    def commit_with_back_calculation(
        self, error: float, dt: float, raw: float, limited: float, gain: float = 0.35
    ) -> None:
        self.integral = clamp(
            self.integral + self.ki * error * dt + gain * (limited - raw),
            self.integral_min,
            self.integral_max,
        )


@dataclass
class MotorState:
    theta_m_rad: float = 0.0
    omega_m_rad_s: float = 0.0
    i_d_a: float = 0.0
    i_q_a: float = 0.0


class SpmsmPlant:
    """Surface-PMSM model matching fw/test/spmsm_motor_simulator.h."""

    def __init__(self, cfg: Dict[str, float]):
        self.p = int(cfg["pole_pairs"])
        self.r = float(cfg["phase_resistance_ohm"])
        self.ld = float(cfg["d_inductance_h"])
        self.lq = float(cfg["q_inductance_h"])
        self.flux = float(cfg["flux_linkage_wb"])
        self.j = float(cfg["inertia_kg_m2"])
        self.b = float(cfg["viscous_friction_nm_per_rad_s"])
        self.fc = float(cfg.get("coulomb_friction_nm", 0.0))
        self.friction_smoothing = float(cfg.get("friction_smoothing_rad_s", 0.2))
        self.state = MotorState()

    @property
    def torque_constant_nm_a(self) -> float:
        return 1.5 * self.p * self.flux

    @property
    def torque_nm(self) -> float:
        s = self.state
        return 1.5 * self.p * (
            self.flux * s.i_q_a + (self.ld - self.lq) * s.i_d_a * s.i_q_a
        )

    def derivatives(self, state: np.ndarray, vd: float, vq: float, load_nm: float) -> np.ndarray:
        theta, omega, i_d, i_q = state
        omega_e = self.p * omega
        did = (vd - self.r * i_d + omega_e * self.lq * i_q) / self.ld
        diq = (
            vq - self.r * i_q - omega_e * (self.ld * i_d + self.flux)
        ) / self.lq
        torque = 1.5 * self.p * (
            self.flux * i_q + (self.ld - self.lq) * i_d * i_q
        )
        friction = self.b * omega + self.fc * math.tanh(omega / self.friction_smoothing)
        domega = (torque - load_nm - friction) / self.j
        return np.array([omega, domega, did, diq], dtype=float)

    def step(self, vd: float, vq: float, load_nm: float, dt: float, substeps: int) -> None:
        # Fourth-order Runge-Kutta with substeps keeps the 28.6 uH electrical
        # dynamics stable at a 30 kHz controller rate.
        h = dt / substeps
        x = np.array(
            [
                self.state.theta_m_rad,
                self.state.omega_m_rad_s,
                self.state.i_d_a,
                self.state.i_q_a,
            ],
            dtype=float,
        )
        for _ in range(substeps):
            k1 = self.derivatives(x, vd, vq, load_nm)
            k2 = self.derivatives(x + 0.5 * h * k1, vd, vq, load_nm)
            k3 = self.derivatives(x + 0.5 * h * k2, vd, vq, load_nm)
            k4 = self.derivatives(x + h * k3, vd, vq, load_nm)
            x += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        self.state = MotorState(*x.tolist())


class MotionProfile:
    """Discrete trapezoidal profile using the firmware's v^2/(2a) decision."""

    def __init__(self, max_speed: float, max_accel: float):
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0

    def update(self, target_position: float, dt: float) -> Tuple[float, float]:
        error = target_position - self.position
        if abs(error) < 1e-12 and abs(self.velocity) < self.max_accel * dt:
            self.position = target_position
            self.velocity = 0.0
            self.acceleration = 0.0
            return self.position, self.velocity
        direction = 1.0 if error >= 0.0 else -1.0
        # The ideal speed which can still stop exactly at the target is
        # sqrt(2*a*distance).  Following this envelope avoids the discrete
        # bang-bang profile overshooting and then reversing around the target.
        braking_speed = math.sqrt(max(0.0, 2.0 * self.max_accel * abs(error)))
        desired_velocity = direction * min(self.max_speed, braking_speed)
        velocity_delta = clamp(
            desired_velocity - self.velocity,
            -self.max_accel * dt,
            self.max_accel * dt,
        )
        old_velocity = self.velocity
        self.velocity = clamp(self.velocity + velocity_delta, -self.max_speed, self.max_speed)
        self.acceleration = (self.velocity - old_velocity) / dt
        self.position += 0.5 * (old_velocity + self.velocity) * dt

        # Enforce an exact terminal state if the discrete integration crosses the
        # target.  This removes the residual one-sample position overshoot.
        new_error = target_position - self.position
        if error != 0.0 and error * new_error <= 0.0:
            self.position = target_position
            self.velocity = 0.0
            self.acceleration = 0.0
        return self.position, self.velocity


def friction_feedforward_torque(
    velocity_rad_s: float,
    viscous_nm_per_rad_s: float,
    coulomb_nm: float,
    smoothing_rad_s: float,
) -> float:
    return (
        viscous_nm_per_rad_s * velocity_rad_s
        + coulomb_nm * math.tanh(velocity_rad_s / smoothing_rad_s)
    )


def limit_dq_voltage(vd: float, vq: float, max_voltage: float) -> Tuple[float, float, bool]:
    """Firmware-style D-axis-priority circular voltage limiter."""
    if vd * vd + vq * vq <= max_voltage * max_voltage:
        return vd, vq, False
    vd_limited = clamp(vd, -max_voltage, max_voltage)
    q_headroom = math.sqrt(max(0.0, max_voltage * max_voltage - vd_limited * vd_limited))
    return vd_limited, clamp(vq, -q_headroom, q_headroom), True


def dq_to_phase_and_pwm(
    vd: float, vq: float, electrical_angle: float, bus_voltage: float
) -> Tuple[float, float, float, float, float, float]:
    c = math.cos(electrical_angle)
    s = math.sin(electrical_angle)
    va = c * vd - s * vq
    vbeta = s * vd + c * vq
    vb = -0.5 * va + math.sqrt(3.0) * 0.5 * vbeta
    vc = -0.5 * va - math.sqrt(3.0) * 0.5 * vbeta
    normalized = [va / bus_voltage, vb / bus_voltage, vc / bus_voltage]
    offset = 0.5 * (min(normalized) + max(normalized)) - 0.5
    duties = [clamp(value - offset, 0.0, 1.0) for value in normalized]
    return va, vb, vc, duties[0], duties[1], duties[2]


def target_position_rev_at(scenario: Dict, time_s: float) -> float:
    """Piecewise-constant commanded target position in revolutions."""
    events = scenario.get("target_events_rev")
    if events:
        target = 0.0
        for event_time, event_target in sorted(events, key=lambda item: float(item[0])):
            if time_s >= float(event_time):
                target = float(event_target)
            else:
                break
        return target
    return (
        float(scenario["target_position_rev"])
        if time_s >= float(scenario["position_step_time_s"])
        else 0.0
    )


def load_torque_nm_at(scenario: Dict, time_s: float) -> float:
    """External load torque, supporting rectangular and sinusoidal disturbances."""
    load_nm = 0.0
    events = scenario.get("load_events_nm")
    if events:
        for start_s, end_s, torque_nm in events:
            if float(start_s) <= time_s < float(end_s):
                load_nm += float(torque_nm)
    elif "load_torque_nm" in scenario:
        if float(scenario["load_start_s"]) <= time_s < float(scenario["load_end_s"]):
            load_nm += float(scenario["load_torque_nm"])

    sine = scenario.get("load_sine_nm")
    if sine:
        start_s = float(sine.get("start_s", 0.0))
        end_s = float(sine.get("end_s", time_s + 1.0))
        if start_s <= time_s < end_s:
            phase = float(sine.get("phase_rad", 0.0))
            load_nm += float(sine.get("offset_nm", 0.0))
            load_nm += float(sine["amplitude_nm"]) * math.sin(
                TWO_PI * float(sine["frequency_hz"]) * (time_s - start_s) + phase
            )
    return load_nm


def first_target_time_s(scenario: Dict) -> float:
    events = scenario.get("target_events_rev")
    if events:
        return min(float(item[0]) for item in events)
    return float(scenario["position_step_time_s"])


def load_interval_s(scenario: Dict) -> Tuple[float, float]:
    intervals: List[Tuple[float, float]] = []
    events = scenario.get("load_events_nm")
    if events:
        intervals.extend((float(start_s), float(end_s)) for start_s, end_s, _ in events)
    elif "load_start_s" in scenario and "load_end_s" in scenario:
        intervals.append((float(scenario["load_start_s"]), float(scenario["load_end_s"])))
    sine = scenario.get("load_sine_nm")
    if sine:
        intervals.append((float(sine.get("start_s", 0.0)), float(sine["end_s"])))
    if not intervals:
        return 0.0, 0.0
    return min(start for start, _ in intervals), max(end for _, end in intervals)


def simulate(config: Dict) -> Tuple[Dict[str, np.ndarray], Dict[str, float]]:
    sim = config["simulation"]
    inv = config["inverter"]
    ctl = config["controller"]
    scenario = config["scenario"]
    plant = SpmsmPlant(config["motor"])

    rate = int(sim["control_rate_hz"])
    dt = 1.0 / rate
    steps = int(round(float(sim["duration_s"]) * rate))
    speed_div = int(sim["speed_loop_divider"])
    position_div = int(sim["position_loop_divider"])
    log_div = int(sim["log_divider"])
    speed_dt = dt * speed_div
    position_dt = dt * position_div

    bus_v = float(inv["bus_voltage_v"])
    max_voltage = bus_v / math.sqrt(3.0) * float(inv["svpwm_utilization"])
    current_limit = float(inv["current_limit_a"])
    max_speed = float(ctl["speed_limit_rev_s"]) * TWO_PI
    max_accel = float(ctl["acceleration_limit_rev_s2"]) * TWO_PI

    profile = MotionProfile(max_speed, max_accel)
    speed_pi = PIController(
        float(ctl["speed_kp_a_per_rad_s"]),
        float(ctl["speed_ki_a_per_rad"]),
        -current_limit,
        current_limit,
    )
    id_pi = PIController(float(ctl["current_kp_v_per_a"]), float(ctl["current_ki_v_per_a_s"]), -max_voltage, max_voltage)
    iq_pi = PIController(float(ctl["current_kp_v_per_a"]), float(ctl["current_ki_v_per_a_s"]), -max_voltage, max_voltage)

    speed_ref = 0.0
    iq_ref_target = 0.0
    iq_ref = 0.0
    id_ref = 0.0
    vd = 0.0
    vq = 0.0
    estimated_load_nm = 0.0
    inertia_ff_nm = 0.0
    friction_ff_nm = 0.0
    motion_ff_nm = 0.0
    previous_speed_sample = 0.0
    observer_alpha = 1.0 - math.exp(
        -TWO_PI * float(ctl.get("disturbance_observer_bandwidth_hz", 80.0)) * speed_dt
    )
    saturation_count = 0
    log: Dict[str, List[float]] = {name: [] for name in (
        "time_s", "command_position_rev", "position_ref_rev", "position_rev", "speed_ref_rev_s",
        "speed_rev_s", "id_ref_a", "id_a", "iq_ref_a", "iq_a",
        "vd_v", "vq_v", "torque_nm", "load_nm", "load_estimate_nm",
        "inertia_feedforward_nm", "friction_feedforward_nm", "motion_feedforward_nm",
        "duty_a", "duty_b", "duty_c"
    )}

    for step in range(steps + 1):
        t = step * dt
        target_rev = target_position_rev_at(scenario, t)
        target_rad = target_rev * TWO_PI

        if step % position_div == 0:
            profile_pos, profile_vel = profile.update(target_rad, position_dt)
            position_error = profile_pos - plant.state.theta_m_rad
            speed_ref = clamp(
                profile_vel + float(ctl["position_kp_rad_s_per_rad"]) * position_error,
                -max_speed,
                max_speed,
            )

        if step % speed_div == 0:
            measured_acceleration = (
                plant.state.omega_m_rad_s - previous_speed_sample
            ) / speed_dt
            previous_speed_sample = plant.state.omega_m_rad_s
            raw_load_estimate = (
                plant.torque_nm
                - plant.j * measured_acceleration
                - plant.b * plant.state.omega_m_rad_s
            )
            observer_limit = float(ctl.get("disturbance_observer_torque_limit_nm", 0.0))
            raw_load_estimate = clamp(raw_load_estimate, -observer_limit, observer_limit)
            if ctl.get("enable_load_disturbance_observer", False):
                estimated_load_nm += observer_alpha * (raw_load_estimate - estimated_load_nm)
            else:
                estimated_load_nm = 0.0

            speed_error = speed_ref - plant.state.omega_m_rad_s
            load_iq_feedforward = estimated_load_nm / plant.torque_constant_nm_a
            inertia_ff_nm = 0.0
            if ctl.get("enable_inertia_feedforward", False):
                inertia = float(ctl.get("inertia_feedforward_kg_m2", plant.j))
                inertia_ff_nm = (
                    float(ctl.get("inertia_feedforward_scale", 1.0))
                    * inertia
                    * profile.acceleration
                )
            friction_ff_nm = 0.0
            if ctl.get("enable_friction_feedforward", False):
                friction_ff_nm = float(ctl.get("friction_feedforward_scale", 1.0)) * friction_feedforward_torque(
                    speed_ref,
                    float(ctl.get("viscous_friction_feedforward_nm_per_rad_s", plant.b)),
                    float(ctl.get("coulomb_friction_feedforward_nm", plant.fc)),
                    float(ctl.get("friction_feedforward_smoothing_rad_s", plant.friction_smoothing)),
                )
            motion_ff_nm = inertia_ff_nm + friction_ff_nm
            motion_iq_feedforward = motion_ff_nm / plant.torque_constant_nm_a
            raw_iq_feedback = speed_pi.update(speed_error, speed_dt)
            raw_iq = raw_iq_feedback + load_iq_feedforward + motion_iq_feedforward
            iq_ref_target = clamp(raw_iq, -current_limit, current_limit)
            speed_pi.commit_with_back_calculation(
                speed_error,
                speed_dt,
                raw_iq_feedback,
                iq_ref_target - load_iq_feedforward - motion_iq_feedforward,
            )

        max_iq_step = float(ctl["current_reference_slew_a_s"]) * dt
        iq_ref += clamp(iq_ref_target - iq_ref, -max_iq_step, max_iq_step)

        omega_e = plant.p * plant.state.omega_m_rad_s
        vd_ff = 0.0
        vq_ff = 0.0
        if ctl["enable_resistance_feedforward"]:
            vd_ff += plant.r * id_ref
            vq_ff += plant.r * iq_ref
        if ctl["enable_bemf_feedforward"]:
            vq_ff += omega_e * plant.flux
        if ctl["enable_cross_coupling_feedforward"]:
            vd_ff -= omega_e * plant.lq * plant.state.i_q_a
            vq_ff += omega_e * plant.ld * plant.state.i_d_a

        id_error = id_ref - plant.state.i_d_a
        iq_error = iq_ref - plant.state.i_q_a
        raw_vd = id_pi.update(id_error, dt, vd_ff)
        raw_vq = iq_pi.update(iq_error, dt, vq_ff)
        vd, vq, saturated = limit_dq_voltage(raw_vd, raw_vq, max_voltage)
        saturation_count += int(saturated)
        id_pi.commit_with_back_calculation(id_error, dt, raw_vd, vd)
        iq_pi.commit_with_back_calculation(iq_error, dt, raw_vq, vq)

        load_nm = load_torque_nm_at(scenario, t)
        electrical_angle = (plant.p * plant.state.theta_m_rad) % TWO_PI
        _, _, _, duty_a, duty_b, duty_c = dq_to_phase_and_pwm(vd, vq, electrical_angle, bus_v)

        if step % log_div == 0:
            values = {
                "time_s": t,
                "command_position_rev": target_rev,
                "position_ref_rev": profile.position / TWO_PI,
                "position_rev": plant.state.theta_m_rad / TWO_PI,
                "speed_ref_rev_s": speed_ref / TWO_PI,
                "speed_rev_s": plant.state.omega_m_rad_s / TWO_PI,
                "id_ref_a": id_ref,
                "id_a": plant.state.i_d_a,
                "iq_ref_a": iq_ref,
                "iq_a": plant.state.i_q_a,
                "vd_v": vd,
                "vq_v": vq,
                "torque_nm": plant.torque_nm,
                "load_nm": load_nm,
                "load_estimate_nm": estimated_load_nm,
                "inertia_feedforward_nm": inertia_ff_nm,
                "friction_feedforward_nm": friction_ff_nm,
                "motion_feedforward_nm": motion_ff_nm,
                "duty_a": duty_a,
                "duty_b": duty_b,
                "duty_c": duty_c,
            }
            for key, value in values.items():
                log[key].append(value)

        if step < steps:
            plant.step(vd, vq, load_nm, dt, int(sim["plant_substeps"]))

    result = {key: np.asarray(value, dtype=float) for key, value in log.items()}
    time = result["time_s"]
    pos = result["position_rev"]
    final_target = target_position_rev_at(scenario, float(time[-1]))
    after_step = time >= first_target_time_s(scenario)
    overshoot = 0.0
    if np.any(after_step):
        if final_target >= 0.0:
            overshoot = max(0.0, float(np.max(pos[after_step]) - final_target))
        else:
            overshoot = max(0.0, float(final_target - np.min(pos[after_step])))
    final_error = float(final_target - pos[-1])
    load_start, load_end = load_interval_s(scenario)
    recovery_mask = time >= min(load_end + 0.15, float(sim["duration_s"]) - 0.05)
    recovery_error = float(np.max(np.abs(pos[recovery_mask] - final_target)))
    position_error_deg = (result["position_ref_rev"] - pos) * 360.0
    def peak_error(start: float, end: float) -> float:
        mask = (time >= start) & (time < end)
        if not np.any(mask):
            return 0.0
        return float(np.max(np.abs(position_error_deg[mask])))
    summary = {
        "final_position_rev": float(pos[-1]),
        "final_position_error_rev": final_error,
        "overshoot_rev": overshoot,
        "position_rms_error_deg": float(np.sqrt(np.mean(position_error_deg * position_error_deg))),
        "position_peak_error_deg": float(np.max(np.abs(position_error_deg))),
        "speed_rms_error_rev_s": float(np.sqrt(np.mean((result["speed_ref_rev_s"] - result["speed_rev_s"]) ** 2))),
        "max_abs_speed_rev_s": float(np.max(np.abs(result["speed_rev_s"]))),
        "max_abs_iq_a": float(np.max(np.abs(result["iq_a"]))),
        "max_abs_id_a": float(np.max(np.abs(result["id_a"]))),
        "max_voltage_vector_v": float(np.max(np.hypot(result["vd_v"], result["vq_v"]))),
        "allowed_voltage_vector_v": max_voltage,
        "voltage_saturation_fraction": saturation_count / (steps + 1),
        "max_abs_inertia_feedforward_nm": float(np.max(np.abs(result["inertia_feedforward_nm"]))),
        "max_abs_friction_feedforward_nm": float(np.max(np.abs(result["friction_feedforward_nm"]))),
        "max_abs_motion_feedforward_nm": float(np.max(np.abs(result["motion_feedforward_nm"]))),
        "post_load_recovery_error_rev": recovery_error,
        "move_peak_error_deg": peak_error(first_target_time_s(scenario), load_start if load_start else float(sim["duration_s"])),
        "load_peak_error_deg": peak_error(load_start, load_end),
        "unload_peak_error_deg": peak_error(load_end, min(float(sim["duration_s"]), load_end + 0.2)),
        "reference_overshoot_deg": max(0.0, float(np.max(result["position_ref_rev"]) - final_target) * 360.0),
        "torque_constant_nm_a": plant.torque_constant_nm_a,
    }
    return result, summary


def write_csv(path: Path, result: Dict[str, np.ndarray]) -> None:
    keys = list(result)
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(keys)
        writer.writerows(zip(*(result[key] for key in keys)))


def polyline_points(x: np.ndarray, y: np.ndarray, left: float, top: float, width: float, height: float,
                    ymin: float, ymax: float) -> str:
    if ymax <= ymin:
        ymax = ymin + 1.0
    stride = max(1, len(x) // 1800)
    xs = left + width * (x[::stride] - x[0]) / max(1e-12, x[-1] - x[0])
    ys = top + height * (1.0 - (y[::stride] - ymin) / (ymax - ymin))
    return " ".join(f"{a:.2f},{b:.2f}" for a, b in zip(xs, ys))


def write_svg(path: Path, result: Dict[str, np.ndarray], summary: Dict[str, float]) -> None:
    width, height = 1200, 1120
    left, plot_width, panel_height = 92, 1050, 175
    panels = [
        ("位置 / rev", [("给定", "position_ref_rev", "#ef4444"), ("实际", "position_rev", "#2563eb")]),
        ("速度 / (rev/s)", [("给定", "speed_ref_rev_s", "#ef4444"), ("实际", "speed_rev_s", "#2563eb")]),
        ("D/Q 电流 / A", [("Iq*", "iq_ref_a", "#ef4444"), ("Iq", "iq_a", "#2563eb"), ("Id", "id_a", "#16a34a")]),
        ("D/Q 电压 / V", [("Vd", "vd_v", "#16a34a"), ("Vq", "vq_v", "#7c3aed")]),
        ("转矩 / N·m", [("电磁转矩", "torque_nm", "#2563eb"), ("负载", "load_nm", "#f97316")]),
    ]
    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<style>text{font-family:"Noto Sans CJK SC","Noto Sans CJK","Microsoft YaHei",Arial,sans-serif;fill:#172033}.title{font-size:24px;font-weight:700}.label{font-size:14px}.small{font-size:12px}</style>',
        '<text x="60" y="38" class="title">RDrive / moteus 三环 FOC 仿真</text>',
        f'<text x="60" y="64" class="label">最终误差 {summary["final_position_error_rev"]:.6f} rev　峰值 Iq {summary["max_abs_iq_a"]:.2f} A　电压饱和占比 {100*summary["voltage_saturation_fraction"]:.2f}%</text>',
    ]
    x = result["time_s"]
    for index, (ylabel, series) in enumerate(panels):
        top = 88 + index * 198
        all_values = np.concatenate([result[key] for _, key, _ in series])
        ymin, ymax = float(np.min(all_values)), float(np.max(all_values))
        margin = max(1e-6, 0.08 * (ymax - ymin if ymax > ymin else max(1.0, abs(ymax))))
        ymin -= margin
        ymax += margin
        pieces.extend([
            f'<rect x="{left}" y="{top}" width="{plot_width}" height="{panel_height}" fill="#f8fafc" stroke="#cbd5e1"/>',
            f'<line x1="{left}" y1="{top+panel_height/2}" x2="{left+plot_width}" y2="{top+panel_height/2}" stroke="#e2e8f0"/>',
            f'<text x="12" y="{top+24}" class="label">{ylabel}</text>',
            f'<text x="{left-8}" y="{top+12}" text-anchor="end" class="small">{ymax:.3g}</text>',
            f'<text x="{left-8}" y="{top+panel_height}" text-anchor="end" class="small">{ymin:.3g}</text>',
        ])
        legend_x = left + 12
        for name, key, color in series:
            points = polyline_points(x, result[key], left, top, plot_width, panel_height, ymin, ymax)
            pieces.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.6"/>')
            pieces.append(f'<line x1="{legend_x}" y1="{top+18}" x2="{legend_x+24}" y2="{top+18}" stroke="{color}" stroke-width="3"/>')
            pieces.append(f'<text x="{legend_x+30}" y="{top+23}" class="small">{name}</text>')
            legend_x += 105
        if index == len(panels) - 1:
            for tick in range(7):
                tx = left + plot_width * tick / 6
                tv = x[0] + (x[-1] - x[0]) * tick / 6
                pieces.append(f'<text x="{tx}" y="{top+panel_height+20}" text-anchor="middle" class="small">{tv:.2f}</text>')
            pieces.append(f'<text x="{left+plot_width/2}" y="{top+panel_height+42}" text-anchor="middle" class="label">时间 / s</text>')
    pieces.append('</svg>')
    path.write_text("\n".join(pieces), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.json"))
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("output"))
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    args.output.mkdir(parents=True, exist_ok=True)
    result, summary = simulate(config)
    write_csv(args.output / "simulation.csv", result)
    (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    write_svg(args.output / "three_loop_response.svg", result, summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"CSV: {args.output / 'simulation.csv'}")
    print(f"SVG: {args.output / 'three_loop_response.svg'}")


if __name__ == "__main__":
    main()
