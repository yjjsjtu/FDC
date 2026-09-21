#!/usr/bin/env python3
"""
calibrate_output_encoder_offset.py
-----------------------------------
精确校准 motor_position.sources.1.offset（AUX1 输出端编码器零点偏移）。

配置假设（与 dual_encoder_test_tmp.txt 一致）：
  - sources.0: AUX2 电机侧, sign=+1, reference=0 (motor space)
  - sources.1: AUX1 输出端, sign=-1, reference=1 (output space)
  - output.source         = 0  (运行时位置跟踪用电机侧编码器)
  - output.reference_source = 1  (上电 homing 用输出端编码器)
  - rotor_to_output_ratio = 1/30

原理：
  pos_s1 (输出转数) = -1 × (raw + offset) / 16384
  上电时: N = round(pos_s1 / ratio - motor_frac)
  若 offset 不准 → pos_s1 偏移 → N 取整错误 → ±12° homing 错误

校准策略：
  1. 将关节置于已知物理位置（建议用出问题的位置，如 132°）
  2. 断电再上电（强制重新 homing）
  3. 运行此脚本，输入真实物理角度
  4. 脚本读取控制器上报位置，计算 offset 修正量并输出命令

用法:
  python3 calibrate_output_encoder_offset.py [--id 6] [--true_pos 0.0]
"""

import asyncio
import argparse
import math

try:
    import moteus
except ImportError:
    print("ERROR: moteus Python package not found.")
    print("Install: pip install moteus")
    raise

# ============================================================
# 配置区：与配置文件保持一致
# ============================================================
CONTROLLER_ID       = 6
CPR                 = 16384       # AUX1 编码器每转计数
SIGN_1              = -1          # sources.1.sign
CURRENT_OFFSET      = 108         # sources.1.offset 当前值（整数计数）
ROTOR_TO_OUTPUT     = 1.0 / 30.0  # 减速比（输出/电机）
ONE_MOTOR_TURN_DEG  = ROTOR_TO_OUTPUT * 360.0  # = 12.0°

# ============================================================


async def read_position_averaged(c: "moteus.Controller", n_samples=30, interval=0.03):
    """读取 n_samples 次位置，去除异常值后取平均（单位：输出转数）"""
    samples = []
    for _ in range(n_samples):
        try:
            state = await c.query()
            if state is not None:
                pos = state.values.get(moteus.Register.POSITION)
                if pos is not None and math.isfinite(pos):
                    samples.append(pos)
        except Exception:
            pass
        await asyncio.sleep(interval)

    if not samples:
        raise RuntimeError("无法从控制器读取位置数据，请检查连接。")

    # 剔除离均值超过 3σ 的异常值
    mean = sum(samples) / len(samples)
    std = math.sqrt(sum((x - mean) ** 2 for x in samples) / len(samples))
    filtered = [x for x in samples if abs(x - mean) <= 3 * std] or samples
    return sum(filtered) / len(filtered)


def compute_offset_correction(error_rev: float) -> dict:
    """
    根据 homing 误差（输出转数）计算 sources.1.offset 修正量。

    数学推导：
      pos_s1 = SIGN_1 × (raw + offset) / CPR
      Δpos_s1 = SIGN_1 × Δoffset / CPR
    需要 Δpos_s1 = -error_rev（消除误差）：
      Δoffset = -error_rev × CPR / SIGN_1 = error_rev × CPR  (SIGN_1=-1)
    """
    delta_offset = (-error_rev) * CPR / SIGN_1  # = error_rev * CPR when SIGN_1=-1
    new_offset = CURRENT_OFFSET + delta_offset
    return {
        "error_rev": error_rev,
        "error_deg": error_rev * 360.0,
        "delta_offset": delta_offset,
        "new_offset": new_offset,
        "new_offset_rounded": round(new_offset),
    }


def print_result(result: dict, controller_id: int):
    print("\n" + "=" * 55)
    print("  校准结果 / Calibration Result")
    print("=" * 55)
    print(f"  Homing 误差      : {result['error_deg']:+.3f}°  ({result['error_rev']:+.6f} rev)")
    print(f"  当前 offset      : {CURRENT_OFFSET} counts")
    print(f"  Δoffset 修正量   : {result['delta_offset']:+.2f} counts")
    print(f"  新 offset        : {result['new_offset']:.2f}  → 取整 {result['new_offset_rounded']}")
    print("=" * 55)

    if abs(result['error_deg']) < 1.0:
        print("\n✅ 误差 < 1°，offset 校准已足够精确，无需修改。")
        return

    n_turns = round(result['error_deg'] / ONE_MOTOR_TURN_DEG)
    print(f"\n⚠️  误差约为 {n_turns:+d} 个电机圈（{n_turns * ONE_MOTOR_TURN_DEG:+.1f}°），"
          f"需要校准。")
    print("\n请执行以下命令应用新 offset：\n")
    cmd = (
        f"python3 -m moteus.moteus_tool --target {controller_id} "
        f"-s 'conf set motor_position.sources.1.offset {result['new_offset_rounded']}' "
        f"'conf write'"
    )
    print(f"  {cmd}")
    print("\n应用后请：")
    print("  1. 断电再上电（强制重新 homing）")
    print("  2. 检查上报位置是否正确")
    print("  3. 如仍有误差，重复运行此脚本迭代校准")


async def calibrate(true_pos_deg: float, controller_id: int):
    print(f"\n{'='*55}")
    print(f"  AUX1 输出端编码器 Offset 校准")
    print(f"{'='*55}")
    print(f"  控制器 ID       : {controller_id}")
    print(f"  当前 offset     : {CURRENT_OFFSET} counts")
    print(f"  减速比          : 1/{round(1/ROTOR_TO_OUTPUT)}")
    print(f"  输入真实位置    : {true_pos_deg:.3f}°")
    print(f"{'='*55}\n")

    print("正在初始化 moteus 控制器...")
    transport = moteus.Fdcanusb()
    c = moteus.Controller(id=controller_id, transport=transport)

    print(f"读取上电后位置（采样 30 次，约 1 秒）...")
    reported_pos_rev = await read_position_averaged(c, n_samples=30, interval=0.03)
    reported_pos_deg = reported_pos_rev * 360.0

    print(f"\n  上报位置  : {reported_pos_deg:.4f}°  ({reported_pos_rev:.6f} rev)")
    print(f"  真实位置  : {true_pos_deg:.4f}°  ({true_pos_deg/360:.6f} rev)")

    # Homing 误差 = 上报 - 真实（在输出转数空间）
    error_deg = reported_pos_deg - true_pos_deg
    error_rev = error_deg / 360.0

    result = compute_offset_correction(error_rev)
    print_result(result, controller_id)

    try:
        transport.close()
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser(
        description="校准 Moteus 双编码器关节的 AUX1 (sources.1) offset"
    )
    parser.add_argument(
        "--id", type=int, default=CONTROLLER_ID,
        help=f"Moteus 控制器 CAN ID（默认 {CONTROLLER_ID}）"
    )
    parser.add_argument(
        "--true_pos", type=float, default=None,
        help="关节当前真实物理角度（度），不指定则交互输入"
    )
    args = parser.parse_args()

    if args.true_pos is None:
        print("\n" + "=" * 55)
        print("  AUX1 输出端编码器 Offset 校准工具")
        print("=" * 55)
        print("\n操作步骤：")
        print("  1. 将关节移动到一个【已知精确角度】的位置")
        print("     建议使用出现 homing 错误的位置（如 132°）")
        print("  2. 【断电再上电】控制器（强制重新执行 homing）")
        print("  3. 上电后【不要移动关节】，立即运行此脚本")
        print("  4. 输入实际物理角度\n")
        try:
            true_pos_deg = float(input("输入关节真实物理角度（度）: "))
        except (ValueError, EOFError):
            print("输入无效，退出。")
            return
    else:
        true_pos_deg = args.true_pos

    asyncio.run(calibrate(true_pos_deg, args.id))


if __name__ == "__main__":
    main()
