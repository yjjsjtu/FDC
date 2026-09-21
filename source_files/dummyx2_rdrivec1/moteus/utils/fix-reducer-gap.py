#!/usr/bin/env python3
"""
减速比精确测量 — 单圈绝对编码器法
原理：命令整数 N 圈，若减速比精确，source1 单圈读数回到初始值。
      偏差量 = 实际 vs 命令的圈数误差。
用法: python3 fix-reducer-gap.py [--id 6] [--rev 10]
"""
import asyncio, argparse, math, time, moteus

parser = argparse.ArgumentParser()
parser.add_argument('--id',     type=int,   default=6)
parser.add_argument('--rev',    type=int,   default=10,  help='整数圈数（越多误差越明显）')
parser.add_argument('--vel',    type=float, default=0.5)
parser.add_argument('--acc',    type=float, default=0.5)
parser.add_argument('--torque', type=float, default=6.0)
ARGS = parser.parse_args()

CURRENT_RATIO = 1.0 / 30.0
SOURCE1_SIGN  = -1   # 配置中 motor_position.sources.1.sign = -1


async def raw_cycle(transport, ctrl, frame, timeout=2.0):
    return await asyncio.wait_for(transport.cycle([frame]), timeout=timeout)


async def read_pos(transport, ctrl):
    """读取当前 servo.position（同时刷新看门狗）"""
    for _ in range(5):
        try:
            res = await raw_cycle(transport, ctrl, ctrl.make_query())
            for r in res:
                if r.id == ARGS.id:
                    v = r.values.get(moteus.Register.POSITION)
                    if v is not None:
                        return v
        except Exception:
            await asyncio.sleep(0.05)
    return None


async def drain_diag(transport, ctrl):
    for _ in range(8):
        try:
            await asyncio.wait_for(
                transport.cycle([ctrl.make_diagnostic_read()]), timeout=0.12)
        except Exception:
            break


async def conf_set(transport, ctrl, key, value):
    """conf set（仅在 motor STOPPED 时调用）"""
    cmd = f'conf set {key} {value}\n'
    await raw_cycle(transport, ctrl,
                    ctrl.make_diagnostic_write(cmd.encode()))
    await asyncio.sleep(0.1)
    await drain_diag(transport, ctrl)


async def read_source1_fractional(transport, ctrl) -> float | None:
    """
    读取 source1 的单圈绝对角（motor 必须处于 STOP 状态）。
    multi-turn 计数被重置没关系，我们只需要单圈分数位置。
    """
    await conf_set(transport, ctrl, 'motor_position.output.source', 1)
    await asyncio.sleep(0.25)          # PLL 稳定
    pos = await read_pos(transport, ctrl)
    # 不切回——调用者负责
    return pos


async def switch_back_to_src0(transport, ctrl):
    await conf_set(transport, ctrl, 'motor_position.output.source', 0)
    await asyncio.sleep(0.1)


async def wait_settled(transport, ctrl, target, tol=0.005, timeout=120.0):
    """等待位置到达目标。只用 read_pos(make_query) 刷新看门狗，不发 nan！"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = await read_pos(transport, ctrl)
        if p is not None:
            err = abs(p - target)
            print(f'  pos={p:+.4f}  target={target:+.4f}  err={err:.4f}', end='\r')
            if err < tol:
                print()
                return True
        await asyncio.sleep(0.05)   # 50ms < servo.timeout_s(100ms)，看门狗安全
    print()
    return False


async def main():
    transport = moteus.get_singleton_transport()
    ctrl = moteus.Controller(id=ARGS.id, transport=transport)
    await asyncio.sleep(0.3)

    N = ARGS.rev   # 整数圈数
    print(f'减速比测量  ID={ARGS.id}  命令圈数={N}（整数）')
    print(f'原理：整数圈后 source1 单圈读数应回到初值，偏差 = 误差 × {N}')
    print('='*60)

    # ── Phase 1: 停止电机，读取初始 source1 单圈角 ──────────────────────────
    print('\n[Phase 1] 停止电机，读 source1 初始角...')
    await raw_cycle(transport, ctrl, ctrl.make_stop())
    await asyncio.sleep(0.2)

    # 注意：停止时 source0 初始位置
    p0_init = await read_pos(transport, ctrl)
    print(f'  source0 初始位置: {p0_init:+.6f} rev')

    f1_init = await read_source1_fractional(transport, ctrl)
    if f1_init is None:
        print('[ERROR] 读取 source1 失败，退出')
        return
    print(f'  source1 初始单圈: {f1_init:+.6f} rev  ← 运动后应回到此值（若减速比精确）')
    await switch_back_to_src0(transport, ctrl)

    # ── Phase 2: 使能，移动整数 N 圈 ────────────────────────────────────────
    target = (p0_init or 0.0) + N
    print(f'\n[Phase 2] 使能并移动 {N} 圈 → target={target:+.2f}...')

    # 先 enable（nan 模式），再发目标
    try:
        await raw_cycle(transport, ctrl, ctrl.make_position(
            position=math.nan, velocity=0.0,
            maximum_torque=ARGS.torque, query=True))
        await asyncio.sleep(0.1)
    except Exception as e:
        print(f'  [WARN] enable: {e}')

    try:
        await raw_cycle(transport, ctrl, ctrl.make_position(
            position=target,
            velocity=0.0,
            maximum_torque=ARGS.torque,
            velocity_limit=ARGS.vel,
            accel_limit=ARGS.acc,
            query=True))
    except Exception as e:
        print(f'[ERROR] 位置命令: {e}')
        return

    ok = await wait_settled(transport, ctrl, target)
    if not ok:
        print('[WARN] 运动超时')
    await asyncio.sleep(1.0)

    p0_final = await read_pos(transport, ctrl)
    print(f'  source0 终止位置: {p0_final:+.6f} rev  (delta={p0_final - p0_init:+.4f})')

    # ── Phase 3: 停止，读 source1 终止角 ────────────────────────────────────
    print('\n[Phase 3] 停止电机，读 source1 终止角...')
    await raw_cycle(transport, ctrl, ctrl.make_stop())
    await asyncio.sleep(0.3)

    f1_final = await read_source1_fractional(transport, ctrl)
    if f1_final is None:
        print('[ERROR] 读取 source1 终止值失败')
        await switch_back_to_src0(transport, ctrl)
        return
    print(f'  source1 终止单圈: {f1_final:+.6f} rev')
    await switch_back_to_src0(transport, ctrl)

    # ── 计算 ─────────────────────────────────────────────────────────────────
    # delta 为 source1 单圈读数变化量，用绕圈修正到 (-0.5, 0.5)
    delta = f1_final - f1_init
    while delta >  0.5: delta -= 1.0
    while delta < -0.5: delta += 1.0

    # 实际输出圈数 = 命令圈数 + delta × SOURCE1_SIGN
    # 原理：正方向运动时 source1（sign=-1）应减少 N，
    #       若 delta>0 说明 source1 减少不够，即输出欠行程
    actual_output = N + delta * SOURCE1_SIGN
    new_ratio = CURRENT_RATIO * actual_output / N

    print('\n' + '='*60)
    print(f'  source1 初始单圈 : {f1_init:+.6f} rev')
    print(f'  source1 终止单圈 : {f1_final:+.6f} rev')
    print(f'  单圈偏差 (delta) : {delta:+.6f} rev  ({delta*360:+.3f}°)')
    print(f'')
    print(f'  命令圈数         : {N}')
    print(f'  实测实际输出     : {actual_output:+.6f} rev')
    print(f'  误差             : {actual_output-N:+.6f} rev  ({(actual_output-N)*360:+.2f}°)')
    print(f'')
    print(f'  当前 ratio       : {CURRENT_RATIO:.8f}  (1/{1/CURRENT_RATIO:.2f})')
    print(f'  建议新 ratio     : {new_ratio:.8f}  (1/{1/new_ratio:.2f})')
    print('='*60)

    err_pct = (actual_output - N) / N * 100
    if abs(err_pct) < 0.2:
        print(f'\n✓ 误差 {err_pct:+.3f}% — 减速比配置准确！')
    else:
        print(f'\n⚠ 误差 {err_pct:+.3f}%，修正命令：')
        print(f'  d cfg set motor_position.rotor_to_output_ratio {new_ratio:.8f}')
        print(f'  d cfg write')
        print(f'\n  修正后重新运行本脚本验证')

asyncio.run(main())
