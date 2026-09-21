#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN FD 往返响应速度测试 (Round-Trip Latency Test)
=================================================
测试 moteus 控制器 CAN FD + BRS 模式下的往返响应延时。

前置条件 (Linux):
    sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on

用法:
    python canfd_speed_test.py                # 默认: ID=1, BRS, 10 次
    python canfd_speed_test.py --id 2         # 指定 CAN ID
    python canfd_speed_test.py --count 20     # 测试 20 次
    python canfd_speed_test.py --no-brs       # 禁用 BRS 对比测试
"""

import argparse
import asyncio
import sys
import time
import statistics

# ---------------------------------------------------------------------------
# moteus imports
# ---------------------------------------------------------------------------
sys.path.insert(0, './lib/python')
import moteus


async def run_test(target_id: int, count: int, disable_brs: bool,
                   channel: str, interface: str):
    """执行 CAN FD 往返响应速度测试。"""

    brs_label = "OFF (disabled)" if disable_brs else "ON (enabled)"
    print("=" * 60)
    print(f"  CAN FD 往返响应速度测试")
    print(f"  Target ID  : {target_id}")
    print(f"  BRS        : {brs_label}")
    print(f"  Interface  : {interface} / {channel}")
    print(f"  Test Count : {count}")
    print("=" * 60)

    # ---- 初始化 CAN 传输层 ----
    transport = moteus.PythonCan(
        interface=interface,
        channel=channel,
        fd=True,
        disable_brs=disable_brs,
    )

    qr = moteus.QueryResolution()
    qr.q_current = moteus.INT16

    controller = moteus.Controller(
        id=target_id,
        transport=transport,
        query_resolution=qr,
    )

    # ---- 发送 stop 清除故障状态 ----
    print("\n[1/3] 发送 Stop 指令清除故障...")
    try:
        await transport.cycle([controller.make_stop()])
    except Exception as e:
        print(f"  ? Stop 失败: {e}")

    await asyncio.sleep(0.1)

    # ---- 预热: 先跑 3 次丢弃，稳定总线 ----
    print("[2/3] 预热 (3 次查询)...")
    for _ in range(3):
        try:
            await asyncio.wait_for(
                transport.cycle([controller.make_query()]),
                timeout=0.5,
            )
        except Exception as e:
            print(f"  ? 预热查询失败: {e}")
            print("  请检查: 1) CAN 接口是否 UP  2) 电机 ID 是否正确  3) 电机是否上电")
            return

    await asyncio.sleep(0.05)

    # ---- 正式测试 ----
    print(f"[3/3] 开始测试 ({count} 次往返)...\n")

    latencies_us = []   # 单位: 微秒
    fail_count = 0

    for i in range(count):
        t_start = time.perf_counter()

        try:
            results = await asyncio.wait_for(
                transport.cycle([controller.make_query()]),
                timeout=0.5,
            )
            t_end = time.perf_counter()

            if not results:
                print(f"  #{i+1:3d}  ? 无响应")
                fail_count += 1
                continue

            latency_us = (t_end - t_start) * 1_000_000
            latencies_us.append(latency_us)

            # 解析响应数据
            r = results[0]
            vals = r.values
            pos = vals.get(moteus.Register.POSITION, None)
            volt = vals.get(moteus.Register.VOLTAGE, None)

            print(f"  #{i+1:3d}  ? {latency_us:8.1f} ?s"
                  f"  |  pos={pos if pos is not None else 'N/A':>10}"
                  f"  volt={f'{volt:.1f}V' if volt is not None else 'N/A':>7}")

        except asyncio.TimeoutError:
            print(f"  #{i+1:3d}  ? 超时")
            fail_count += 1
        except Exception as e:
            print(f"  #{i+1:3d}  ? 错误: {e}")
            fail_count += 1

        # 每次查询间短暂间隔，避免总线拥堵
        await asyncio.sleep(0.005)

    # ---- 统计结果 ----
    print("\n" + "=" * 60)
    print("  测试结果统计")
    print("=" * 60)

    if not latencies_us:
        print("  ? 没有成功的响应，无法统计。")
        return

    avg = statistics.mean(latencies_us)
    med = statistics.median(latencies_us)
    mn  = min(latencies_us)
    mx  = max(latencies_us)
    std = statistics.stdev(latencies_us) if len(latencies_us) > 1 else 0.0

    print(f"  成功次数 : {len(latencies_us)} / {count}")
    if fail_count:
        print(f"  失败次数 : {fail_count}")
    print(f"  平均延时 : {avg:10.1f} ?s  ({avg/1000:.3f} ms)")
    print(f"  中位延时 : {med:10.1f} ?s  ({med/1000:.3f} ms)")
    print(f"  最小延时 : {mn:10.1f} ?s  ({mn/1000:.3f} ms)")
    print(f"  最大延时 : {mx:10.1f} ?s  ({mx/1000:.3f} ms)")
    print(f"  标准差   : {std:10.1f} ?s")
    print(f"  BRS      : {brs_label}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='CAN FD 往返响应速度测试 (moteus)')
    parser.add_argument('--id', type=int, default=1,
                        help='目标电机 CAN ID (默认: 1)')
    parser.add_argument('--count', '-n', type=int, default=10,
                        help='测试次数 (默认: 10)')
    parser.add_argument('--no-brs', action='store_true',
                        help='禁用 BRS (Bit Rate Switch)')
    parser.add_argument('--channel', type=str, default='can0',
                        help='CAN 通道 (默认: can0)')
    parser.add_argument('--interface', type=str, default='socketcan',
                        help='CAN 接口类型 (默认: socketcan)')
    args = parser.parse_args()

    asyncio.run(run_test(
        target_id=args.id,
        count=args.count,
        disable_brs=args.no_brs,
        channel=args.channel,
        interface=args.interface,
    ))


if __name__ == '__main__':
    main()

