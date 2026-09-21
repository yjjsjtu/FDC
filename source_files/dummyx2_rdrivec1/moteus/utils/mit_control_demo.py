import asyncio
import math
import moteus

# Parameters from the configuration file:
# servo.pid_position.kp 200.000000
# servo.pid_position.kd 10.000000
CONFIG_KP = 200.0
CONFIG_KD = 10.0

# 生成平滑轨迹 (使用余弦插值，保证起步和停止时速度为0，非常顺滑)
def generate_trajectory(start_pos, end_pos, duration, dt):
    steps = int(duration / dt)
    traj = []
    for i in range(steps + 1):
        t = i * dt
        # 归一化时间 0 -> 1
        progress = t / duration
        # 余弦缓动曲线 0 -> 1
        ease = 0.5 * (1 - math.cos(math.pi * progress))
        # 位置
        p = start_pos + (end_pos - start_pos) * ease
        # 速度 (位置对时间的导数)
        v = (end_pos - start_pos) * 0.5 * (math.pi / duration) * math.sin(math.pi * progress)
        traj.append((p, v))
    return traj

async def main():
    # 初始化 moteus 控制器 (如果需要可以指定对应的 id)
    c = moteus.Controller(id=1)
    
    # 1. 启动前先清除报错和进入 stop 状态
    await c.set_stop()
    
    p_des_list = [0.88, 0.0]  # 在 1.0 圈和 0.0 圈之间来回运动
    target_index = 0
    
    kp_des = 50.0   
    kd_des = 2.0    
    t_ff = 0.0      # 如果不需要前馈力矩也可以设为0
    accel_limit_val = math.nan  # Moteus的加速度限制选项，设为 math.nan 表示不限制，也可以设置为浮点数(如 0.2)
    
    kp_scale = kp_des / CONFIG_KP
    kd_scale = kd_des / CONFIG_KD
    
    print(f"Sending MIT Format Command with Trajectory Planner:")
    print(f"  Kp:     {kp_des:.3f} (对应 Moteus scale: {kp_scale:.3f})")
    print(f"  Kd:     {kd_des:.3f} (对应 Moteus scale: {kd_scale:.3f})")
    print("Starting loop...")

    # 更新频率 (秒)
    dt = 0.02
    
    # 获取电机的当前处于的位置作为初始点
    state = await c.set_position(position=math.nan, query=True)
    current_pos = state.values.get(moteus.Register.POSITION, 0) if state else 0.0

    try:
        while True:
            target_p = p_des_list[target_index]
            
            # 生成一段时长为 1.5 秒的平滑轨迹
            duration = 1.5
            print(f"\nPlanning trajectory from {current_pos:.2f} to {target_p:.2f} over {duration}s")
            trajectory = generate_trajectory(current_pos, target_p, duration, dt)
            
            # 沿着轨迹逐点下发
            for p_des, v_des in trajectory:
                state = await c.set_position(
                    position=p_des,
                    velocity=v_des,
                    kp_scale=kp_scale,
                    kd_scale=kd_scale,
                    feedforward_torque=t_ff,
                    accel_limit=accel_limit_val,
                    query=True
                )
                
                # 打印当前电机的实际反馈状态
                if state:
                    pos = state.values.get(moteus.Register.POSITION, 0)
                    vel = state.values.get(moteus.Register.VELOCITY, 0)
                    trq = state.values.get(moteus.Register.TORQUE, 0)
                    print(f"Target: {p_des:.3f} | Pos: {pos:.3f} rev, Vel: {vel:.3f} rev/s, Trq: {trq:.3f} Nm", end="\r")
                
                await asyncio.sleep(dt)
            
            print(f"\nFinished trajectory generation. Waiting for motor to physically arrive...")
            
            # 动态等待，直到电机实际物理位置接近目标位置，或者超时(最多等 5 秒)
            wait_timeout = 5.0
            elapsed_wait = 0.0
            while elapsed_wait < wait_timeout:
                state = await c.set_position(
                    position=target_p,
                    velocity=0.0,
                    kp_scale=kp_scale,
                    kd_scale=kd_scale,
                    feedforward_torque=0.0,
                    accel_limit=accel_limit_val,
                    query=True
                )
                if state:
                    pos = state.values.get(moteus.Register.POSITION, 0)
                    vel = state.values.get(moteus.Register.VELOCITY, 0)
                    trq = state.values.get(moteus.Register.TORQUE, 0)
                    print(f"Waiting arrival | Pos: {pos:.3f} rev, Vel: {vel:.3f} rev/s, Trq: {trq:.3f} Nm", end="\r")
                    
                    # 误差在 0.01 圈(约3.6度)以内，并且速度极小，认为已经成功到位
                    if abs(pos - target_p) < 0.01 and abs(vel) < 0.1:
                        break
                        
                await asyncio.sleep(dt)
                elapsed_wait += dt
            
            print(f"\nMotor arrived at {target_p:.2f}. Holding position for 6.0 seconds...")
            # 到达后将当前位置更新，以免下一段轨迹出现跳变漂移
            current_pos = target_p 
            
            # 等待 3 秒 (您可以更改这里的 3.0 为更大的值)延长时间
            for _ in range(int(6.0 / dt)):
                await c.set_position(
                    position=target_p,
                    velocity=0.0,
                    kp_scale=kp_scale,
                    kd_scale=kd_scale,
                    feedforward_torque=0.0,
                    accel_limit=accel_limit_val,
                    query=False
                )
                await asyncio.sleep(dt)
            
            # 切换到下一个目标
            target_index = (target_index + 1) % len(p_des_list)
            
    except KeyboardInterrupt:
        print("\nStopping motor...")
        await c.set_stop()

if __name__ == '__main__':
    asyncio.run(main())

