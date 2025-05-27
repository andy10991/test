import numpy as np

# 参数设置
L = 10.0     # 立方体边长
r = 0.5      # 小球半径
g = 9.81     # 重力加速度
dt = 0.01    # 时间步长
restitution = 0.8  # 恢复系数 (0.8表示保留80%速度)

# 初始化
x, y, z = 5.0, 5.0, 5.0  # 初始位置（中心）
vx, vy, vz = 1.0, 0.0, 1.0  # 初始速度（初始y速度设为0）

max_steps = 10000  # 防止无限循环
step_count = 0

while step_count < max_steps:
    step_count += 1
    remaining_dt = dt  # 当前时间步剩余时间

    while remaining_dt > 1e-9:  # 处理子时间步
        # 计算加速度（只在自由运动时应用）
        vy -= g * remaining_dt  # 重力加速度向下

        # 初始化碰撞检测
        collision_axes = []
        collision_times = []

        # x轴碰撞检测
        if vx > 0:
            t_x = (L - r - x) / vx if vx != 0 else np.inf
        elif vx < 0:
            t_x = (x - r) / abs(vx) if vx != 0 else np.inf
        else:
            t_x = np.inf
        if t_x > 0:  # 只考虑未来的碰撞
            collision_axes.append('x')
            collision_times.append(t_x)

        # y轴碰撞检测
        if vy > 0:
            t_y = (L - r - y) / vy if vy != 0 else np.inf
        elif vy < 0:
            t_y = (y - r) / abs(vy) if vy != 0 else np.inf
        else:
            t_y = np.inf
        if t_y > 0:
            collision_axes.append('y')
            collision_times.append(t_y)

        # z轴碰撞检测
        if vz > 0:
            t_z = (L - r - z) / vz if vz != 0 else np.inf
        elif vz < 0:
            t_z = (z - r) / abs(vz) if vz != 0 else np.inf
        else:
            t_z = np.inf
        if t_z > 0:
            collision_axes.append('z')
            collision_times.append(t_z)

        # 找到最早发生的有效碰撞
        min_t = min(collision_times) if collision_times else np.inf
        min_t = min(min_t, remaining_dt)

        if min_t < remaining_dt:  # 发生碰撞
            # 更新位置到碰撞点
            x += vx * min_t
            y += vy * min_t
            z += vz * min_t

            # 处理碰撞响应
            if 'x' in collision_axes and collision_times[collision_axes.index('x')] == min_t:
                vx *= -restitution
            if 'y' in collision_axes and collision_times[collision_axes.index('y')] == min_t:
                vy *= -restitution
            if 'z' in collision_axes and collision_times[collision_axes.index('z')] == min_t:
                vz *= -restitution

            remaining_dt -= min_t
        else:  # 无碰撞，完成整个时间步
            x += vx * remaining_dt
            y += vy * remaining_dt
            z += vz * remaining_dt
            remaining_dt = 0

        # 位置边界约束（防止数值误差）
        x = np.clip(x, r, L - r)
        y = np.clip(y, r, L - r)
        z = np.clip(z, r, L - r)

    # 停止条件检测
    speed = np.sqrt(vx**2 + vy**2 + vz**2)
    if speed < 0.01:  # 当速度足够小时停止
        print(f"小球停止在位置 ({x:.2f}, {y:.2f}, {z:.2f})")
        break

if step_count == max_steps:
    print("达到最大步数仍未停止")

print("最终位置:", (x, y, z))
print("最终速度:", (vx, vy, vz))