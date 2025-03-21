import numpy as np
import matplotlib.pyplot as plt
import os
import shutil

# 字体大小设置（可调节）
TITLE_FONT_SIZE = 22
LABEL_FONT_SIZE = 18
LEGEND_FONT_SIZE = 'medium'

# Parameters start
r= 0.3
r_cbf = 0.4
X = 50
Y = 50
EPSILON = 0.1
H = 0.2
C1_ALPHA = 20
C2_ALPHA = 2 * np.sqrt(C1_ALPHA)
N = 50  # Number of UAV
M = 2  # Space dimensions
D = 15  # Desired distance among UAV
K = 1.2  # Scaling factor
R = K * D  # Interaction range
DELTA_T = 0.009
A = 5
B = 5
C = np.abs(A - B) / np.sqrt(4 * A * B)
ITERATION = 1000
SNAPSHOT_COUNT = 8  # Desired number of snapshots
SNAPSHOT_INTERVAL = ITERATION // SNAPSHOT_COUNT
POSITION_X = np.zeros([N, ITERATION])
POSITION_Y = np.zeros([N, ITERATION])

# 修改节点初始化，确保不在原点
nodes = (np.random.rand(N, M) * (X - 1)) + 0.5  # 初始化在 [0.5, X-0.5) 范围内
nodes_old = nodes.copy()
nodes_velocity_p = np.zeros([N, M])
velocity_magnitudes = np.zeros([N, ITERATION])
connectivity = np.zeros([ITERATION, 1])
q_mt = np.array([200, 25])  # Target point
c1_mt = 1.1
c2_mt = 2 * np.sqrt(c1_mt)

obstacles = np.array([[100, 25]])  # Obstacle positions
Rk = np.array([15])  # Obstacle radii
num_obstacles = obstacles.shape[0]
c1_beta = 1500
c2_beta = 2 * np.sqrt(c1_beta)
r_prime = 0.22 * K * R
d_prime = 15
d_beta = 15  # 添加缺失的d_beta定义
center_of_mass = np.zeros([ITERATION, M])
# Parameters end
def sigma_norm(z):
    val = EPSILON * (z ** 2)
    val = np.sqrt(1 + val) - 1
    val = val / EPSILON
    return val

def create_adjacency_matrix():
    adjacency_matrix = np.zeros([N, N])
    for i in range(N):
        for j in range(N):
            if i != j:
                distance = np.linalg.norm(nodes[j] - nodes[i])
                if distance <= R:
                    adjacency_matrix[i, j] = 1
    return adjacency_matrix

def plot_snapshot(t, snapshot_dir, colors):
    """
    在指定的迭代时刻绘制快照并保存为图像文件
    """
    plt.figure(figsize=(8, 8))  # 统一图形尺寸为正方形
    ax = plt.gca()
    
    # 绘制目标点
    plt.plot(q_mt[0], q_mt[1], 'go', markersize=10, label='Target Point')  # 'go' -> 绿色圆点
    
    # 绘制障碍物
    for i in range(num_obstacles):
        obstacle_circle = plt.Circle((obstacles[i, 0], obstacles[i, 1]), Rk[i],
                                     color='red', alpha=0.5, label='Obstacle' if i == 0 else "")
        ax.add_artist(obstacle_circle)
        # 'red' 表示红色
    
    # 绘制UAV节点
    plt.plot(nodes[:, 0], nodes[:, 1], 'r*', markersize=6, label='UAV')  # 'ro' -> 红色圆点
    
    # 绘制连接线
    for i in range(N):
        for j in range(i + 1, N):  # 只绘制一次连接线
            distance = np.linalg.norm(nodes[j] - nodes[i])
            if distance <= R:
                plt.plot([nodes[i, 0], nodes[j, 0]],
                         [nodes[i, 1], nodes[j, 1]],
                         'b-', lw=0.5)  # 'b-' -> 蓝色实线
    
    # 动态计算绘图范围并调整为正方形
    margin = 20  # 边距
    all_x = np.concatenate((nodes[:, 0], obstacles[:, 0], [q_mt[0]]))
    all_y = np.concatenate((nodes[:, 1], obstacles[:, 1], [q_mt[1]]))
    min_x = np.min(all_x) - margin
    max_x = np.max(all_x) + margin
    min_y = np.min(all_y) - margin
    max_y = np.max(all_y) + margin
    # 计算x和y的跨度
    x_range = max_x - min_x
    y_range = max_y - min_y
    max_range = max(x_range, y_range)
    # 计算中心点
    x_center = (min_x + max_x) / 2
    y_center = (min_y + max_y) / 2
    # 设置新的x和y范围以确保正方形
    new_min_x = x_center - max_range / 2
    new_max_x = x_center + max_range / 2
    new_min_y = y_center - max_range / 2
    new_max_y = y_center + max_range / 2
    plt.xlim(new_min_x, new_max_x)
    plt.ylim(new_min_y, new_max_y)
    ax.set_aspect('equal', adjustable='box')  # 保持比例一致
    # 设置标题和坐标轴标签的字体大小
    plt.title(f"Formation at {float(t)*0.1:.2f} sec", fontsize=TITLE_FONT_SIZE)  # 修改标题字体大小
    plt.xlabel("Px(m)", fontsize=LABEL_FONT_SIZE)  # 修改X轴标签字体大小
    plt.ylabel("Py(m)", fontsize=LABEL_FONT_SIZE)  # 修改Y轴标签字体大小
    # 只在第一次绘制障碍物时添加图例
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc='upper right', fontsize=LEGEND_FONT_SIZE)
    plt.grid(True)
    # 保存快照图像
    snapshot_path = os.path.join(snapshot_dir, f"snapshot_{t:05d}.png")
    plt.savefig(snapshot_path, dpi=300)  # 设置相同的dpi
    plt.close()
    print(f"Saved snapshot at iteration {t} to {snapshot_path}")

def plot_trajectories_up_to_time(t, snapshot_dir, colors):
    """
    绘制所有节点从开始到指定迭代时刻的移动轨迹，并保存为图像文件
    参数：
    - t: 当前迭代次数
    - snapshot_dir: 轨迹图保存目录
    - colors: 智能体颜色列表
    """
    plt.figure(figsize=(8, 8))  # 统一图形尺寸为正方形
    ax = plt.gca()
    
    for i in range(N):
        # 检查智能体是否有多个位置点且初始位置不在原点
        if t > 0 and (POSITION_X[i,0] != 0 or POSITION_Y[i,0] != 0):
            plt.plot(POSITION_X[i, :t+1], POSITION_Y[i, :t+1], '-', 
                     color=colors[i], alpha=0.6, linewidth=1, 
                     label='Trajectories' if i == 0 else "")
        else:
            plt.plot(POSITION_X[i, t], POSITION_Y[i, t], '*', 
                     color=colors[i], label='Trajectories' if i == 0 else "")
    # 绘制目标点
    plt.plot(q_mt[0], q_mt[1], 'go', markersize=10, label='Target Point')  # 'go' -> 绿色圆点
    # 绘制障碍物
    for i in range(num_obstacles):
        obstacle_circle = plt.Circle((obstacles[i, 0], obstacles[i, 1]), Rk[i],
                                     color='red', alpha=0.5, label='Obstacle' if i == 0 else "")
        ax.add_artist(obstacle_circle)
        # 'red' 表示红色
    # 绘制连接线（截至当前迭代）
    for i in range(N):
        for j in range(i + 1, N):
            distance = np.linalg.norm(nodes[j] - nodes[i])
            if distance <= R:
                plt.plot([nodes[i, 0], nodes[j, 0]],
                         [nodes[i, 1], nodes[j, 1]],
                         'b-', lw=0.5)  # 'b-' -> 蓝色实线
    
    # 动态计算绘图范围并调整为正方形
    margin = 20  # 边距
    all_x = np.concatenate((POSITION_X[:, :t+1].flatten(), obstacles[:, 0], [q_mt[0]]))
    all_y = np.concatenate((POSITION_Y[:, :t+1].flatten(), obstacles[:, 1], [q_mt[1]]))
    min_x = np.min(all_x) - margin
    max_x = np.max(all_x) + margin
    min_y = np.min(all_y) - margin
    max_y = np.max(all_y) + margin
    
    # 计算x和y的跨度
    x_range = max_x - min_x
    y_range = max_y - min_y
    max_range = max(x_range, y_range)
    
    # 计算中心点
    x_center = (min_x + max_x) / 2
    y_center = (min_y + max_y) / 2
    
    # 设置新的x和y范围以确保正方形
    new_min_x = x_center - max_range / 2
    new_max_x = x_center + max_range / 2
    new_min_y = y_center - max_range / 2
    new_max_y = y_center + max_range / 2
    
    plt.xlim(new_min_x, new_max_x)
    plt.ylim(new_min_y, new_max_y)
    ax.set_aspect('equal', adjustable='box')  # 保持比例一致
    
    # 设置标题和坐标轴标签的字体大小
    plt.title(f"Trajectories at {float(t)*0.1:.2f} sec", fontsize=TITLE_FONT_SIZE)  # 修改标题字体大小
    plt.xlabel("Px", fontsize=LABEL_FONT_SIZE)  # 修改X轴标签字体大小
    plt.ylabel("Py", fontsize=LABEL_FONT_SIZE)  # 修改Y轴标签字体大小
    
    # 只在第一次绘制障碍物时添加图例
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc='upper right', fontsize=LEGEND_FONT_SIZE)
    
    plt.grid(True)
    
    # 保存轨迹图像
    trajectories_path = os.path.join(snapshot_dir, f"trajectories_{t:05d}.png")
    plt.savefig(trajectories_path, dpi=300)  # 设置相同的dpi
    plt.close()
    print(f"Saved trajectories plot up to iteration {t} to {trajectories_path}")

def plot_final_trajectories(snapshot_dir, colors):
    """
    绘制所有节点的最终移动轨迹，并保存为图像文件
    """
    plt.figure(figsize=(8, 8))  # 统一图形尺寸为正方形
    ax = plt.gca()
    
    for i in range(N):
        plt.plot(POSITION_X[i, :], POSITION_Y[i, :], '-', color=colors[i], alpha=0.6, linewidth=1, label=f'Node {i+1}' if i == 0 else "")
    
    # 绘制目标点
    plt.plot(q_mt[0], q_mt[1], 'go', markersize=10, label='Target Point')  # 'go' -> 绿色圆点
    
    # 绘制障碍物
    for i in range(num_obstacles):
        obstacle_circle = plt.Circle((obstacles[i, 0], obstacles[i, 1]), Rk[i],
                                     color='red', alpha=0.5, label='Obstacle' if i == 0 else "")
        ax.add_artist(obstacle_circle)
        # 'red' 表示红色
    
    # 绘制连接线（截至最终迭代）
    for i in range(N):
        for j in range(i + 1, N):
            distance = np.linalg.norm(nodes[j] - nodes[i])
            if distance <= R:
                plt.plot([nodes[i, 0], nodes[j, 0]],
                         [nodes[i, 1], nodes[j, 1]],
                         'b-', lw=0.5)  # 'b-' -> 蓝色实线
    
    # 动态计算绘图范围并调整为正方形
    margin = 20  # 边距
    all_x = np.concatenate((POSITION_X[:, :].flatten(), obstacles[:, 0], [q_mt[0]]))
    all_y = np.concatenate((POSITION_Y[:, :].flatten(), obstacles[:, 1], [q_mt[1]]))
    min_x = np.min(all_x) - margin
    max_x = np.max(all_x) + margin
    min_y = np.min(all_y) - margin
    max_y = np.max(all_y) + margin
    
    # 计算x和y的跨度
    x_range = max_x - min_x
    y_range = max_y - min_y
    max_range = max(x_range, y_range)
    
    # 计算中心点
    x_center = (min_x + max_x) / 2
    y_center = (min_y + max_y) / 2
    
    # 设置新的x和y范围以确保正方形
    new_min_x = x_center - max_range / 2
    new_max_x = x_center + max_range / 2
    new_min_y = y_center - max_range / 2
    new_max_y = y_center + max_range / 2
    
    plt.xlim(new_min_x, new_max_x)
    plt.ylim(new_min_y, new_max_y)
    ax.set_aspect('equal', adjustable='box')  # 保持比例一致
    
    # 设置标题和坐标轴标签的字体大小
    plt.title("Final Trajectories of UAVs", fontsize=TITLE_FONT_SIZE)  # 修改标题字体大小
    plt.xlabel("Px", fontsize=LABEL_FONT_SIZE)  # 修改X轴标签字体大小
    plt.ylabel("Py", fontsize=LABEL_FONT_SIZE)  # 修改Y轴标签字体大小
    
    # 只在第一次绘制障碍物时添加图例
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc='upper right', fontsize=LEGEND_FONT_SIZE)
    
    plt.grid(True)
    
    # 保存最终轨迹图像
    final_trajectories_path = os.path.join(snapshot_dir, "final_trajectories.png")
    plt.savefig(final_trajectories_path, dpi=300)  # 设置相同的dpi
    plt.close()
    print(f"Saved final trajectories plot to {final_trajectories_path}")

def plot_velocity(ax):
    """
    绘制所有节点的速度幅值随时间的变化
    """
    for i in range(N):
        velocity_i = velocity_magnitudes[i, :]
        ax.plot(velocity_i, label=f'Node {i + 1}')
        # 颜色可以通过指定颜色参数来修改，例如：color='C{}'.format(i)
    ax.set_title("Velocity Magnitudes Over Time", fontsize=TITLE_FONT_SIZE)
    ax.set_xlabel("Iteration", fontsize=LABEL_FONT_SIZE)
    ax.set_ylabel("Velocity Magnitude", fontsize=LABEL_FONT_SIZE)
    if N <= 10:
        ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE)
    ax.grid(True)
    ax.set_aspect('auto', adjustable='box')

def plot_connectivity(ax):
    """
    绘制系统的连通性随时间的变化
    """
    ax.plot(connectivity, 'b-', label='Connectivity')  # 'b-' -> 蓝色实线
    ax.set_title("Connectivity Over Time", fontsize=TITLE_FONT_SIZE)
    ax.set_xlabel("Iteration", fontsize=LABEL_FONT_SIZE)
    ax.set_ylabel("Average Connectivity (Matrix Rank / N)", fontsize=LABEL_FONT_SIZE)
    ax.grid(True)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE)
    ax.set_aspect('auto', adjustable='box')

def plot_center_of_mass(ax):
    """
    绘制系统质心的移动轨迹
    """
    ax.plot(center_of_mass[:, 0], center_of_mass[:, 1], 'k-', label='Center of Mass Trajectory')  # 'k-' -> 黑色实线
    ax.set_title("Center of Mass Trajectory", fontsize=TITLE_FONT_SIZE)
    ax.set_xlabel("X Position", fontsize=LABEL_FONT_SIZE)
    ax.set_ylabel("Y Position", fontsize=LABEL_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE)
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')

def bump_function(z):
    if 0 <= z < H:
        return 1
    elif H <= z < 1:
        val = (z - H) / (1 - H)
        val = np.cos(np.pi * val)
        val = (1 + val) / 2
        return val
    else:
        return 0

def sigma_1(z):
    val = 1 + z ** 2
    val = np.sqrt(val)
    val = z / val
    return val

def phi(z):
    val_1 = A + B
    val_2 = sigma_1(z + C)
    val_3 = A - B
    val = val_1 * val_2 + val_3
    val = val / 2
    return val

def phi_alpha(z):
    input_1 = z / sigma_norm(R)  # Sigma norm of R is R_alpha
    input_2 = z - sigma_norm(D)  # Sigma norm of D is D_alpha
    val_1 = bump_function(input_1)
    val_2 = phi(input_2)
    val = val_1 * val_2
    return val

def phi_beta(z):
    val1 = bump_function(z / d_beta)
    val2 = sigma_1(z - d_beta) - 1
    return val1 * val2

def get_a_ij(i, j):
    val_1 = nodes[j] - nodes[i]
    norm = np.linalg.norm(val_1)
    val_2 = sigma_norm(norm) / sigma_norm(R)
    val = bump_function(val_2)
    return val

def get_n_ij(i, j):
    val_1 = nodes[j] - nodes[i]
    norm = np.linalg.norm(val_1)
    val_2 = 1 + EPSILON * norm ** 2
    val = val_1 / np.sqrt(val_2)
    return val

def get_u_i(i, mu, a_k, P, p_i_k, q_i_k, b_i_k, n_i_k, old_position):
    sum_1 = np.array([0.0, 0.0])
    sum_2 = np.array([0.0, 0.0])
    for j in range(N):
        distance = np.linalg.norm(nodes[j] - nodes[i])
        if distance <= R:
            phi_alpha_val = phi_alpha(sigma_norm(distance))
            sum_1 += phi_alpha_val * get_n_ij(i, j)

            val_2 = nodes_velocity_p[j] - nodes_velocity_p[i]
            sum_2 += get_a_ij(i, j) * val_2
    # 计算u_i
    val = (C1_ALPHA * sum_1 +
           C2_ALPHA * sum_2 -
           c1_mt * (old_position - q_mt) +
           c1_beta * phi_beta(np.linalg.norm(q_i_k - old_position)) * n_i_k +
           c2_beta * b_i_k * (p_i_k - nodes_velocity_p[i]))
    return val

def get_positions(snapshot_dir, colors, snapshot_times):
    for t in range(ITERATION):
        adjacency_matrix = create_adjacency_matrix()
        connectivity[t] = (1 / N) * np.linalg.matrix_rank(adjacency_matrix)
        center_of_mass[t] = np.array([np.mean(nodes[:, 0]), np.mean(nodes[:, 1])])

        if t == 0:
            # 记录初始位置
            POSITION_X[:, t] = nodes[:, 0]
            POSITION_Y[:, t] = nodes[:, 1]
            # 打印部分智能体的初始位置
            print(f"Initial Positions at t={t}: {nodes[:5]}")
            # 绘制初始快照
            plot_snapshot(t, snapshot_dir, colors)
            # 绘制初始轨迹图
            plot_trajectories_up_to_time(t, snapshot_dir, colors)
            snapshot_times.append(t)
        else:
            for i in range(N):
                old_velocity = nodes_velocity_p[i, :]
                old_position = np.array([POSITION_X[i, t - 1],
                                         POSITION_Y[i, t - 1]])

                # 处理障碍物相关计算
                distance_to_obstacle = np.linalg.norm(old_position - obstacles[0])
                if distance_to_obstacle == 0:
                    mu = 0
                else:
                    mu = Rk[0] / distance_to_obstacle
                a_k = (old_position - obstacles[0]) / (distance_to_obstacle + 1e-8)  # 防止除零
                P = 1 - np.outer(a_k, a_k)
                p_i_k = mu * P @ old_velocity
                q_i_k = mu * old_position + (1 - mu) * obstacles[0]
                b_i_k = bump_function(sigma_norm(np.linalg.norm(q_i_k - old_position)) / d_beta)
                n_i_k = (q_i_k - old_position) / (np.sqrt(1 + EPSILON *
                                                        (np.linalg.norm(q_i_k - old_position)) ** 2) + 1e-8)

                u_i = get_u_i(i, mu, a_k, P, p_i_k, q_i_k, b_i_k, n_i_k, old_position)
                new_position = old_position + DELTA_T * old_velocity + (DELTA_T ** 2 / 2) * u_i
                new_velocity = (new_position - old_position) / DELTA_T

                POSITION_X[i, t] = new_position[0]
                POSITION_Y[i, t] = new_position[1]
                nodes_velocity_p[i, :] = new_velocity
                nodes[i, :] = new_position
                velocity_magnitudes[i, t] = np.linalg.norm(new_velocity)

        # 判断是否需要绘制快照
        if (t + 1) % SNAPSHOT_INTERVAL == 0 or (t + 1) == ITERATION:
            plot_snapshot(t + 1, snapshot_dir, colors)
            snapshot_times.append(t + 1)
            # 生成对应的轨迹图
            plot_trajectories_up_to_time(t + 1, snapshot_dir, colors)

        # 可选：打印进度和节点位置
        if (t + 1) % 100 == 0:
            print(f"Iteration {t + 1}/{ITERATION} completed.")
            print(f"Sample Node Positions at Iteration {t + 1}: {nodes[:5]}")

    print("Position updates and snapshot generation completed.")

def plot_final_trajectories(snapshot_dir, colors):
    """
    绘制所有节点的最终移动轨迹，并保存为图像文件
    """
    plt.figure(figsize=(8, 8))  # 统一图形尺寸为正方形
    ax = plt.gca()
    
    for i in range(N):
        plt.plot(POSITION_X[i, :], POSITION_Y[i, :], '-', color=colors[i], alpha=0.6, linewidth=1, label=f'Node {i+1}' if i == 0 else "")
    
    # 绘制目标点
    plt.plot(q_mt[0], q_mt[1], 'go', markersize=10, label='Target Point')  # 'go' -> 绿色圆点
    
    # 绘制障碍物
    for i in range(num_obstacles):
        obstacle_circle = plt.Circle((obstacles[i, 0], obstacles[i, 1]), Rk[i],
                                     color='red', alpha=0.5, label='Obstacle' if i == 0 else "")
        ax.add_artist(obstacle_circle)
        # 'red' 表示红色
    
    # 绘制连接线（截至最终迭代）
    for i in range(N):
        for j in range(i + 1, N):
            distance = np.linalg.norm(nodes[j] - nodes[i])
            if distance <= R:
                plt.plot([nodes[i, 0], nodes[j, 0]],
                         [nodes[i, 1], nodes[j, 1]],
                         'b-', lw=0.5)  # 'b-' -> 蓝色实线
    
    # 动态计算绘图范围并调整为正方形
    margin = 20  # 边距
    all_x = np.concatenate((POSITION_X[:, :].flatten(), obstacles[:, 0], [q_mt[0]]))
    all_y = np.concatenate((POSITION_Y[:, :].flatten(), obstacles[:, 1], [q_mt[1]]))
    min_x = np.min(all_x) - margin
    max_x = np.max(all_x) + margin
    min_y = np.min(all_y) - margin
    max_y = np.max(all_y) + margin
    
    # 计算x和y的跨度
    x_range = max_x - min_x
    y_range = max_y - min_y
    max_range = max(x_range, y_range)
    
    # 计算中心点
    x_center = (min_x + max_x) / 2
    y_center = (min_y + max_y) / 2
    
    # 设置新的x和y范围以确保正方形
    new_min_x = x_center - max_range / 2
    new_max_x = x_center + max_range / 2
    new_min_y = y_center - max_range / 2
    new_max_y = y_center + max_range / 2
    
    plt.xlim(new_min_x, new_max_x)
    plt.ylim(new_min_y, new_max_y)
    ax.set_aspect('equal', adjustable='box')  # 保持比例一致
    
    # 设置标题和坐标轴标签的字体大小
    plt.title("Final Trajectories of UAVs", fontsize=TITLE_FONT_SIZE)  # 修改标题字体大小
    plt.xlabel("Px", fontsize=LABEL_FONT_SIZE)  # 修改X轴标签字体大小
    plt.ylabel("Py", fontsize=LABEL_FONT_SIZE)  # 修改Y轴标签字体大小
    
    # 只在第一次绘制障碍物时添加图例
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc='upper right', fontsize=LEGEND_FONT_SIZE)
    
    plt.grid(True)
    
    # 保存最终轨迹图像
    final_trajectories_path = os.path.join(snapshot_dir, "final_trajectories.png")
    plt.savefig(final_trajectories_path, dpi=300)  # 设置相同的dpi
    plt.close()
    print(f"Saved final trajectories plot to {final_trajectories_path}")

# 其他函数保持不变（plot_velocity, plot_connectivity, plot_center_of_mass, bump_function, sigma_1, phi, phi_alpha, phi_beta, get_a_ij, get_n_ij, get_u_i）

def plot_velocity(ax):
    """
    绘制所有节点的速度幅值随时间的变化
    """
    for i in range(N):
        velocity_i = velocity_magnitudes[i, :]
        ax.plot(velocity_i, label=f'Node {i + 1}')
        # 颜色可以通过指定颜色参数来修改，例如：color='C{}'.format(i)
    ax.set_title("Velocity Magnitudes Over Time", fontsize=TITLE_FONT_SIZE)
    ax.set_xlabel("Iteration", fontsize=LABEL_FONT_SIZE)
    ax.set_ylabel("Velocity Magnitude", fontsize=LABEL_FONT_SIZE)
    if N <= 10:
        ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE)
    ax.grid(True)
    ax.set_aspect('auto', adjustable='box')

def plot_connectivity(ax):
    """
    绘制系统的连通性随时间的变化
    """
    ax.plot(connectivity, 'b-', label='Connectivity')  # 'b-' -> 蓝色实线
    ax.set_title("Connectivity Over Time", fontsize=TITLE_FONT_SIZE)
    ax.set_xlabel("Iteration", fontsize=LABEL_FONT_SIZE)
    ax.set_ylabel("Average Connectivity (Matrix Rank / N)", fontsize=LABEL_FONT_SIZE)
    ax.grid(True)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE)
    ax.set_aspect('auto', adjustable='box')

def plot_center_of_mass(ax):
    """
    绘制系统质心的移动轨迹
    """
    ax.plot(center_of_mass[:, 0], center_of_mass[:, 1], 'k-', label='Center of Mass Trajectory')  # 'k-' -> 黑色实线
    ax.set_title("Center of Mass Trajectory", fontsize=TITLE_FONT_SIZE)
    ax.set_xlabel("X Position", fontsize=LABEL_FONT_SIZE)
    ax.set_ylabel("Y Position", fontsize=LABEL_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE)
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')

# 定义其余函数（bump_function, sigma_1, phi, phi_alpha, phi_beta, get_a_ij, get_n_ij, get_u_i）保持不变

def bump_function(z):
    if 0 <= z < H:
        return 1
    elif H <= z < 1:
        val = (z - H) / (1 - H)
        val = np.cos(np.pi * val)
        val = (1 + val) / 2
        return val
    else:
        return 0

def sigma_1(z):
    val = 1 + z ** 2
    val = np.sqrt(val)
    val = z / val
    return val

def phi(z):
    val_1 = A + B
    val_2 = sigma_1(z + C)
    val_3 = A - B
    val = val_1 * val_2 + val_3
    val = val / 2
    return val

def phi_alpha(z):
    input_1 = z / sigma_norm(R)  # Sigma norm of R is R_alpha
    input_2 = z - sigma_norm(D)  # Sigma norm of D is D_alpha
    val_1 = bump_function(input_1)
    val_2 = phi(input_2)
    val = val_1 * val_2
    return val

def phi_beta(z):
    val1 = bump_function(z / d_beta)
    val2 = sigma_1(z - d_beta) - 1
    return val1 * val2

def get_a_ij(i, j):
    val_1 = nodes[j] - nodes[i]
    norm = np.linalg.norm(val_1)
    val_2 = sigma_norm(norm) / sigma_norm(R)
    val = bump_function(val_2)
    return val

def get_n_ij(i, j):
    val_1 = nodes[j] - nodes[i]
    norm = np.linalg.norm(val_1)
    val_2 = 1 + EPSILON * norm ** 2
    val = val_1 / np.sqrt(val_2)
    return val

def get_u_i(i, mu, a_k, P, p_i_k, q_i_k, b_i_k, n_i_k, old_position):
    sum_1 = np.array([0.0, 0.0])
    sum_2 = np.array([0.0, 0.0])
    for j in range(N):
        distance = np.linalg.norm(nodes[j] - nodes[i])
        if distance <= R:
            phi_alpha_val = phi_alpha(sigma_norm(distance))
            sum_1 += phi_alpha_val * get_n_ij(i, j)

            val_2 = nodes_velocity_p[j] - nodes_velocity_p[i]
            sum_2 += get_a_ij(i, j) * val_2
    # 计算u_i
    val = (C1_ALPHA * sum_1 +
           C2_ALPHA * sum_2 -
           c1_mt * (old_position - q_mt) +
           c1_beta * phi_beta(np.linalg.norm(q_i_k - old_position)) * n_i_k +
           c2_beta * b_i_k * (p_i_k - nodes_velocity_p[i]))
    return val

def get_positions(snapshot_dir, colors, snapshot_times):
    for t in range(ITERATION):
        adjacency_matrix = create_adjacency_matrix()
        connectivity[t] = (1 / N) * np.linalg.matrix_rank(adjacency_matrix)
        center_of_mass[t] = np.array([np.mean(nodes[:, 0]), np.mean(nodes[:, 1])])

        if t == 0:
            # 记录初始位置
            POSITION_X[:, t] = nodes[:, 0]
            POSITION_Y[:, t] = nodes[:, 1]
            # 打印部分智能体的初始位置
            print(f"Initial Positions at t={t}: {nodes[:5]}")
            # 绘制初始快照
            plot_snapshot(t, snapshot_dir, colors)
            # 绘制初始轨迹图
            plot_trajectories_up_to_time(t, snapshot_dir, colors)
            snapshot_times.append(t)
        else:
            for i in range(N):
                old_velocity = nodes_velocity_p[i, :]
                old_position = np.array([POSITION_X[i, t - 1],
                                         POSITION_Y[i, t - 1]])

                # 处理障碍物相关计算
                distance_to_obstacle = np.linalg.norm(old_position - obstacles[0])
                if distance_to_obstacle == 0:
                    mu = 0
                else:
                    mu = Rk[0] / distance_to_obstacle
                a_k = (old_position - obstacles[0]) / (distance_to_obstacle + 1e-8)  # 防止除零
                P = 1 - np.outer(a_k, a_k)
                p_i_k = mu * P @ old_velocity
                q_i_k = mu * old_position + (1 - mu) * obstacles[0]
                b_i_k = bump_function(sigma_norm(np.linalg.norm(q_i_k - old_position)) / d_beta)
                n_i_k = (q_i_k - old_position) / (np.sqrt(1 + EPSILON *
                                                        (np.linalg.norm(q_i_k - old_position)) ** 2) + 1e-8)

                u_i = get_u_i(i, mu, a_k, P, p_i_k, q_i_k, b_i_k, n_i_k, old_position)
                new_position = old_position + DELTA_T * old_velocity + (DELTA_T ** 2 / 2) * u_i
                new_velocity = (new_position - old_position) / DELTA_T

                POSITION_X[i, t] = new_position[0]
                POSITION_Y[i, t] = new_position[1]
                nodes_velocity_p[i, :] = new_velocity
                nodes[i, :] = new_position
                velocity_magnitudes[i, t] = np.linalg.norm(new_velocity)

        # 判断是否需要绘制快照
        if (t + 1) % SNAPSHOT_INTERVAL == 0 or (t + 1) == ITERATION:
            plot_snapshot(t + 1, snapshot_dir, colors)
            snapshot_times.append(t + 1)
            # 生成对应的轨迹图
            plot_trajectories_up_to_time(t + 1, snapshot_dir, colors)

        # 可选：打印进度和节点位置
        if (t + 1) % 100 == 0:
            print(f"Iteration {t + 1}/{ITERATION} completed.")
            print(f"Sample Node Positions at Iteration {t + 1}: {nodes[:5]}")

    print("Position updates and snapshot generation completed.")

# 主程序部分
if __name__ == "__main__":
    # 设置快照保存目录
    snapshot_directory = "snapshots"

    # 清空快照目录（如果存在）
    if os.path.exists(snapshot_directory):
        shutil.rmtree(snapshot_directory)
    os.makedirs(snapshot_directory)

    # 生成颜色列表，为每个智能体分配不同颜色
    colors = plt.cm.get_cmap('tab20', N).colors  # 使用tab20色图

    # 初始化快照时刻列表
    snapshot_times = []

    # 运行仿真并生成快照及对应轨迹图
    get_positions(snapshot_directory, colors, snapshot_times)

    # 绘制最终的trajectories of sensor nodes图
    plot_final_trajectories(snapshot_directory, colors)

    # 绘制速度图
    plt.figure(figsize=(8, 8))  # 统一图形尺寸为正方形
    plot_velocity(plt.gca())
    plt.title("Velocity Magnitudes Over Time", fontsize=TITLE_FONT_SIZE)
    plt.xlabel("Iteration", fontsize=LABEL_FONT_SIZE)
    plt.ylabel("Velocity Magnitude", fontsize=LABEL_FONT_SIZE)
    if N <= 10:
        plt.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE)
    plt.grid(True)
    plt.axis('auto')
    plt.savefig(os.path.join(snapshot_directory, "velocity_magnitudes.png"), dpi=300)  # 统一dpi
    plt.close()
    print("Saved velocity magnitudes plot.")

    # 绘制连通性图
    plt.figure(figsize=(8, 8))  # 统一图形尺寸为正方形
    plot_connectivity(plt.gca())
    plt.title("Connectivity Over Time", fontsize=TITLE_FONT_SIZE)
    plt.xlabel("Iteration", fontsize=LABEL_FONT_SIZE)
    plt.ylabel("Average Connectivity (Matrix Rank / N)", fontsize=LABEL_FONT_SIZE)
    plt.grid(True)
    plt.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE)
    plt.savefig(os.path.join(snapshot_directory, "connectivity_over_time.png"), dpi=300)  # 统一dpi
    plt.close()
    print("Saved connectivity plot.")

    # 绘制质心轨迹图
    plt.figure(figsize=(8, 8))  # 统一图形尺寸为正方形
    plot_center_of_mass(plt.gca())
    plt.title("Center of Mass Trajectory", fontsize=TITLE_FONT_SIZE)
    plt.xlabel("X Position", fontsize=LABEL_FONT_SIZE)
    plt.ylabel("Y Position", fontsize=LABEL_FONT_SIZE)
    plt.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE)
    plt.grid(True)
    plt.axis('equal')
    plt.savefig(os.path.join(snapshot_directory, "center_of_mass_trajectory.png"), dpi=300)  # 统一dpi
    plt.close()
    print("Saved center of mass trajectory plot.")

    print(f"All snapshots and plots have been saved to the '{snapshot_directory}' directory.")