import numpy as np
import matplotlib.pyplot as plt

# 设置 macOS 中文字体，正常显示负号
plt.rcParams['font.sans-serif'] = ['Heiti TC', 'PingFang HK', 'Arial Unicode MS', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

def run_simulation():
    # --- 1. 物理模型与参数 ---
    dt = 0.1
    total_steps = 50  # 增加步数，使收敛过程更清晰
    sigma_a_true = 0.2  # 真实物理扰动标准差
    sigma_z = 1.5       # 测量噪声标准差
    
    # 状态转移矩阵 A
    A = np.array([
        [1.0, dt,  0.5*dt*dt],
        [0.0, 1.0, dt],
        [0.0, 0.0, 1.0]
    ])
    
    # 观测矩阵 H
    H = np.array([[1.0, 0.0, 0.0]])
    
    # 真实初值 [位置, 速度, 加速度]^T
    true_x = np.array([[0.0], [0.0], [1.5]])
    
    # 保存仿真轨迹
    true_history = []
    measurements = []
    
    np.random.seed(42) # 固定随机数种子，使实验可重复
    
    for step in range(total_steps):
        # 状态转移 + 扰动
        true_x = A @ true_x
        true_x[2, 0] += np.random.normal(0.0, sigma_a_true)
        true_history.append(true_x.copy())
        
        # 观测值
        z = H @ true_x + np.random.normal(0.0, sigma_z)
        measurements.append(z[0, 0])
        
    return dt, total_steps, true_history, measurements, A, H, sigma_z

def run_kalman(dt, total_steps, measurements, A, H, sigma_z, sigma_a_filter):
    # 初始化状态估计 x 和协方差 P
    x = np.array([[0.0], [0.0], [0.0]])
    P = np.eye(3) * 10.0
    
    # 观测噪声协方差 R
    R = np.array([[sigma_z**2]])
    
    # 滤波器内部的过程噪声协方差 Q
    dt2 = dt * dt
    dt3 = dt2 * dt
    dt4 = dt3 * dt
    sa2 = sigma_a_filter**2
    Q = np.array([
        [0.25*dt4*sa2, 0.5*dt3*sa2, 0.5*dt2*sa2],
        [0.5*dt3*sa2,  dt2*sa2,     dt*sa2],
        [0.5*dt2*sa2,  dt*sa2,      1.0*sa2]
    ])
    
    est_history = []
    
    for step in range(total_steps):
        # 1. 预测步 (Predict)
        x = A @ x
        P = A @ P @ A.T + Q
        
        # 2. 更新步 (Update)
        z = np.array([[measurements[step]]])
        y = z - H @ x
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x = x + K @ y
        P = (np.eye(3) - K @ H) @ P
        
        est_history.append(x.copy())
        
    return est_history

# 执行仿真
dt, total_steps, true_hist, measurements, A, H, sigma_z = run_simulation()

# 运行两个不同过程噪声参数的滤波器进行对比
# 滤波器 A：低过程噪声估计 (默认，对加速度变化反应慢)
est_hist_slow = run_kalman(dt, total_steps, measurements, A, H, sigma_z, sigma_a_filter=0.2)
# 滤波器 B：高过程噪声估计 (调优后，对加速度变化反应快)
est_hist_fast = run_kalman(dt, total_steps, measurements, A, H, sigma_z, sigma_a_filter=1.2)

# 转换数据格式用于绘图
times = np.arange(total_steps) * dt
true_pos = [h[0, 0] for h in true_hist]
true_vel = [h[1, 0] for h in true_hist]
true_acc = [h[2, 0] for h in true_hist]

est_pos_slow = [h[0, 0] for h in est_hist_slow]
est_vel_slow = [h[1, 0] for h in est_hist_slow]
est_acc_slow = [h[2, 0] for h in est_hist_slow]

est_pos_fast = [h[0, 0] for h in est_hist_fast]
est_vel_fast = [h[1, 0] for h in est_hist_fast]
est_acc_fast = [h[2, 0] for h in est_hist_fast]

# 绘图对比
fig, axs = plt.subplots(3, 1, figsize=(10, 10))

# 1. 位置对比
axs[0].plot(times, true_pos, 'g-', label='True Position (真实位置)', linewidth=2)
axs[0].scatter(times, measurements, color='red', alpha=0.3, label='Noisy Observations (观测值)', s=15)
axs[0].plot(times, est_pos_slow, 'orange', linestyle=':', label='KF (Default $\sigma_a=0.2$)', linewidth=2)
axs[0].plot(times, est_pos_fast, 'blue', linestyle='--', label='KF (Tuned $\sigma_a=1.2$)', linewidth=2)
axs[0].set_title('Position Tracking Comparison / 位置追踪对比', fontsize=12)
axs[0].set_ylabel('Position (m)', fontsize=10)
axs[0].legend()
axs[0].grid(True, linestyle=':')

# 2. 速度对比
axs[1].plot(times, true_vel, 'g-', label='True Velocity (真实速度)', linewidth=2)
axs[1].plot(times, est_vel_slow, 'orange', linestyle=':', label='KF (Default $\sigma_a=0.2$)', linewidth=2)
axs[1].plot(times, est_vel_fast, 'blue', linestyle='--', label='KF (Tuned $\sigma_a=1.2$)', linewidth=2)
axs[1].set_title('Velocity Estimation Comparison / 速度估计对比', fontsize=12)
axs[1].set_ylabel('Velocity (m/s)', fontsize=10)
axs[1].legend()
axs[1].grid(True, linestyle=':')

# 3. 加速度对比
axs[2].plot(times, true_acc, 'g-', label='True Acceleration (真实加速度)', linewidth=2)
axs[2].plot(times, est_acc_slow, 'orange', linestyle=':', label='KF (Default $\sigma_a=0.2$)', linewidth=2)
axs[2].plot(times, est_acc_fast, 'blue', linestyle='--', label='KF (Tuned $\sigma_a=1.2$)', linewidth=2)
axs[2].set_title('Acceleration Estimation Comparison / 加速度估计对比', fontsize=12)
axs[2].set_xlabel('Time (s)', fontsize=10)
axs[2].set_ylabel('Acceleration (m/s^2)', fontsize=10)
axs[2].legend()
axs[2].grid(True, linestyle=':')

plt.tight_layout()
output_name = 'kalman_tuning_comparison.png'
plt.savefig(output_name, dpi=300)
print(f"Tuning experiment complete. Saved plot as '{output_name}'")
