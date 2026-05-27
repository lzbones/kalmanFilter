import os
import re
import matplotlib.pyplot as plt

# 设置 macOS 中文字体，避免 Matplotlib 中文显示为方块
plt.rcParams['font.sans-serif'] = ['Heiti TC', 'PingFang HK', 'Arial Unicode MS', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False  # 正常显示负号

# 1. 读取 results.txt
filePath = "results.txt"
if not os.path.exists(filePath):
    # 尝试在父目录寻找 (以防脚本被移动到子文件夹中运行)
    filePath = os.path.join("..", "results.txt")
    if not os.path.exists(filePath):
        print("Error: results.txt not found in current or parent directory.")
        exit(1)

times = []
truePos = []
obsPos = []
estPos = []
trueVel = []
estVel = []
estAcc = []

# 编译正则表达式匹配包含数据的行，例如：
# 0.100s          0.030         -0.923            -0.287          0.293            -0.314            -0.044
rowPattern = re.compile(
    r"^\s*([0-9.]+)\s*s\s+"      # 时间 (例如 0.100s)
    r"([-\d.]+)\s+"              # 真实位置
    r"([-\d.]+)\s+"              # 观测位置
    r"([-\d.]+)\s+"              # 估计位置
    r"([-\d.]+)\s+"              # 真实速度
    r"([-\d.]+)\s+"              # 估计速度
    r"([-\d.]+)"                 # 估计加速度
)

with open(filePath, "r", encoding="utf-8") as f:
    for line in f:
        match = rowPattern.match(line)
        if match:
            times.append(float(match.group(1)))
            truePos.append(float(match.group(2)))
            obsPos.append(float(match.group(3)))
            estPos.append(float(match.group(4)))
            trueVel.append(float(match.group(5)))
            estVel.append(float(match.group(6)))
            estAcc.append(float(match.group(7)))

if not times:
    print("Error: No data rows matched in results.txt. Please check the file format.")
    exit(1)

# 2. 绘制图表
fig, axs = plt.subplots(3, 1, figsize=(10, 9))

# 子图 1: 位置 (Position)
axs[0].plot(times, truePos, 'g-', label='True Position (真实位置)', linewidth=2)
axs[0].scatter(times, obsPos, color='red', alpha=0.5, label='Noisy Observations (带噪声测量值)', s=20)
axs[0].plot(times, estPos, 'b--', label='Kalman Estimated Position (滤波估计位置)', linewidth=2)
axs[0].set_title('Kalman Filter Tracking Results (Position / 位置)', fontsize=12)
axs[0].set_ylabel('Position (m)', fontsize=10)
axs[0].legend(loc='best')
axs[0].grid(True, linestyle=':')

# 子图 2: 速度 (Velocity)
axs[1].plot(times, trueVel, 'g-', label='True Velocity (真实速度)', linewidth=2)
axs[1].plot(times, estVel, 'b--', label='Kalman Estimated Velocity (滤波估计速度)', linewidth=2)
axs[1].set_title('Velocity Estimation (Velocity / 速度)', fontsize=12)
axs[1].set_ylabel('Velocity (m/s)', fontsize=10)
axs[1].legend(loc='best')
axs[1].grid(True, linestyle=':')

# 子图 3: 加速度 (Acceleration)
axs[2].plot(times, estAcc, 'b--', label='Kalman Estimated Acceleration (滤波估计加速度)', linewidth=2)
axs[2].axhline(y=1.5, color='g', linestyle='-', label='Initial True Acceleration (1.5 m/s^2)', linewidth=1.5)
axs[2].set_title('Acceleration Estimation (Acceleration / 加速度)', fontsize=12)
axs[2].set_xlabel('Time (s)', fontsize=10)
axs[2].set_ylabel('Acceleration (m/s^2)', fontsize=10)
axs[2].legend(loc='best')
axs[2].grid(True, linestyle=':')

plt.tight_layout()
outputImageName = 'kalman_filter_results.png'
plt.savefig(outputImageName, dpi=300)
print(f"Plot successfully saved as '{outputImageName}'.")
