# 卡尔曼滤波 C++ 教学程序说明文档

本项目在根目录下完成了一个常规卡尔曼滤波（Kalman Filter）C++ 教学示例项目的开发。该项目严格遵循**过程化设计**，将底层矩阵计算与上层卡尔曼更新方程进行完全解耦，且**每个函数和与之对应的文件都有独立的头文件与源文件（完全使用小驼峰命名的对应关系）**，采用了符合专业规范的**接口与实现分离模式（Standard Library Layout）**。

---

## 目录结构

所有文件已在相应子目录中创建完毕：

```text
kalmanFilter/ (项目根目录)
 ├── CMakeLists.txt                 # CMake 构建配置文件
 ├── main.cpp                       # 仿真与滤波器验证主程序
 ├── Readme.md                      # 说明文档 (本文件)
 ├── include/                       # 所有的头文件 (.h)
 │    ├── math/                     # 通用矩阵数学库头文件
 │    │    ├── matrixDef.h          # 矩阵数据结构 Matrix 定义 (小驼峰文件名，下同)
 │    │    ├── matrix.h             # 矩阵库总聚合头文件
 │    │    ├── matrixCreate.h       # 创建矩阵
 │    │    ├── matrixIdentity.h     # 创建单位矩阵
 │    │    ├── matrixAdd.h          # 矩阵加法
 │    │    ├── matrixSub.h          # 矩阵减法
 │    │    ├── matrixMul.h          # 矩阵乘法
 │    │    ├── matrixTranspose.h    # 矩阵转置
 │    │    ├── matrixInvert.h       # 矩阵求逆 (高斯-约旦消元法)
 │    │    └── matrixPrint.h        # 矩阵输出打印
 │    └── filter/                   # 滤波器算法库头文件
 │         ├── kalmanDef.h          # 滤波器结构体 KalmanFilter 定义 (小驼峰文件名，下同)
 │         ├── kalman.h             # 滤波器总聚合头文件
 │         ├── kalmanInit.h         # 初始化滤波器
 │         ├── kalmanPredictState.h       # 状态外推预测
 │         ├── kalmanPredictCovariance.h  # 协方差外推预测
 │         ├── kalmanPredict.h            # 预测总控制
 │         ├── kalmanCalcResidual.h       # 计算残差
 │         ├── kalmanCalcInnovationCov.h # 计算残差协方差
 │         ├── kalmanCalcGain.h           # 计算卡尔曼增益
 │         ├── kalmanCorrectState.h       # 修正状态
 │         ├── kalmanCorrectCovariance.h  # 修正协方差
 │         └── kalmanUpdate.h              # 更新总控制
 ├── src/                           # 所有的源实现文件 (.cpp)
 │    ├── math/                     # 矩阵库函数实现 (例如 matrixCreate.cpp)
 │    └── filter/                   # 滤波器步骤函数实现 (例如 kalmanInit.cpp)
 └── analysis/                      # 数据分析与可视化文件夹 (已整理)
      ├── results.txt               # 运行 demo 重定向输出的数据日志
      ├── plot_results.py           # 用于读取结果并绘图的 Python 脚本
      ├── tuning_experiment.py      # 对比参数调优前后的仿真实验脚本
      ├── kalman_filter_results.png # 位置、速度和加速度估计的默认趋势图
      └── kalman_tuning_comparison.png # 默认滤波器与调优滤波器在速度、加速度上的跟踪对比图
```

---

## 函数层级关系与公式映射 (Function Hierarchy & Formulas)

项目采用高度解耦的分层设计，层级调用关系与卡尔曼滤波数学公式的映射关系如下（单字母成员变量和函数全部使用小写/小驼峰命名）：

```text
main() (main.cpp)
 │
 ├── kalmanInit() (kalmanInit.h / .cpp) -> 初始化 x, p, a, b, h, q, r 矩阵
 │    ├── matrixCreate()
 │    └── matrixIdentity()
 │
 └── [时间循环步]
      │
      ├── kalmanPredict() (kalmanPredict.h / .cpp) -> 状态与协方差预测阶段
      │    ├── kalmanPredictState()        -> 预测状态: x = a * x + b * u
      │    │    ├── matrixMul()
      │    │    └── matrixAdd()
      │    └── kalmanPredictCovariance()   -> 预测协方差: p = a * p * a^T + q
      │         ├── matrixMul()
      │         └── matrixTranspose()
      │
      └── kalmanUpdate() (kalmanUpdate.h / .cpp) -> 测量更新与数据融合阶段
           ├── kalmanCalcResidual()        -> 计算残差: y = z - h * x
           │    ├── matrixMul()
           │    └── matrixSub()
           ├── kalmanCalcInnovationCov() -> 计算残差协方差: s = h * p * h^T + r
           │    ├── matrixMul()
           │    ├── matrixTranspose()
           │    └── matrixAdd()
           ├── kalmanCalcGain()            -> 计算增益: k = p * h^T * s^-1
           │    ├── matrixTranspose()
           │    ├── matrixInvert()          -> 矩阵求逆 (Gauss-Jordan消元法)
           │    └── matrixMul()
           ├── kalmanCorrectState()        -> 修正状态: x = x + k * y
           │    ├── matrixMul()
           │    └── matrixAdd()
           └── kalmanCorrectCovariance()   -> 修正误差协方差: p = (i - k * h) * p
                ├── matrixMul()
                ├── matrixIdentity()
                └── matrixSub()
```

---

## 命名空间说明 (Namespaces)

为了防止命名污染，并顺应现代 C++ 的工程实践，本项目引入了两个命名空间（使用下划线命名）：
1. **`kf_math`**：包含底层的纯通用矩阵数学操作（`Matrix` 结构体，以及 `matrixCreate`, `matrixMul` 等函数）。
2. **`kf_filter`**：包含上层的卡尔曼滤波器逻辑（`KalmanFilter` 结构体，以及 `kalmanPredict`, `kalmanUpdate` 等函数）。

在 `main.cpp` 中，可通过 `using namespace kf_math;` and `using namespace kf_filter;` 方便地引用它们。

---

## 如何编译与运行

可以使用 `clang++` 直接编译该项目：

```bash
# 直接使用 Clang 编译所有源文件 (C++11)
clang++ -std=c++11 -Iinclude main.cpp src/math/*.cpp src/filter/*.cpp -o kalman_demo

# 运行程序
./kalman_demo
```

---

## 仿真验证结果

主程序模拟了一个**一维匀加速运动小车（3个状态量：位置、速度、加速度）**。输入给卡尔曼滤波器的只有**含有大量噪声的位置测量数据**，估计出的位置非常平滑，且能同时还原出速度和加速度的数值。

---

## 关键代码链接

* 核心数据结构: [matrixDef.h](file:///Users/qingxu/Documents/Software/Cpp/kalmanFilter/include/math/matrixDef.h), [kalmanDef.h](file:///Users/qingxu/Documents/Software/Cpp/kalmanFilter/include/filter/kalmanDef.h)
* 核心数学模块: [matrixInvert.cpp (高斯求逆)](file:///Users/qingxu/Documents/Software/Cpp/kalmanFilter/src/math/matrixInvert.cpp), [matrixMul.cpp (乘法)](file:///Users/qingxu/Documents/Software/Cpp/kalmanFilter/src/math/matrixMul.cpp)
* 滤波器核心逻辑: [kalmanPredict.cpp](file:///Users/qingxu/Documents/Software/Cpp/kalmanFilter/src/filter/kalmanPredict.cpp), [kalmanUpdate.cpp](file:///Users/qingxu/Documents/Software/Cpp/kalmanFilter/src/filter/kalmanUpdate.cpp)
* 教学演示主入口: [main.cpp](file:///Users/qingxu/Documents/Software/Cpp/kalmanFilter/main.cpp)

---

## 数据保存与可视化 (Data & Plotting)

所有分析数据与图表均已整理在 `analysis/` 文件夹下：
1. **数据日志文件**: [results.txt](file:///Users/qingxu/Documents/Software/Cpp/kalmanFilter/analysis/results.txt) （包含了小车真实位置/速度、观测位置、估计位置/速度/加速度的文本数据）。
2. **可视化绘图脚本**: [plot_results.py](file:///Users/qingxu/Documents/Software/Cpp/kalmanFilter/analysis/plot_results.py) （使用 Matplotlib 自动读取并生成 3 个子图展示）。
3. **参数调优对比脚本**: [tuning_experiment.py](file:///Users/qingxu/Documents/Software/Cpp/kalmanFilter/analysis/tuning_experiment.py) （对比默认参数与调优后参数的滤波跟踪效果）。
4. **输出图表**:
   * 默认参数效果: [kalman_filter_results.png](file:///Users/qingxu/Documents/Software/Cpp/kalmanFilter/analysis/kalman_filter_results.png)
   * Q 协方差调优对比: [kalman_tuning_comparison.png](file:///Users/qingxu/Documents/Software/Cpp/kalmanFilter/analysis/kalman_tuning_comparison.png)

若想重新生成数据并绘图，可在终端运行：
```bash
# 1. 运行 demo 并将输出重定向到 results.txt (放置于 analysis 目录下)
./kalman_demo > analysis/results.txt

# 2. 运行 Python 脚本重新绘图
cd analysis
python3 plot_results.py
python3 tuning_experiment.py
```
