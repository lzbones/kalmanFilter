#include "math/matrix.h"
#include "filter/kalman.h"
#include <iostream>
#include <random>
#include <iomanip>
#include <cmath>

using namespace kf_math;
using namespace kf_filter;

int main() {
    std::cout << "==========================================================" << std::endl;
    std::cout << "        常规卡尔曼滤波 (Kalman Filter) 教学演示程序" << std::endl;
    std::cout << "               场景: 一维匀加速运动小车的轨迹追踪" << std::endl;
    std::cout << "        状态维度 n=3: [位置, 速度, 加速度]^T" << std::endl;
    std::cout << "        观测维度 m=1: [位置] (带较大随机噪声)" << std::endl;
    std::cout << "==========================================================" << std::endl;

    // --- 1. 参数与仿真设置 ---
    const double dt = 0.1;           // 采样时间步长 (秒)
    const int totalSteps = 30;       // 仿真总步数 (小驼峰)
    const double sigmaA = 0.2;       // 过程噪声标准差 (加速度扰动) (小驼峰)
    const double sigmaZ = 1.5;       // 测量噪声标准差 (位置传感器误差) (小驼峰)

    // 随机数发生器，用于模拟真实世界中的高斯白噪声
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> distProcess(0.0, sigmaA);
    std::normal_distribution<double> distMeasure(0.0, sigmaZ);

    // --- 2. 初始化滤波器 ---
    KalmanFilter kf;
    // 状态维度 n=3，观测维度 m=1，控制维度 l=0
    kalmanInit(kf, 3, 1, 0);

    // 2.1 设定初始状态估计 x_0 和初始协方差 p_0 (单字母变量小写)
    kf.x.data[0] = 0.0; // 估计初始位置
    kf.x.data[1] = 0.0; // 估计初始速度
    kf.x.data[2] = 0.0; // 估计初始加速度

    kf.p.data[0 * 3 + 0] = 10.0; // 位置初始不确定度
    kf.p.data[1 * 3 + 1] = 10.0; // 速度初始不确定度
    kf.p.data[2 * 3 + 2] = 10.0; // 加速度初始不确定度

    // 2.2 设定状态转移矩阵 a (n x n)
    // s = s + v*dt + 0.5*a*dt^2
    // v = v + a*dt
    // a = a
    kf.a.data[0 * 3 + 0] = 1.0;  kf.a.data[0 * 3 + 1] = dt;   kf.a.data[0 * 3 + 2] = 0.5 * dt * dt;
    kf.a.data[1 * 3 + 0] = 0.0;  kf.a.data[1 * 3 + 1] = 1.0;  kf.a.data[1 * 3 + 2] = dt;
    kf.a.data[2 * 3 + 0] = 0.0;  kf.a.data[2 * 3 + 1] = 0.0;  kf.a.data[2 * 3 + 2] = 1.0;

    // 2.3 设定观测矩阵 h (m x n)
    // z = [1, 0, 0] * [s, v, a]^T
    kf.h.data[0 * 3 + 0] = 1.0;
    kf.h.data[0 * 3 + 1] = 0.0;
    kf.h.data[0 * 3 + 2] = 0.0;

    // 2.4 设定过程噪声协方差矩阵 q (n x n)
    // 噪声主要引入在加速度级，q = g * g^T * sigma_a^2
    // 其中 g = [0.5*dt^2, dt, 1]^T
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;
    double sa2 = sigmaA * sigmaA;
    kf.q.data[0 * 3 + 0] = 0.25 * dt4 * sa2; kf.q.data[0 * 3 + 1] = 0.5 * dt3 * sa2;  kf.q.data[0 * 3 + 2] = 0.5 * dt2 * sa2;
    kf.q.data[1 * 3 + 0] = 0.5 * dt3 * sa2;  kf.q.data[1 * 3 + 1] = dt2 * sa2;        kf.q.data[1 * 3 + 2] = dt * sa2;
    kf.q.data[2 * 3 + 0] = 0.5 * dt2 * sa2;  kf.q.data[2 * 3 + 1] = dt * sa2;         kf.q.data[2 * 3 + 2] = 1.0 * sa2;

    // 2.5 设定测量噪声协方差 r (m x m)
    kf.r.data[0] = sigmaZ * sigmaZ;

    // --- 3. 仿真初值设定 (真实物理状态) ---
    // 真实状态定义为 3x1 矩阵 [真实位置, 真实速度, 真实加速度]^T
    Matrix trueState = matrixCreate(3, 1, 0.0);
    trueState.data[0] = 0.0;   // 真实初始位置
    trueState.data[1] = 0.0;   // 真实初始速度
    trueState.data[2] = 1.5;   // 真实初始加速度 (假定小车以 1.5 m/s^2 恒定加速度起步)

    // --- 4. 循环仿真与滤波计算 ---
    // 打印表头
    std::cout << std::string(105, '-') << std::endl;
    std::cout << std::setw(6)  << "时间"
              << std::setw(15) << "真实位置" 
              << std::setw(15) << "观测位置" 
              << std::setw(18) << "估计位置" 
              << std::setw(15) << "真实速度" 
              << std::setw(18) << "估计速度"
              << std::setw(18) << "估计加速度" << std::endl;
    std::cout << std::string(105, '-') << std::endl;

    for (int step = 0; step < totalSteps; ++step) {
        double currentTime = step * dt;

        // 4.1 物理状态外推（生成真实状态）
        // x_true = a * x_true
        Matrix nextTrue = matrixMul(kf.a, trueState);
        // 在加速度项添加一点随机物理扰动
        nextTrue.data[2] += distProcess(gen);
        trueState = nextTrue;

        // 4.2 生成含噪声的观测值
        // z = h * x_true + v_z
        double noiseZ = distMeasure(gen);
        Matrix z = matrixCreate(1, 1);
        z.data[0] = trueState.data[0] + noiseZ;

        // 4.3 滤波器预测阶段 (Predict)
        kalmanPredict(kf);

        // 4.4 滤波器更新融合阶段 (Update)
        bool success = kalmanUpdate(kf, z);
        if (!success) {
            std::cerr << "警告: 卡尔曼滤波更新失败 (矩阵不可逆)!" << std::endl;
            break;
        }

        // 4.5 打印当前时间步对比结果
        std::cout << std::fixed << std::setprecision(3)
                  << std::setw(5)  << currentTime << "s"
                  << std::setw(15) << trueState.data[0]
                  << std::setw(15) << z.data[0]
                  << std::setw(18) << kf.x.data[0]
                  << std::setw(15) << trueState.data[1]
                  << std::setw(18) << kf.x.data[1]
                  << std::setw(18) << kf.x.data[2]
                  << std::endl;
    }
    
    std::cout << std::string(105, '-') << std::endl;
    std::cout << "说明:" << std::endl;
    std::cout << "1. 可以看出，虽然[观测位置]包含了非常强烈的随机噪声波动，" << std::endl;
    std::cout << "2. 但卡尔曼滤波器估计出的[估计位置]非常平滑，且迅速靠拢[真实位置]；" << std::endl;
    std::cout << "3. 同时，即使没有直接传感器测量速度和加速度，滤波器也成功还原了它们的真实值。" << std::endl;
    std::cout << "==========================================================" << std::endl;

    return 0;
}
