#pragma once

#include "math/matrixDef.h"

namespace kf_filter {

/**
 * @brief 常规卡尔曼滤波器数据结构体 (大驼峰命名)
 * 包含状态向量、估计协方差以及所有的系统模型矩阵
 */
struct KalmanFilter {
    int stateDim;       ///< 状态维度 (n)
    int controlDim;     ///< 控制输入维度 (l)
    int measurementDim; ///< 观测测量维度 (m)

    // 状态与协方差 (单字母变量小驼峰/小写形式)
    kf_math::Matrix x;            ///< 状态估计向量 (n x 1)
    kf_math::Matrix p;            ///< 估计误差协方差矩阵 (n x n)

    // 系统模型参数 (单字母变量小驼峰/小写形式)
    kf_math::Matrix a;            ///< 状态转移矩阵 (n x n)
    kf_math::Matrix b;            ///< 控制输入矩阵 (n x l)
    kf_math::Matrix h;            ///< 观测（测量）矩阵 (m x n)

    // 噪声协方差 (单字母变量小驼峰/小写形式)
    kf_math::Matrix q;            ///< 过程噪声协方差矩阵 (n x n)
    kf_math::Matrix r;            ///< 测量噪声协方差矩阵 (m x m)
};

} // namespace kf_filter
