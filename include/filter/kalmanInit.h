#pragma once

#include "kalmanDef.h"

namespace kf_filter {

/**
 * @brief 初始化卡尔曼滤波器，分配各矩阵的尺寸和内存 (小驼峰命名)
 * @param kf [out] 待初始化的滤波器结构体
 * @param stateDim 状态变量维度 (n)
 * @param measurementDim 观测变量维度 (m)
 * @param controlDim 控制输入维度 (l)，默认为 0（无控制输入）
 */
void kalmanInit(KalmanFilter& kf, int stateDim, int measurementDim, int controlDim = 0);

} // namespace kf_filter
