#pragma once

#include "kalman_def.h"

namespace kf_filter {

/**
 * @brief 计算测量残差（Innovation or Residual）： y = z - h * x
 * @param kf 滤波器参数（使用其已预测的状态估计 x 和观测矩阵 h）
 * @param z 测量向量 (m x 1)
 * @param y [out] 残差结果向量 (m x 1)
 */
void kalmanCalcResidual(const KalmanFilter& kf, const kf_math::Matrix& z, kf_math::Matrix& y);

} // namespace kf_filter
