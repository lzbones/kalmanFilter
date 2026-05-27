#pragma once

#include "kalmanDef.h"

namespace kf_filter {

/**
 * @brief 计算测量残差协方差（Innovation Covariance）： s = h * p * h^T + r
 * @param kf 滤波器参数（使用其已预测的估计误差协方差 p、观测矩阵 h 和测量噪声协方差 r）
 * @param s [out] 计算出的残差协方差结果矩阵 (m x m)
 */
void kalmanCalcInnovationCov(const KalmanFilter& kf, kf_math::Matrix& s);

} // namespace kf_filter
