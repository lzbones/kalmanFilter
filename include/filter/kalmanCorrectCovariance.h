#pragma once

#include "kalmanDef.h"

namespace kf_filter {

/**
 * @brief 使用卡尔曼增益和观测矩阵来更新估计误差协方差： p = (i - k * h) * p
 * @param kf 滤波器参数（其误差协方差矩阵 p 将被就地更新）
 * @param k 计算出的卡尔曼增益矩阵 (n x m)
 */
void kalmanCorrectCovariance(KalmanFilter& kf, const kf_math::Matrix& k);

} // namespace kf_filter
