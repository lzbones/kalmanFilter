#pragma once

#include "kalman_def.h"

namespace kf_filter {

/**
 * @brief 使用观测残差和卡尔曼增益来更新状态估计值： x = x + k * y
 * @param kf 滤波器参数（其状态估计值 x 将被就地修正）
 * @param k 计算出的卡尔曼增益矩阵 (n x m)
 * @param y 计算出的测量残差向量 (m x 1)
 */
void kalmanCorrectState(KalmanFilter& kf, const kf_math::Matrix& k, const kf_math::Matrix& y);

} // namespace kf_filter
