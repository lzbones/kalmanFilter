#pragma once

#include "kalman_def.h"

namespace kf_filter {

/**
 * @brief 计算卡尔曼增益（Kalman Gain）： k = p * h^T * s^-1
 * @param kf 滤波器参数（使用其估计误差协方差 p、观测矩阵 h）
 * @param s 残差协方差矩阵 (m x m)
 * @param k [out] 计算出的增益矩阵 (n x m)
 * @return 是否计算成功（在求逆 s 时，若 s 不可逆则返回 false）
 */
bool kalmanCalcGain(const KalmanFilter& kf, const kf_math::Matrix& s, kf_math::Matrix& k);

} // namespace kf_filter
