#pragma once

#include "kalman_def.h"

namespace kf_filter {

/**
 * @brief 整个卡尔曼滤波的预测阶段 (结合状态预测与协方差预测)
 * @param kf 滤波器结构体 (就地更新预测的状态和协方差)
 * @param u 控制输入向量 (l x 1)，若无控制输入可传入空矩阵
 */
void kalmanPredict(KalmanFilter& kf, const kf_math::Matrix& u = kf_math::Matrix{0, 0, {}});

} // namespace kf_filter
