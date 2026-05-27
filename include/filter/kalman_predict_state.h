#pragma once

#include "kalman_def.h"

namespace kf_filter {

/**
 * @brief 卡尔曼滤波预测步的“状态预测”子步骤 (x = A * x + B * u)
 * @param kf 滤波器结构体 (其状态 x 将被就地更新)
 * @param u 控制输入向量 (l x 1)，若无控制输入可传入空矩阵
 */
void kalmanPredictState(KalmanFilter& kf, const kf_math::Matrix& u);

} // namespace kf_filter
