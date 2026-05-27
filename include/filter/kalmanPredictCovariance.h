#pragma once

#include "kalmanDef.h"

namespace kf_filter {

/**
 * @brief 卡尔曼滤波预测步的“协方差预测”子步骤 (P = A * P * A^T + Q)
 * @param kf 滤波器结构体 (其协方差 P 将被就地更新)
 */
void kalmanPredictCovariance(KalmanFilter& kf);

} // namespace kf_filter
