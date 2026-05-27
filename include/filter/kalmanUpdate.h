#pragma once

#include "kalmanDef.h"

namespace kf_filter {

/**
 * @brief 整个卡尔曼滤波的更新阶段 (协调残差、残差协方差、增益计算与状态/协方差修正)
 * @param kf 滤波器参数 (就地更新估计的状态和协方差)
 * @param z 测量向量 (m x 1)
 * @return 是否更新成功 (若测量残差协方差不可逆则返回 false)
 */
bool kalmanUpdate(KalmanFilter& kf, const kf_math::Matrix& z);

} // namespace kf_filter
