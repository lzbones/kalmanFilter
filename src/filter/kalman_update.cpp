#include "filter/kalman_update.h"
#include "filter/kalman_calc_residual.h"
#include "filter/kalman_calc_innovation_cov.h"
#include "filter/kalman_calc_gain.h"
#include "filter/kalman_correct_state.h"
#include "filter/kalman_correct_covariance.h"

namespace kf_filter {

bool kalmanUpdate(KalmanFilter& kf, const kf_math::Matrix& z) {
    // 1. 计算测量残差 y = z - h * x
    kf_math::Matrix y;
    kalmanCalcResidual(kf, z, y);
    
    // 2. 计算残差协方差 s = h * p * h^T + r
    kf_math::Matrix s;
    kalmanCalcInnovationCov(kf, s);
    
    // 3. 计算卡尔曼增益 k = p * h^T * s^-1
    kf_math::Matrix k;
    if (!kalmanCalcGain(kf, s, k)) {
        return false; // 计算失败（可能由于观测异常，矩阵不可逆）
    }
    
    // 4. 修正状态估计 x = x + k * y
    kalmanCorrectState(kf, k, y);
    
    // 5. 修正估计协方差 p = (i - k * h) * p
    kalmanCorrectCovariance(kf, k);
    
    return true;
}

} // namespace kf_filter
