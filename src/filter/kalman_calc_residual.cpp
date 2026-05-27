#include "filter/kalman_calc_residual.h"
#include "math/matrix_mul.h"
#include "math/matrix_sub.h"

namespace kf_filter {

void kalmanCalcResidual(const KalmanFilter& kf, const kf_math::Matrix& z, kf_math::Matrix& y) {
    // 1. 计算 h * x (预测的观测值)
    kf_math::Matrix hx = kf_math::matrixMul(kf.h, kf.x);
    
    // 2. 计算残差 y = z - h * x
    y = kf_math::matrixSub(z, hx);
}

} // namespace kf_filter
