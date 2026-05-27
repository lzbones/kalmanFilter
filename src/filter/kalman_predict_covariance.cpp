#include "filter/kalman_predict_covariance.h"
#include "math/matrix_mul.h"
#include "math/matrix_add.h"
#include "math/matrix_transpose.h"

namespace kf_filter {

void kalmanPredictCovariance(KalmanFilter& kf) {
    // 1. 计算 a^T
    kf_math::Matrix at = kf_math::matrixTranspose(kf.a);
    
    // 2. 计算 a * p
    kf_math::Matrix ap = kf_math::matrixMul(kf.a, kf.p);
    
    // 3. 计算 a * p * a^T
    kf_math::Matrix apat = kf_math::matrixMul(ap, at);
    
    // 4. 更新协方差矩阵 p = a * p * a^T + q
    kf.p = kf_math::matrixAdd(apat, kf.q);
}

} // namespace kf_filter
