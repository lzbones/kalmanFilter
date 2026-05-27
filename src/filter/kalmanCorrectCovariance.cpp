#include "filter/kalmanCorrectCovariance.h"
#include "math/matrixMul.h"
#include "math/matrixSub.h"
#include "math/matrixIdentity.h"

namespace kf_filter {

void kalmanCorrectCovariance(KalmanFilter& kf, const kf_math::Matrix& k) {
    // 1. 生成单位矩阵 i (n x n)
    kf_math::Matrix i = kf_math::matrixIdentity(kf.stateDim);
    
    // 2. 计算 k * h
    kf_math::Matrix kh = kf_math::matrixMul(k, kf.h);
    
    // 3. 计算 i - k * h
    kf_math::Matrix ikh = kf_math::matrixSub(i, kh);
    
    // 4. 更新协方差矩阵 p = (i - k * h) * p
    kf.p = kf_math::matrixMul(ikh, kf.p);
}

} // namespace kf_filter
