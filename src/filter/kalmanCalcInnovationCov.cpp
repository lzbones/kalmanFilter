#include "filter/kalmanCalcInnovationCov.h"
#include "math/matrixMul.h"
#include "math/matrixAdd.h"
#include "math/matrixTranspose.h"

namespace kf_filter {

void kalmanCalcInnovationCov(const KalmanFilter& kf, kf_math::Matrix& s) {
    // 1. 计算 h^T
    kf_math::Matrix ht = kf_math::matrixTranspose(kf.h);
    
    // 2. 计算 h * p
    kf_math::Matrix hp = kf_math::matrixMul(kf.h, kf.p);
    
    // 3. 计算 h * p * h^T
    kf_math::Matrix hpht = kf_math::matrixMul(hp, ht);
    
    // 4. 计算残差协方差 s = h * p * h^T + r
    s = kf_math::matrixAdd(hpht, kf.r);
}

} // namespace kf_filter
