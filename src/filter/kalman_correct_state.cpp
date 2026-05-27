#include "filter/kalman_correct_state.h"
#include "math/matrix_mul.h"
#include "math/matrix_add.h"

namespace kf_filter {

void kalmanCorrectState(KalmanFilter& kf, const kf_math::Matrix& k, const kf_math::Matrix& y) {
    // 1. 计算增益对残差的作用项： k * y
    kf_math::Matrix ky = kf_math::matrixMul(k, y);
    
    // 2. 融合测量值： x = x + k * y
    kf.x = kf_math::matrixAdd(kf.x, ky);
}

} // namespace kf_filter
