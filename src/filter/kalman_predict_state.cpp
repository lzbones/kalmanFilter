#include "filter/kalman_predict_state.h"
#include "math/matrix_mul.h"
#include "math/matrix_add.h"

namespace kf_filter {

void kalmanPredictState(KalmanFilter& kf, const kf_math::Matrix& u) {
    // 1. 计算状态转移： ax = a * x
    kf_math::Matrix ax = kf_math::matrixMul(kf.a, kf.x);

    // 2. 如果存在控制输入，则加上控制影响： x = a * x + b * u
    if (kf.controlDim > 0 && u.rows > 0 && u.cols > 0) {
        kf_math::Matrix bu = kf_math::matrixMul(kf.b, u);
        kf.x = kf_math::matrixAdd(ax, bu);
    } else {
        kf.x = ax;
    }
}

} // namespace kf_filter
