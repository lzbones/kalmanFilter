#include "filter/kalman_predict.h"
#include "filter/kalman_predict_state.h"
#include "filter/kalman_predict_covariance.h"

namespace kf_filter {

void kalmanPredict(KalmanFilter& kf, const kf_math::Matrix& u) {
    // 1. 状态外推预测
    kalmanPredictState(kf, u);
    
    // 2. 协方差外推预测
    kalmanPredictCovariance(kf);
}

} // namespace kf_filter
