#include "filter/kalmanPredict.h"
#include "filter/kalmanPredictState.h"
#include "filter/kalmanPredictCovariance.h"

namespace kf_filter {

void kalmanPredict(KalmanFilter& kf, const kf_math::Matrix& u) {
    // 1. 状态外推预测
    kalmanPredictState(kf, u);
    
    // 2. 协方差外推预测
    kalmanPredictCovariance(kf);
}

} // namespace kf_filter
