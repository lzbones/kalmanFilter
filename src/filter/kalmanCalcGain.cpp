#include "filter/kalmanCalcGain.h"
#include "math/matrixMul.h"
#include "math/matrixTranspose.h"
#include "math/matrixInvert.h"

namespace kf_filter {

bool kalmanCalcGain(const KalmanFilter& kf, const kf_math::Matrix& s, kf_math::Matrix& k) {
    // 1. 对残差协方差 s 求逆 (s^-1)
    kf_math::Matrix sInv;
    if (!kf_math::matrixInvert(s, sInv)) {
        return false; // 若 s 为奇异矩阵无法求逆，滤波计算失败
    }

    // 2. 计算 h^T
    kf_math::Matrix ht = kf_math::matrixTranspose(kf.h);

    // 3. 计算 p * h^T
    kf_math::Matrix pht = kf_math::matrixMul(kf.p, ht);

    // 4. 计算卡尔曼增益 k = p * h^T * s^-1
    k = kf_math::matrixMul(pht, sInv);

    return true;
}

} // namespace kf_filter
