#include "filter/kalmanInit.h"
#include "math/matrixCreate.h"
#include "math/matrixIdentity.h"

namespace kf_filter {

void kalmanInit(KalmanFilter& kf, int stateDim, int measurementDim, int controlDim) {
    kf.stateDim = stateDim;
    kf.measurementDim = measurementDim;
    kf.controlDim = controlDim;

    // 分配并初始化状态向量 x (n x 1)
    kf.x = kf_math::matrixCreate(stateDim, 1, 0.0);
    
    // 初始化估计误差协方差矩阵 p (n x n) 为单位阵
    kf.p = kf_math::matrixIdentity(stateDim);
    
    // 初始化状态转移矩阵 a (n x n) 为单位阵
    kf.a = kf_math::matrixIdentity(stateDim);
    
    // 初始化控制矩阵 b (n x l)
    if (controlDim > 0) {
        kf.b = kf_math::matrixCreate(stateDim, controlDim, 0.0);
    } else {
        kf.b = kf_math::matrixCreate(0, 0, 0.0);
    }
    
    // 初始化观测矩阵 h (m x n) 为零阵，由用户配置具体观测关系
    kf.h = kf_math::matrixCreate(measurementDim, stateDim, 0.0);
    
    // 初始化过程噪声协方差 q (n x n) 和测量噪声协方差 r (m x m) 为单位阵
    kf.q = kf_math::matrixIdentity(stateDim);
    kf.r = kf_math::matrixIdentity(measurementDim);
}

} // namespace kf_filter
