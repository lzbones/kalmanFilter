#pragma once

#include "matrix_def.h"

namespace kf_math {

/**
 * @brief 计算矩阵转置 (B = A^T)
 * @param A 输入矩阵 A
 * @return 转置后的结果矩阵 B
 */
Matrix matrixTranspose(const Matrix& A);

} // namespace kf_math
