#pragma once

#include "matrix_def.h"

namespace kf_math {

/**
 * @brief 使用高斯-约旦消元法计算方阵的逆矩阵 (inv = A^-1)
 * @param A 输入方阵 A
 * @param inv [out] 计算出的逆矩阵结果
 * @return 是否求逆成功（若为奇异矩阵或非方阵则返回 false）
 */
bool matrixInvert(const Matrix& A, Matrix& inv);

} // namespace kf_math
