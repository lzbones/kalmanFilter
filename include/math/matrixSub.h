#pragma once

#include "matrixDef.h"

namespace kf_math {

/**
 * @brief 两个矩阵相减 (C = A - B)
 * @param A 减数矩阵 A
 * @param B 被减数矩阵 B
 * @return 相减后的结果矩阵 C
 * @note 必须确保 A 和 B 的行列维度一致
 */
Matrix matrixSub(const Matrix& A, const Matrix& B);

} // namespace kf_math
