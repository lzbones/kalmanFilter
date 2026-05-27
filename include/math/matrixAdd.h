#pragma once

#include "matrixDef.h"

namespace kf_math {

/**
 * @brief 两个矩阵相加 (C = A + B)
 * @param A 矩阵 A
 * @param B 矩阵 B
 * @return 相加后的结果矩阵 C
 * @note 必须确保 A 和 B 的行列维度一致
 */
Matrix matrixAdd(const Matrix& A, const Matrix& B);

} // namespace kf_math
