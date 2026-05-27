#pragma once

#include "matrixDef.h"

namespace kf_math {

/**
 * @brief 两个矩阵相乘 (C = A * B)
 * @param A 左乘矩阵 A (维度为 p x q)
 * @param B 右乘矩阵 B (维度为 q x r)
 * @return 乘积结果矩阵 C (维度为 p x r)
 * @note 必须确保 A 的列数等于 B 的行数
 */
Matrix matrixMul(const Matrix& A, const Matrix& B);

} // namespace kf_math
