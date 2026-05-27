#pragma once

#include "matrix_def.h"

namespace kf_math {

/**
 * @brief 创建指定大小并初始化的矩阵 (小驼峰命名)
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * @param val 初始化的值（默认为 0.0）
 * @return 创建好的 Matrix 结构体
 */
Matrix matrixCreate(int rows, int cols, double val = 0.0);

} // namespace kf_math
