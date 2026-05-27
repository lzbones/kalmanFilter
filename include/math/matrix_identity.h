#pragma once

#include "matrix_def.h"

namespace kf_math {

/**
 * @brief 创建指定维度的单位方阵 (小驼峰命名)
 * @param size 方阵维度数
 * @return 对应的单位矩阵
 */
Matrix matrixIdentity(int size);

} // namespace kf_math
