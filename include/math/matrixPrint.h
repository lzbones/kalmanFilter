#pragma once

#include "matrixDef.h"
#include <string>

namespace kf_math {

/**
 * @brief 打印输出矩阵内容（用于调试和教学展示）
 * @param A 要打印的矩阵
 * @param name 矩阵名称标签
 */
void matrixPrint(const Matrix& A, const std::string& name);

} // namespace kf_math
