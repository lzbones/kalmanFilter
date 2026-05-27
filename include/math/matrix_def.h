#pragma once

#include <vector>

namespace kf_math {

/**
 * @brief 矩阵数据结构体 (大驼峰命名)
 * 采用一维 vector 存储二维矩阵数据（行主序存储）
 */
struct Matrix {
    int rows;                 ///< 矩阵行数
    int cols;                 ///< 矩阵列数
    std::vector<double> data; ///< 矩阵元素数据
};

} // namespace kf_math
