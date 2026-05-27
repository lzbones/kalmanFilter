#include "math/matrixInvert.h"
#include "math/matrixCreate.h"
#include <cmath>
#include <vector>
#include <algorithm>

namespace kf_math {

bool matrixInvert(const Matrix& A, Matrix& inv) {
    // 检查是否为方阵，只有方阵才有逆矩阵
    if (A.rows != A.cols) {
        return false;
    }
    
    int n = A.rows;
    
    // 创建增广矩阵 [A | I]，大小为 n x 2n
    // 左半部分为原矩阵 A，右半部分为单位矩阵 I
    std::vector<double> aug(n * 2 * n, 0.0);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            aug[i * (2 * n) + j] = A.data[i * n + j];
        }
        aug[i * (2 * n) + (n + i)] = 1.0;
    }

    // 开始高斯-约旦消元法
    for (int i = 0; i < n; ++i) {
        // 1. 寻找主元：在当前列 i 中，寻找从 i 行开始绝对值最大的元素所在行
        // 这样做是为了防止数值除以过小的值导致溢出，提高数值计算稳定性
        int pivotRow = i;
        double maxVal = std::abs(aug[i * (2 * n) + i]);
        for (int k = i + 1; k < n; ++k) {
            double val = std::abs(aug[k * (2 * n) + i]);
            if (val > maxVal) {
                maxVal = val;
                pivotRow = k;
            }
        }

        // 如果主元值接近于 0，说明该矩阵是奇异矩阵（不可逆），返回失败
        if (maxVal < 1e-9) {
            return false;
        }

        // 2. 交换当前行与主元所在行
        if (pivotRow != i) {
            for (int j = 0; j < 2 * n; ++j) {
                std::swap(aug[i * (2 * n) + j], aug[pivotRow * (2 * n) + j]);
            }
        }

        // 3. 将当前行归一化：使主元元素（对角线位置）的值变为 1
        double pivot = aug[i * (2 * n) + i];
        for (int j = i; j < 2 * n; ++j) {
            aug[i * (2 * n) + j] /= pivot;
        }

        // 4. 消元：用当前行去消除其他所有行（k != i）中第 i 列的元素，使其变为 0
        for (int k = 0; k < n; ++k) {
            if (k != i) {
                double factor = aug[k * (2 * n) + i];
                for (int j = i; j < 2 * n; ++j) {
                    aug[k * (2 * n) + j] -= factor * aug[i * (2 * n) + j];
                }
            }
        }
    }

    // 5. 提取增广矩阵右半部分 [I | A^-1] 的 A^-1，存入输出参数 inv 中
    inv = matrixCreate(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            inv.data[i * n + j] = aug[i * (2 * n) + (n + j)];
        }
    }

    return true;
}

} // namespace kf_math
