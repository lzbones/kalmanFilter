#include "math/matrixMul.h"
#include "math/matrixCreate.h"
#include <stdexcept>

namespace kf_math {

Matrix matrixMul(const Matrix& A, const Matrix& B) {
    if (A.cols != B.rows) {
        throw std::runtime_error("Matrix dimensions are incompatible for multiplication! A.cols must equal B.rows.");
    }
    
    Matrix C = matrixCreate(A.rows, B.cols, 0.0);
    
    // 采用 i-k-j 循环顺序，提高缓存命中率（行主序存储下友好）
    for (int i = 0; i < A.rows; ++i) {
        for (int k = 0; k < A.cols; ++k) {
            double temp = A.data[i * A.cols + k];
            for (int j = 0; j < B.cols; ++j) {
                C.data[i * B.cols + j] += temp * B.data[k * B.cols + j];
            }
        }
    }
    return C;
}

} // namespace kf_math
