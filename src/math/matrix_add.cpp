#include "math/matrix_add.h"
#include "math/matrix_create.h"
#include <stdexcept>

namespace kf_math {

Matrix matrixAdd(const Matrix& A, const Matrix& B) {
    if (A.rows != B.rows || A.cols != B.cols) {
        throw std::runtime_error("Matrix dimensions must match for addition!");
    }
    
    Matrix C = matrixCreate(A.rows, A.cols);
    for (size_t i = 0; i < A.data.size(); ++i) {
        C.data[i] = A.data[i] + B.data[i];
    }
    return C;
}

} // namespace kf_math
