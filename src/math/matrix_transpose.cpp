#include "math/matrix_transpose.h"
#include "math/matrix_create.h"

namespace kf_math {

Matrix matrixTranspose(const Matrix& A) {
    Matrix B = matrixCreate(A.cols, A.rows);
    for (int i = 0; i < A.rows; ++i) {
        for (int j = 0; j < A.cols; ++j) {
            B.data[j * A.rows + i] = A.data[i * A.cols + j];
        }
    }
    return B;
}

} // namespace kf_math
