#include "math/matrix_print.h"
#include <cstdio>

namespace kf_math {

void matrixPrint(const Matrix& A, const std::string& name) {
    std::printf("--- Matrix: %s (%dx%d) ---\n", name.c_str(), A.rows, A.cols);
    for (int i = 0; i < A.rows; ++i) {
        std::printf("[");
        for (int j = 0; j < A.cols; ++j) {
            std::printf(" %8.4f", A.data[i * A.cols + j]);
        }
        std::printf(" ]\n");
    }
    std::printf("\n");
}

} // namespace kf_math
