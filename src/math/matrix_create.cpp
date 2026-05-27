#include "math/matrix_create.h"

namespace kf_math {

Matrix matrixCreate(int rows, int cols, double val) {
    Matrix m;
    m.rows = rows;
    m.cols = cols;
    m.data.assign(rows * cols, val);
    return m;
}

} // namespace kf_math
