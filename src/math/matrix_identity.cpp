#include "math/matrix_identity.h"
#include "math/matrix_create.h"

namespace kf_math {

Matrix matrixIdentity(int size) {
    Matrix m = matrixCreate(size, size, 0.0);
    for (int i = 0; i < size; ++i) {
        m.data[i * size + i] = 1.0;
    }
    return m;
}

} // namespace kf_math
