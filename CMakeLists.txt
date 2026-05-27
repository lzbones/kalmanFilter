cmake_minimum_required(VERSION 3.10)
project(kalman_demo CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含头文件路径
include_directories(include)

# 收集所有源文件
set(SOURCES
    # 主函数
    main.cpp

    # 底层矩阵数学库
    src/math/matrix_create.cpp
    src/math/matrix_identity.cpp
    src/math/matrix_add.cpp
    src/math/matrix_sub.cpp
    src/math/matrix_mul.cpp
    src/math/matrix_transpose.cpp
    src/math/matrix_invert.cpp
    src/math/matrix_print.cpp

    # 卡尔曼滤波器核心库
    src/filter/kalman_init.cpp
    src/filter/kalman_predict_state.cpp
    src/filter/kalman_predict_covariance.cpp
    src/filter/kalman_predict.cpp
    src/filter/kalman_calc_residual.cpp
    src/filter/kalman_calc_innovation_cov.cpp
    src/filter/kalman_calc_gain.cpp
    src/filter/kalman_correct_state.cpp
    src/filter/kalman_correct_covariance.cpp
    src/filter/kalman_update.cpp
)

# 生成可执行文件
add_executable(kalman_demo ${SOURCES})
