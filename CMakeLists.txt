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

    # 底层矩阵数学库 (小驼峰文件名)
    src/math/matrixCreate.cpp
    src/math/matrixIdentity.cpp
    src/math/matrixAdd.cpp
    src/math/matrixSub.cpp
    src/math/matrixMul.cpp
    src/math/matrixTranspose.cpp
    src/math/matrixInvert.cpp
    src/math/matrixPrint.cpp

    # 卡尔曼滤波器核心库 (小驼峰文件名)
    src/filter/kalmanInit.cpp
    src/filter/kalmanPredictState.cpp
    src/filter/kalmanPredictCovariance.cpp
    src/filter/kalmanPredict.cpp
    src/filter/kalmanCalcResidual.cpp
    src/filter/kalmanCalcInnovationCov.cpp
    src/filter/kalmanCalcGain.cpp
    src/filter/kalmanCorrectState.cpp
    src/filter/kalmanCorrectCovariance.cpp
    src/filter/kalmanUpdate.cpp
)

# 生成可执行文件
add_executable(kalman_demo ${SOURCES})
