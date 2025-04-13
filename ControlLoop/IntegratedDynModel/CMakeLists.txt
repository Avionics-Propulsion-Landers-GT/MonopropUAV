cmake_minimum_required(VERSION 3.10)
project(MonopropSimulation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set optimization level
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

# Source files
set(SOURCES
    integratedDynMode.cpp
    ../CustomLinear/Matrix.cpp
    ../CustomLinear/Vector.cpp
    ../CustomLinear/Quaternion.cpp
    ../Loop/loop.cpp
    ../Loop/init.cpp
    ../LQR/lqr.cpp
    ../LQR/calculateA.cpp
    ../LQR/calculateB.cpp
    ../LQR/solveCARE.cpp
    ../Filters/EKFs/ExtendedKalmanFilterGeneral.cpp
    ../Filters/EKFs/EKF_xy.cpp
    ../Filters/EKFs/EKF_z.cpp
    ../Filters/Madgwick/updatedMadgwick.cpp
)

# Create executable
add_executable(monoprop_sim ${SOURCES})

# Find and link required libraries
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
target_link_libraries(monoprop_sim ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})

# Include directories
target_include_directories(monoprop_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../CustomLinear
    ${CMAKE_CURRENT_SOURCE_DIR}/../Loop
    ${CMAKE_CURRENT_SOURCE_DIR}/../LQR
    ${CMAKE_CURRENT_SOURCE_DIR}/../Filters
)