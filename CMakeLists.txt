cmake_minimum_required(VERSION 2.8)
project(dmp_simple)

set(CMAKE_BUILD_TYPE DEBUG)

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)
message("here")
add_library(${PROJECT_NAME} SHARED
src/dmp/dmp.cpp
src/dmp/fourier_approx.cpp
src/dmp/linear_approx.cpp
src/dmp/radial_approx.cpp
)

add_executable(dmp_using src/dmp_using.cpp)
target_link_libraries(dmp_using ${EIGEN3_LIBS} ${PROJECT_NAME})