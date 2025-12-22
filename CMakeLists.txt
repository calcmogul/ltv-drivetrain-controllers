cmake_minimum_required(VERSION 3.21)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(ltv-drivetrain-controllers)

include(FetchContent)

set(EIGEN_BUILD_CMAKE_PACKAGE TRUE)
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 5.0.1
    SYSTEM
)
FetchContent_MakeAvailable(Eigen3)

file(GLOB_RECURSE wpimath_cpp_src src/wpi/math/*.cpp)

add_executable(ltv-diff-drive ${wpimath_cpp_src} src/ltv_diff_drive_main.cpp)
target_compile_options(ltv-diff-drive PRIVATE -Wall -Wextra -pedantic)
set_property(TARGET ltv-diff-drive PROPERTY COMPILE_WARNING_AS_ERROR ON)
target_compile_features(ltv-diff-drive PUBLIC cxx_std_14)
target_include_directories(ltv-diff-drive PRIVATE src)
target_link_libraries(ltv-diff-drive PRIVATE Eigen3::Eigen)

add_executable(ltv-unicycle ${wpimath_cpp_src} src/ltv_unicycle_main.cpp)
target_compile_options(ltv-unicycle PRIVATE -Wall -Wextra -pedantic)
set_property(TARGET ltv-unicycle PROPERTY COMPILE_WARNING_AS_ERROR ON)
target_compile_features(ltv-unicycle PUBLIC cxx_std_14)
target_include_directories(ltv-unicycle PRIVATE src)
target_link_libraries(ltv-unicycle PRIVATE Eigen3::Eigen)
