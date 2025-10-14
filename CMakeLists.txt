cmake_minimum_required(VERSION 3.21)

# Set default build type to release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(ltv-drivetrain-controllers)

file(GLOB_RECURSE frc_cpp_src src/frc/*.cpp)

add_executable(
    ltv-diff-drive-controller
    ${frc_cpp_src}
    src/ltv_diff_drive_controller_main.cpp
)
target_compile_options(
    ltv-diff-drive-controller
    PRIVATE -Wall -Wextra -pedantic
)
set_property(
    TARGET ltv-diff-drive-controller
    PROPERTY COMPILE_WARNING_AS_ERROR ON
)
target_compile_features(ltv-diff-drive-controller PUBLIC cxx_std_14)
target_include_directories(ltv-diff-drive-controller PRIVATE src eigen3)

add_executable(
    ltv-unicycle-controller
    ${frc_cpp_src}
    src/ltv_unicycle_controller_main.cpp
)
target_compile_options(ltv-unicycle-controller PRIVATE -Wall -Wextra -pedantic)
set_property(
    TARGET ltv-unicycle-controller
    PROPERTY COMPILE_WARNING_AS_ERROR ON
)
target_compile_features(ltv-unicycle-controller PUBLIC cxx_std_14)
target_include_directories(ltv-unicycle-controller PRIVATE src eigen3)
