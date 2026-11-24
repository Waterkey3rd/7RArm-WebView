cmake_minimum_required(VERSION 3.22)
project(hostlink_trajectory LANGUAGES CXX)
set(FIRMWARE_ROOT "" CACHE PATH "Path to a dc-unit-01-d firmware checkout")
if(NOT EXISTS "${FIRMWARE_ROOT}/App/RoboArm/DualArmProfiles.hpp" OR
   NOT EXISTS "${FIRMWARE_ROOT}/SheriffOS/third/eigen/Eigen/Core" OR
   NOT EXISTS "${FIRMWARE_ROOT}/SheriffOS/third/units/include/units.h")
    message(FATAL_ERROR "Set FIRMWARE_ROOT to dc-unit-01-d with its submodules initialized")
endif()
add_executable(hostlink_trajectory preview.cpp)
target_compile_features(hostlink_trajectory PRIVATE cxx_std_20)
target_compile_definitions(hostlink_trajectory PRIVATE UNIT_LIB_DEFAULT_TYPE=float)
target_compile_options(hostlink_trajectory PRIVATE -Wall -Wextra)
target_include_directories(hostlink_trajectory PRIVATE . "${FIRMWARE_ROOT}/App"
    "${FIRMWARE_ROOT}/SheriffOS/third/eigen" "${FIRMWARE_ROOT}/SheriffOS/third/units/include")
