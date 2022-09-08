cmake_minimum_required(VERSION 3.0.0)
project(armv2 VERSION 0.1.0)

include(CTest)
enable_testing()

include_directories(/usr/include/eigen3)
add_executable(armv2 src/main.cpp src/kinematics.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
