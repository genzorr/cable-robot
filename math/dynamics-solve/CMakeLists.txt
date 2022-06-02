cmake_minimum_required(VERSION 3.22)
project(dynamics_solve)

set(CMAKE_CXX_STANDARD 17)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})
add_executable(dynamics_solve main.cpp)
target_link_libraries(dynamics_solve ${ARMADILLO_LIBRARIES})
