cmake_minimum_required(VERSION 3.8)
project(cable_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# set dependencies
set(dependencies "rclcpp" "tf2" "tf2_ros" "std_msgs" "geometry_msgs")

# add sources/headers
set(gcu_node_SRCS
  src/general_control_unit.cpp
)
set(gcu_node_HDRS
  include/cable_robot/general_control_unit.h
)

# add execs
add_executable(general_control_unit ${gcu_node_SRCS} ${gcu_node_HDRS})
ament_target_dependencies(general_control_unit ${dependencies})
target_include_directories(general_control_unit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS general_control_unit
        DESTINATION lib/${PROJECT_NAME})

#add_executable(general_control_unit src/general_control_unit.cpp)
#ament_target_dependencies(general_control_unit rclcpp std_msgs geometry_msgs)
#target_include_directories(general_control_unit PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#install(TARGETS general_control_unit
#  DESTINATION lib/${PROJECT_NAME})

# copy dirs to install folder
install(DIRECTORY launch config rviz urdf sdf DESTINATION share/${PROJECT_NAME})

ament_package()
