cmake_minimum_required(VERSION 3.8)
project(cable_robot)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(VISP REQUIRED)
find_package(Eigen3)

# add custom messages
set(msg_files
        "msg/Tensions.msg"
        )
rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES geometry_msgs
        ADD_LINTER_TESTS
        )
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# GCU node
set(gcu_node_srcs
        src/gcu.cpp)
set(gcu_node_hdrs
        include/cable_robot/gcu.h)
add_executable(gcu ${gcu_node_srcs} ${gcu_node_hdrs})
ament_target_dependencies(gcu "rclcpp" "geometry_msgs")
target_include_directories(gcu PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
install(TARGETS gcu DESTINATION lib/${PROJECT_NAME})

# Core node
set(core_node_srcs
        src/core.cpp
        src/EigenQP.cpp)
set(core_node_hdrs
        include/cable_robot/core.h
        include/cable_robot/EigenQP.h)
add_executable(core ${core_node_srcs} ${core_node_hdrs})
ament_target_dependencies(core "rclcpp" "sensor_msgs" "geometry_msgs" "gazebo_msgs" "VISP")
target_include_directories(core PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
install(TARGETS core DESTINATION lib/${PROJECT_NAME})

# Joint control plugin for Gazebo
add_library(cdpr_plugin SHARED src/cdpr_plugin.cpp include/cable_robot/cdpr_plugin.h)
target_include_directories(cdpr_plugin PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cdpr_plugin "rclcpp" "sensor_msgs" "geometry_msgs" "gazebo" "gazebo_ros" "gazebo_msgs" "VISP")
target_link_libraries(cdpr_plugin "${cpp_typesupport_target}")
install(TARGETS cdpr_plugin
        DESTINATION share/${PROJECT_NAME})

# copy dirs to install folder
install(DIRECTORY launch config sdf DESTINATION share/${PROJECT_NAME})

ament_package()
