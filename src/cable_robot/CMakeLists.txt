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
#find_package(tf2 REQUIRED)
#find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

# set dependencies
set(dependencies "rclcpp" #[["tf2" "tf2_ros"]] "std_msgs" "sensor_msgs" "geometry_msgs")

# GCU node
set(gcu_node_SRCS src/gcu.cpp)
set(gcu_node_HDRS include/cable_robot/gcu.h)

add_executable(general_control_unit ${gcu_node_SRCS} ${gcu_node_HDRS})
ament_target_dependencies(general_control_unit ${dependencies})
target_include_directories(general_control_unit PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
install(TARGETS general_control_unit
        DESTINATION lib/${PROJECT_NAME})

# small model plugin subscribes to joint efforts and applies them in Gazebo
add_library(cdpr_plugin SHARED src/cdpr_plugin.cpp include/cable_robot/cdpr_plugin.h)
target_include_directories(cdpr_plugin PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cdpr_plugin ${dependencies} "gazebo" "gazebo_ros")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(cdpr_plugin "${cpp_typesupport_target}")
install(TARGETS cdpr_plugin
        DESTINATION share/${PROJECT_NAME})

# copy dirs to install folder
install(DIRECTORY launch config rviz urdf sdf DESTINATION share/${PROJECT_NAME})

ament_package()
