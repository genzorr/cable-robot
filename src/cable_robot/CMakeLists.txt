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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(VISP REQUIRED)

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

# set dependencies
set(dependencies "rclcpp" "std_msgs" "sensor_msgs" "geometry_msgs"
        "gazebo" "gazebo_ros" "gazebo_msgs"
        "VISP")

# GCU node
set(gcu_node_SRCS
        src/gcu.cpp
        src/tda.cpp
    )
set(gcu_node_HDRS
        include/cable_robot/gcu.h
        include/cable_robot/tda.h
    )

add_executable(gcu ${gcu_node_SRCS} ${gcu_node_HDRS})
ament_target_dependencies(gcu ${dependencies})
target_include_directories(gcu PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
install(TARGETS gcu
        DESTINATION lib/${PROJECT_NAME})

# small model plugin subscribes to joint efforts and applies them in Gazebo
add_library(cdpr_plugin SHARED src/cdpr_plugin.cpp include/cable_robot/cdpr_plugin.h)
target_include_directories(cdpr_plugin PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cdpr_plugin ${dependencies})
target_link_libraries(cdpr_plugin "${cpp_typesupport_target}")
install(TARGETS cdpr_plugin
        DESTINATION share/${PROJECT_NAME})

# copy dirs to install folder
install(DIRECTORY launch config sdf DESTINATION share/${PROJECT_NAME})

ament_package()
