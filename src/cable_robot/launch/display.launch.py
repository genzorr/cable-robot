import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Set environment variables
    os.environ['ROS_LOG_DIR'] = '~/dev/cable-robot/logs'
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} ({severity}) [{name}] {message}' # ({function_name}():{line_number})'
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

    package_path = get_package_share_path('cable_robot')
    model_config = os.path.join(package_path, 'urdf', 'cable_robot.urdf')
    rviz_config = os.path.join(package_path, 'rviz', 'urdf.rviz')
    gcu_config = os.path.join(package_path, 'config', 'gcu.yaml')

    robot_description = ParameterValue(Command(['xacro ', model_config]), value_type=str)

    gcu_node = Node(
        package='cable_robot',
        executable='general_control_unit',
        namespace='cable_robot',
        name='gcu',
        output='screen',
        parameters=[gcu_config],
        arguments=['--ros-args', '--log-level', 'cable_robot.gcu:=debug']   # changing log level per node
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # )
    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        gcu_node,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
