import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Set environment variables
    os.environ['ROS_LOG_DIR'] = '~/dev/cable-robot/logs'
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} ({severity}) [{name}] {message}' # ({function_name}():{line_number})'
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

    package_path = get_package_share_path('cable_robot')
    model_config = os.path.join(package_path, 'sdf', 'cable_robot.sdf')
    gcu_config = os.path.join(package_path, 'config', 'gcu.yaml')

    gcu_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('cable_robot'), 'launch'), '/general_control_unit.launch.py'])
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'])
    )
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=[
            '-entity', 'cable_robot',
            '-file', model_config
        ],
        output='screen'
    )

    return LaunchDescription([
        # gcu_node,
        gazebo,
        spawn_entity
    ])
