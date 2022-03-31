import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # Set environment variables
    os.environ['ROS_LOG_DIR'] = '~/dev/cable-robot/logs'
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} ({severity}) [{name}] {message}' # ({function_name}():{line_number})'
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

    # Load configurations
    gcu_config = os.path.join(get_package_share_directory('cable_robot'), 'config', 'gcu.yaml')

    return LaunchDescription([
        # DeclareLaunchArgument(name='log_level', default_value='debug'),
        Node(
            package='cable_robot',
            executable='general_control_unit',
            namespace='cable_robot',
            name='gcu',
            parameters=[gcu_config],
            arguments=['--ros-args', '--log-level', 'cable_robot.gcu:=debug']   # changing log level per node
        )
    ])
