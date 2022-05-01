import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Set environment variables
    os.environ['ROS_LOG_DIR'] = '~/dev/cable-robot/logs'
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} ({severity}) [{name}] {message}' # ({function_name}():{line_number})'
    os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

    gcu_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('cable_robot'), 'launch'),
            '/gcu.launch.py'])
    )

    return LaunchDescription([
        gcu_node
    ])
