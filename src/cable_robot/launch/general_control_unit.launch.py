import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gcu_config = os.path.join(get_package_share_directory('cable_robot'), 'config', 'gcu.yaml')

    return LaunchDescription([
        Node(
            package='cable_robot',
            executable='general_control_unit',
            namespace='cable_robot',
            name='gcu',
            parameters=[gcu_config],
            arguments=['--ros-args', '--log-level', 'cable_robot.gcu:=debug']   # changing log level per node
        )
    ])
