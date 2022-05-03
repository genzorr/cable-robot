import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# this is a nested launch file; don't use only this
def generate_launch_description():
    gcu_config = os.path.join(get_package_share_directory('cable_robot'), 'config', 'gcu.yaml')

    return LaunchDescription([
        Node(
            package='cable_robot',
            executable='gcu',
            namespace='cable_robot',
            name='gcu',
            output='screen',
            parameters=[
                gcu_config,
                {'use_sim_time': True}
            ],
            arguments=['--ros-args', '--log-level', 'cable_robot.gcu:=debug'],   # changing log level per node
            # prefix=['gdb -ex run --args'],
        )
    ])
