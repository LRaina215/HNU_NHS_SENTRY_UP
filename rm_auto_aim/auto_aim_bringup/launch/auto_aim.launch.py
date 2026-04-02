import os
import sys
from ament_index_python.packages import get_package_share_directory
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory('auto_aim_bringup'), 'launch'))


def generate_launch_description():
    def get_params(name):
        params_file = os.path.join(
            get_package_share_directory('auto_aim_bringup'),
            'config',
            'node_params',
            f'{name}_params.yaml'
        )
        if not os.path.exists(params_file):
            raise FileNotFoundError(f"Config file not found: {params_file}")
        return params_file

    armor_detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        emulate_tty=True,
        output='screen',
        name='armor_detector',
        parameters=[
            '/home/robomaster/shaobing/src/rm_auto_aim/auto_aim_bringup/config/node_params/armor_detector_params.yaml'
        ],
        prefix=['taskset -c 0-11 ']
    )

    armor_solver_node = Node(
        package='armor_solver',
        executable='armor_solver_node',
        emulate_tty=True,
        output='screen',
        name='armor_solver',
        parameters=[get_params('armor_solver')],
        prefix=['taskset -c 0-11 ']
    )

    return LaunchDescription([
        armor_detector_node,
        armor_solver_node,
    ])
