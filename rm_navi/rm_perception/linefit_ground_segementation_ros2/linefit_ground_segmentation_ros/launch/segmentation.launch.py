import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Getting directories and launch-files
    bringup_dir = get_package_share_directory('linefit_ground_segmentation_ros')
    params_file = os.path.join(bringup_dir, 'launch', 'segmentation_params.yaml')

    # Nodes launching commands
    node_start_cmd = Node(
            package='linefit_ground_segmentation_ros',
            executable='ground_segmentation_node',
            output='screen',
            parameters=[params_file],
            # 【修改】添加 remappings
            # 虽然我们在 yaml 里改了 input_topic，但有些版本的代码只认 remapping
            # 这样写是双保险：
            remappings=[
                # 告诉节点：凡是你要订阅 "cloud_in" 或 "input_topic" 的地方，
                # 都给我去订阅 "/livox/lidar_no_body"
                ('cloud_in', '/livox/lidar_no_body'),
                ('input_topic', '/livox/lidar_no_body'),
                
                # 输出重映射 (可选，保持和 yaml 一致即可)
                ('obstacles', '/segmentation/obstacle'),
                ('ground', '/segmentation/ground')
            ]
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(node_start_cmd)

    return ld