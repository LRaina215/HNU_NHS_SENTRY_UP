import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('rm_description')
    
    # 手动捏一个极其简单的 URDF 字符串，只包含这个有嫌疑的 STL 模型
    urdf_content = """
    <?xml version="1.0"?>
    <robot name="test_world">
      <link name="base_link">
        <visual>
          <geometry>
            <mesh filename="package://rm_description/models/rmuc_2025/meshes/rmuc_2025.stl"/>
          </geometry>
          <material name="white">
            <color rgba="1 1 1 1"/>
          </material>
        </visual>
      </link>
    </robot>
    """
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_content}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2'
    )

    return LaunchDescription([rsp_node, rviz_node])