# import os
# import yaml
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.substitutions import Command
# from launch_ros.actions import Node

# def generate_launch_description():
#     share_dir = get_package_share_directory('rm_description')
    
#     # 读取参数
#     config_file = os.path.join(share_dir, 'config', 'launch_params.yaml')
#     launch_params = yaml.safe_load(open(config_file))
#     xyz_param = launch_params['odom2camera']['xyz'].replace('"', '')
#     rpy_param = launch_params['odom2camera']['rpy'].replace('"', '')


#     # 导航部分

#     nav_urdf = os.path.join(share_dir, 'urdf', 'infantry.urdf.xacro')
#     robot_desc_nav = Command(['xacro ', nav_urdf])

#     rsp_nav = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='rsp_navigation',
#         parameters=[{'robot_description': robot_desc_nav, 'use_sim_time': True}], # 默认 50Hz，够导航用
#    #     remappings=[('/robot_description', '/robot_description_nav')]
#     )

#     # 自瞄部分

#     aim_urdf = os.path.join(share_dir, 'urdf', 'rm_gimbal.urdf.xacro')
#     robot_desc_aim = Command([
#         'xacro ', aim_urdf, 
#         ' xyz:="' + xyz_param + '"', 
#         ' rpy:="' + rpy_param + '"'
#     ])

#     rsp_aim = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='rsp_autoaim',
#         parameters=[{
#             'robot_description': robot_desc_aim,
#             'publish_frequency': 1000.0, 
#             'use_sim_time': True
#         }],
#    #     remappings=[('/robot_description', '/robot_description_aim')]
#     )
    
#     lidar_static_tf = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='lidar_static_tf',
#         arguments=['0', '0', '0.25', '0', '0', '0', 'base_footprint', 'sentry/base_footprint/lidar']
#     )
    
#     # return LaunchDescription([rsp_nav, rsp_aim])
#     return LaunchDescription([rsp_nav, lidar_static_tf])

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('rm_description')
    
    config_file = os.path.join(share_dir, 'config', 'launch_params.yaml')
    launch_params = yaml.safe_load(open(config_file))
    xyz_param = launch_params['odom2camera']['xyz'].replace('"', '')
    rpy_param = launch_params['odom2camera']['rpy'].replace('"', '')

    # 1. 导航底盘 URDF
    nav_urdf = os.path.join(share_dir, 'urdf', 'infantry.urdf.xacro')
    robot_desc_nav = Command(['xacro ', nav_urdf])
    rsp_nav = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rsp_navigation',
        parameters=[{'robot_description': robot_desc_nav, 'use_sim_time': True}], 
    )

    # 2. 自瞄云台 URDF
    aim_urdf = os.path.join(share_dir, 'urdf', 'rm_gimbal.urdf.xacro')
    robot_desc_aim = Command([
        'xacro ', aim_urdf, 
        ' xyz:="' + xyz_param + '"', 
        ' rpy:="' + rpy_param + '"'
    ])
    rsp_aim = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='rsp_autoaim',
        parameters=[{
            'robot_description': robot_desc_aim,
            'publish_frequency': 1000.0, 
            'use_sim_time': True
        }],
        remappings=[('/robot_description', '/robot_description_aim')]
    )
    
    # ==========================================
    # 3. 终极缝合 TF：打通所有断链！
    # ==========================================

    # A. 给 Nav2 提供绝对静止的地图原点
    stitch_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stitch_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # B. 解决 Gazebo 雷达改名导致的问题
    stitch_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stitch_lidar',
        arguments=['0', '0', '0', '0', '0', '0', 'livox_frame', 'sentry/base_footprint/lidar']
    )

    # C. 将云台 (gimbal_odom) 缝合到底盘 (base_link) 上
    # 就是这条线把上下半身连起来，之前断了！
    stitch_gimbal = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stitch_gimbal',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'gimbal_odom']
    )

    return LaunchDescription([
        rsp_nav, rsp_aim, stitch_map, stitch_lidar, stitch_gimbal
    ])