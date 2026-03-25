#!/usr/bin/env python3
"""
智能脱困系统启动文件

启动智能脱困Action Server节点
需要与nav2导航栈一起使用
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('smart_escape')
    
    # 参数声明
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'smart_escape_params.yaml'),
        description='Full path to the ROS2 parameters file to use')
    
    # 设置日志格式
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    # 智能脱困Action Server节点
    smart_escape_server_node = Node(
        package='smart_escape',
        executable='smart_escape_server',
        name='smart_escape_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/global_costmap/costmap', '/global_costmap/costmap'),
        ]
    )
    
    # 创建LaunchDescription
    ld = LaunchDescription()
    
    # 添加声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(stdout_linebuf_envvar)
    
    # 添加节点
    ld.add_action(smart_escape_server_node)
    
    return ld
