#!/usr/bin/env python3
"""
完整导航系统启动文件（包含智能脱困）

这个launch文件启动完整的nav2导航栈，并集成智能脱困功能
使用方法：
  ros2 launch smart_escape navigation_with_smart_escape.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 获取包路径
    smart_escape_dir = get_package_share_directory('smart_escape')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 启动配置
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    use_lifecycle_mgr = LaunchConfiguration('use_lifecycle_mgr')
    
    # 声明参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(smart_escape_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')
    
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(smart_escape_dir, 'behavior_trees', 'navigate_with_smart_escape.xml'),
        description='Full path to the behavior tree xml file to use')
    
    declare_map_subscribe_transient_local_cmd = DeclareLaunchArgument(
        'map_subscribe_transient_local',
        default_value='true',
        description='Whether to set the map subscriber QoS to transient local')
    
    declare_use_lifecycle_mgr_cmd = DeclareLaunchArgument(
        'use_lifecycle_mgr',
        default_value='true',
        description='Whether to use lifecycle manager')
    
    # 重写参数文件
    param_substitutions = {
        'autostart': autostart,
        'default_bt_xml_filename': bt_xml_file,
        'use_sim_time': use_sim_time,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)
    
    # 设置日志格式
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    # ==================== 智能脱困Action Server ====================
    # 这个节点必须首先启动，因为它提供智能脱困服务
    smart_escape_server_node = Node(
        package='smart_escape',
        executable='smart_escape_server',
        name='smart_escape_server',
        output='screen',
        parameters=[os.path.join(smart_escape_dir, 'config', 'smart_escape_params.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[
            ('/global_costmap/costmap', '/global_costmap/costmap'),
        ]
    )
    
    # ==================== Nav2核心节点 ====================
    
    # 生命周期管理器
    lifecycle_manager_node = Node(
        condition=IfCondition(use_lifecycle_mgr),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': autostart,
                     'node_names': ['controller_server',
                                    'planner_server',
                                    'recoveries_server',
                                    'bt_navigator',
                                    'waypoint_follower',
                                    'smart_escape_server']},  # 注意：包含smart_escape_server
                     {'use_sim_time': use_sim_time}])
    
    # 控制器服务器
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('cmd_vel', 'cmd_vel')])
    
    # 规划器服务器
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=[('costmap', 'global_costmap/costmap')])
    
    # 恢复行为服务器
    recoveries_server_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[configured_params])
    
    # 行为树导航器 - 使用包含SmartEscape的行为树
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params,
                    {'plugin_lib_names': ['smart_escape_nodes']}])  # 加载智能脱困BT节点插件
    
    # 航点跟随器
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params])
    
    # ==================== 代价地图节点 ====================
    
    # 全局代价地图
    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='costmap_2d_node',
        name='global_costmap',
        output='screen',
        parameters=[configured_params,
                    {'use_sim_time': use_sim_time}],
        remappings=[('costmap', 'global_costmap/costmap')])
    
    # 局部代价地图
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='costmap_2d_node',
        name='local_costmap',
        output='screen',
        parameters=[configured_params,
                    {'use_sim_time': use_sim_time}],
        remappings=[('costmap', 'local_costmap/costmap')])
    
    # 创建LaunchDescription
    ld = LaunchDescription()
    
    # 添加声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_subscribe_transient_local_cmd)
    ld.add_action(declare_use_lifecycle_mgr_cmd)
    ld.add_action(stdout_linebuf_envvar)
    
    # 添加节点 - 注意顺序：smart_escape_server必须先启动
    ld.add_action(smart_escape_server_node)
    ld.add_action(global_costmap_node)
    ld.add_action(local_costmap_node)
    ld.add_action(controller_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(recoveries_server_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(waypoint_follower_node)
    ld.add_action(lifecycle_manager_node)
    
    return ld
