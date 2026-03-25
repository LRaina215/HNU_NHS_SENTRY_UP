'''
Copyright (c) 2025 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: AI Assistant
Date: 2025-11-09
FilePath: /bubble/src/rm_communication/bubble_decision/launch/slip_detection_launch.py
Description: 打滑检测启动文件
'''

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 打滑检测节点
    slip_detection_node = Node(
        package='bubble_decision',
        executable='slip_detection',
        name='slip_detection',
        output='screen',
        parameters=[{
            'slip_threshold': 0.3,      # 打滑阈值
            'window_size': 10,          # 滑动窗口大小
            'min_speed_threshold': 0.1, # 最小速度阈值
        }],
        remappings=[
            ('/chassis', '/chassis'),   # 底盘控制话题
            ('/odom', '/odom'),         # 里程计话题
        ]
    )
    
    return LaunchDescription([
        slip_detection_node,
    ])
