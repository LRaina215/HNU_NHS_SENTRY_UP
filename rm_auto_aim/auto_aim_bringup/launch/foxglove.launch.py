from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge', # 如果报错找不到，试着改成 'foxglove_bridge_node'
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'tls': False,
                'topic_whitelist': ['.*'], # 允许所有话题
                'service_whitelist': ['.*'],
                'param_whitelist': ['.*'],
                'client_topic_whitelist': ['.*'],
            }]
        )
    ])
