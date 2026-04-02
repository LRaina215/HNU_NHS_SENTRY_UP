import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    aiming_param_dir = LaunchConfiguration(
        'params_file', default=os.path.join(
            get_package_share_directory('bubble_decision'), 'config', 'config.yaml')
    )

    arg_list = [
        DeclareLaunchArgument(
            'params_file',
            default_value=aiming_param_dir,
            description='Full path to aiming parameters file'
        ),
        # === 新增：暴露 team_color 参数 ===
        DeclareLaunchArgument(
            'team_color',
            default_value='red',
            description='Team color, optional: red | blue'
        ),
    ]

    # 根据你的原版代码，兼容旧版 ROS2 的写法
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription(arg_list+[
            DeclareLaunchArgument(
                'robot_type',
                default_value='sentry_down',
                description='Robot name, optional sentry_up| sentry_down| infantry| engineer| hero| air| radar| gather| standard.'
            ),
            Node(
                package='bubble_decision',
                node_name='bubble_decision',
                node_executable='decision',
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        'robot_type': LaunchConfiguration('robot_type'),
                        'team_color': LaunchConfiguration('team_color') # 传入参数
                    }
                ]
            )
        ])
    else:
        return LaunchDescription(arg_list+[
            DeclareLaunchArgument(
                'robot_type',
                default_value='sentry_down',
                description='Robot name, optional sentry_up| sentry_down| infantry| engineer| hero| air| radar| gather| standard.'
            ),
            Node(
                package='bubble_decision',
                name='bubble_decision',
                executable='decision',
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        'robot_type': LaunchConfiguration('robot_type'),
                        'team_color': LaunchConfiguration('team_color') # 传入参数
                    }
                ]
            )
        ])
