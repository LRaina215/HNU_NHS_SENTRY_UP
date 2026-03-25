import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    sentry_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('bubble_protocol'), '/sentry_up_serial_launch.py'])
    )

    sentry_down = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('bubble_protocol'), '/sentry_down_serial_launch.py'])
    )


    return LaunchDescription([
        sentry_up,
        sentry_down
    ])