import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import RegisterEventHandler
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.events.process.process_exited import ProcessExited
from launch.launch_context import LaunchContext

from launch_ros.actions import Node

args_list = [
    DeclareLaunchArgument(
        'robot_type',
        default_value='sentry_up',
        description='Robot name',
        choices=[
            "sentry_up", "sentry_down", "infantry"
        ]),

    DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/stm32_up',
        description='Onboard serial port name'
    ),
]


def generate_launch_description():
    return LaunchDescription(args_list + [
        Node(
            package='bubble_protocol',
            name='bcp_core',
            executable='bcp_core',
            output="screen",
            emulate_tty=True,
            parameters=[{
                'robot_type': LaunchConfiguration('robot_type'),
                'serial_port': LaunchConfiguration('serial_port')
            }]
        )
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(on_exit=on_exit_restart)
        # )
    ])

# def on_exit_restart(event: ProcessExited, context: LaunchContext):
#
#     import time
#     time.sleep(3)
#     os.system("bash '/home/nv/Desktop/bubble/src/bubble_bringup/script/autostart.sh'")
