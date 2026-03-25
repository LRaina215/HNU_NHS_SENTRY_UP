# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="False") # 实体车设为 False

    fake_vel_transform_node = Node(
        package="fake_vel_transform",
        executable="fake_vel_transform_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_base_frame": "base_link",          # 真实的、在狂转的底盘坐标系
            "fake_robot_base_frame": "base_link_fake",# 虚拟的、不转的坐标系（你在 YAML 里填的那个）
            "input_cmd_vel_topic": "cmd_vel_nav",     # 订阅 Nav2 controller 输出的速度
            "output_cmd_vel_topic": "cmd_vel",        # 计算叠加后，发给你底盘控制节点的最终速度
            "init_spin_speed": 0.0,                   # 哨兵小陀螺的默认自旋角速度 (rad/s)，按需修改
        }],
    )

    ld = LaunchDescription()
    ld.add_action(fake_vel_transform_node)

    return ld
