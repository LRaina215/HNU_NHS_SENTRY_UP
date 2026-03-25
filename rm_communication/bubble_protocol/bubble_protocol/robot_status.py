'''
Robot communication status layer,
and this module may be refactored in the future.
This module will publish the robot status data sent by MCU
to onboard to DDS for other subscriber to receice.
机器人通讯状态层，并且这个模块将来可能会被重构。
该模块会发布MCU发送的机器人状态数据，载入 DDS 以供其他订户接收。
'''

from rclpy.node import Node
import rclpy
import math

from std_msgs.msg import Int8
import tf2_ros
import sensor_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import game_msgs.msg
import rmctrl_msgs
import rmctrl_msgs.msg
import numpy as np

from bubble_protocol.protocol import *


class RobotStatus():
    """Send robot status information.
        发送机器人状态信息。
    Attributes
    ----------
    node: `Node`
        Node of maintain robot status information.
    status: `STATUS`
        Robot current status data.
    """

    def __init__(self, status: dict, node: Node) -> None:
        self.node = node
        self.name = node.name
        self.status = status
        self.realtime_callback = REALTIME_CALLBACK
        self.status_init()

        # 定时发布非实时状态
        self.game_mode_pub = self.node.create_publisher(Int8, '/status/game_mode', 10)
        self.non_realtime_timer = self.node.create_timer(
             0.1, self.non_realtime_status)

    def status_init(self) -> None:
        ''' The function defines publishes required for the robot status.
            该函数定义机器人状态所需的发布。
        '''

        def gimbal_callback():
            gimbal_yaw = float(
                self.status["gimbal"]["gimbal_yaw"][IDX_VAL])
            gimbal_pitch = float(
                self.status["gimbal"]["gimbal_pitch"][IDX_VAL])
            joint_msg = sensor_msgs.msg.JointState()
            # 给时间戳作点补偿
            # duration = rclpy.time.Duration(seconds=0)
            #joint_msg.header.stamp = (self.node.get_clock().now() - duration).to_msg()
            joint_msg.header.stamp = self.node.get_clock().now().to_msg()

            joint_msg.name = ["yaw_joint", "pitch_joint"]
            joint_msg.position = [math.radians(gimbal_yaw), math.radians(gimbal_pitch)]
            self.joint_pub.publish(joint_msg)
            #print(f"pub yaw:{gimbal_yaw},pitch:{gimbal_pitch}")

        def shooter_callback():
            bullet_msg = rmctrl_msgs.msg.Shooter()
            bullet_msg.is_shoot = bool(
                self.status["barrel"]["is_shoot"][IDX_VAL])
            bullet_msg.bullet_vel = int(
                self.status["barrel"]["bullet_vel"][IDX_VAL])
            bullet_msg.remain_bullet = int(
                self.status["barrel"]["remain_bullet"][IDX_VAL])
            self.barrel_pub.publish(bullet_msg)

        def chassis_callback():
            odom_msg = nav_msgs.msg.Odometry()
            odom_msg.header.stamp = self.node.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_footprint"
            odom_msg.twist.twist.linear.x = float(
                self.status["chassis_ctrl"]["chassis_target_linear_x"][IDX_VAL])
            odom_msg.twist.twist.linear.y = float(
                self.status["chassis_ctrl"]["chassis_target_linear_y"][IDX_VAL])
            odom_msg.twist.twist.linear.z = float(
                self.status["chassis_ctrl"]["chassis_target_linear_z"][IDX_VAL])
            odom_msg.twist.twist.angular.x = float(
                self.status["chassis_ctrl"]["chassis_target_angular_x"][IDX_VAL])
            odom_msg.twist.twist.angular.y = float(
                self.status["chassis_ctrl"]["chassis_target_angular_y"][IDX_VAL])
            odom_msg.twist.twist.angular.z = float(
                self.status["chassis_ctrl"]["chassis_target_angular_z"][IDX_VAL])
            self.odom_pub.publish(odom_msg)
            # self.chassis_pub.publish(chassis_msg)

        def chassis_imu_callback():
            # 接收imu的原始数据
            #print("收到imu数据")
            chassis_imu_msg = sensor_msgs.msg.Imu()
            chassis_imu_msg.header.stamp = self.node.get_clock().now().to_msg()
            chassis_imu_msg.header.frame_id = "chassis_imu_link"
            # chassis_imu_msg.header.frame_id = "base_link"
            q = quaternion_from_euler(
                math.radians(float(
                    self.status["chassis_imu"]["chassis_imu_eul_rol"][IDX_VAL]) % 360),#绿
                -math.radians(float(
                    self.status["chassis_imu"]["chassis_imu_eul_pit"][IDX_VAL]) % 360),#红
                math.radians(float(
                    self.status["chassis_imu"]["chassis_imu_eul_yaw"][IDX_VAL]) % 360)#蓝
            )
            chassis_imu_msg.orientation.x = q[0]
            chassis_imu_msg.orientation.y = q[1]
            chassis_imu_msg.orientation.z = q[2]
            chassis_imu_msg.orientation.w = q[3]
            chassis_imu_msg.linear_acceleration.x = float(
                self.status["chassis_imu"]["chassis_imu_acc_x"][IDX_VAL])
            chassis_imu_msg.linear_acceleration.y = float(
                self.status["chassis_imu"]["chassis_imu_acc_y"][IDX_VAL])
            chassis_imu_msg.linear_acceleration.z = float(
                self.status["chassis_imu"]["chassis_imu_acc_z"][IDX_VAL])
            chassis_imu_msg.angular_velocity.x = math.radians(float(
                self.status["chassis_imu"]["chassis_imu_angle_x"][IDX_VAL]))
            chassis_imu_msg.angular_velocity.y = math.radians(float(
                self.status["chassis_imu"]["chassis_imu_angle_y"][IDX_VAL]))
            chassis_imu_msg.angular_velocity.z = math.radians(float(
                self.status["chassis_imu"]["chassis_imu_angle_z"][IDX_VAL]))
            #self.chassis_imu_pub.publish(chassis_imu_msg)

        def sentry_posture_feedback_callback():
            # 获取硬件解包后的最新真实反馈姿态值
            current_posture = int(self.status["sentry_posture_fdb"]["posture"][IDX_VAL])
            msg = Int8()
            msg.data = current_posture
            self.posture_feedback_pub.publish(msg)

        def chassis_odom_callback():
            # 发布tf2的odom变换
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = float(
                self.status["chassis_odom"]["odom_position_x"][IDX_VAL])
            t.transform.translation.y = float(
                self.status["chassis_odom"]["odom_position_y"][IDX_VAL])
            t.transform.translation.z = 0.0
            q = quaternion_from_euler(0, 0, float(
                self.status["chassis_odom"]["odom_angle"][IDX_VAL]))
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.odom_br.sendTransform(t)

            # print("收到odom数据")
            chassis_odom_msg = nav_msgs.msg.Odometry()
            chassis_odom_msg.header.stamp = self.node.get_clock().now().to_msg()
            chassis_odom_msg.header.frame_id = "odom"
            chassis_odom_msg.child_frame_id = "base_footprint"
            chassis_odom_msg.pose.pose.position.x = float(
                self.status["chassis_odom"]["odom_position_x"][IDX_VAL])
            chassis_odom_msg.pose.pose.position.y = float(
                self.status["chassis_odom"]["odom_position_y"][IDX_VAL])

            chassis_odom_msg.pose.pose.orientation.x = q[0]
            chassis_odom_msg.pose.pose.orientation.y = q[1]
            chassis_odom_msg.pose.pose.orientation.z = q[2]
            chassis_odom_msg.pose.pose.orientation.w = q[3]

            chassis_odom_msg.twist.twist.linear.x = float(
                self.status["chassis_odom"]["odom_velocity_x"][IDX_VAL])
            chassis_odom_msg.twist.twist.linear.y = float(
                self.status["chassis_odom"]["odom_velocity_y"][IDX_VAL])
            chassis_odom_msg.twist.twist.angular.z = float(
                self.status["chassis_odom"]["odom_angular_velocity"][IDX_VAL])
            self.odom_pub.publish(chassis_odom_msg)

        def quaternion_from_euler(ai, aj, ak):
            ai /= 2.0
            aj /= 2.0
            ak /= 2.0
            ci = math.cos(ai)
            si = math.sin(ai)
            cj = math.cos(aj)
            sj = math.sin(aj)
            ck = math.cos(ak)
            sk = math.sin(ak)
            cc = ci * ck
            cs = ci * sk
            sc = si * ck
            ss = si * sk

            q = np.empty((4,))
            q[0] = cj * sc - sj * cs
            q[1] = cj * ss + sj * cc
            q[2] = cj * cs - sj * sc
            q[3] = cj * cc + sj * ss

            return q

        # real-time publisher api  -- 0911猜測此處將發送最終IMU信息(先設置為步兵狀態 -- 接受C版發送的IMU等數據 -- 最後再傳給C版)
        if self.name == "infantry":
            self.joint_pub = self.node.create_publisher(
                sensor_msgs.msg.JointState, '/joint_states', 10)

            self.realtime_callback["gimbal"] = gimbal_callback
        
        # # 1020 合并自瞄与导航发送数据
        # elif self.name == "sentry_up":
        #     self.joint_pub = self.node.create_publisher(
        #         sensor_msgs.msg.JointState, '/joint_states', 10)
        #     self.barrel_pub = self.node.create_publisher(
        #         rmctrl_msgs.msg.Shooter, '/status/barrel', 10)

        #     self.realtime_callback["gimbal"] = gimbal_callback
        #     self.realtime_callback["barrel"] = shooter_callback
        # elif self.name == "sentry_down":
        #     self.odom_pub = self.node.create_publisher(
        #         nav_msgs.msg.Odometry, '/odom', 10)
        #     self.chassis_imu_pub = self.node.create_publisher(
        #         sensor_msgs.msg.Imu, '/imu', 10)
        #     self.odom_br = tf2_ros.TransformBroadcaster(self.node)
        #     # self.chassis_pub = self.node.create_publisher(
        #     #    rmctrl_msgs.msg.Chassis, '/status/chassis', 10)
        #     # self.chassis_odom_imu_pub = self.node.create_publisher(
        #     #     rmctrl_msgs.msg.Odom, '/status/chassis_odom_imu', 10)

        #     # self.realtime_callback["chassis_ctrl"] = chassis_callback
        #     self.realtime_callback["chassis_imu"] = chassis_imu_callback
        #     self.realtime_callback["chassis_odom"] = chassis_odom_callback

        elif self.name == "sentry":
            self.joint_pub = self.node.create_publisher(
                sensor_msgs.msg.JointState, '/joint_states', 10)
            self.barrel_pub = self.node.create_publisher(
                rmctrl_msgs.msg.Shooter, '/status/barrel', 10)
            self.odom_pub = self.node.create_publisher(
                nav_msgs.msg.Odometry, '/odom', 10)
            self.chassis_imu_pub = self.node.create_publisher(
                sensor_msgs.msg.Imu, '/imu', 10)
            self.odom_br = tf2_ros.TransformBroadcaster(self.node)
            self.posture_feedback_pub = self.node.create_publisher(
                Int8, '/status/sentry_posture_feedback', 10)
            
            # self.chassis_pub = self.node.create_publisher(
            #    rmctrl_msgs.msg.Chassis, '/status/chassis', 10)
            # self.chassis_odom_imu_pub = self.node.create_publisher(
            #     rmctrl_msgs.msg.Odom, '/status/chassis_odom_imu', 10)

            # self.realtime_callback["chassis_ctrl"] = chassis_callback

            self.realtime_callback["gimbal"] = gimbal_callback
            self.realtime_callback["barrel"] = shooter_callback
            self.realtime_callback["chassis_imu"] = chassis_imu_callback
            self.realtime_callback["chassis_odom"] = chassis_odom_callback
            self.realtime_callback["sentry_posture_fdb"] = sentry_posture_feedback_callback  # 这里也要改名字
        # non-realtime publisher api
        self.manifold_ctrl_pub = self.node.create_publisher(
            Int8, '/status/manifold_ctrl', 10)
        self.gameStatus_pub = self.node.create_publisher(
            game_msgs.msg.GameStatus, '/status/game', 10)
        self.zone_pub = self.node.create_publisher(
            game_msgs.msg.Zone, '/status/zone', 10)
        self.hp_pub = self.node.create_publisher(
            game_msgs.msg.RobotHP, '/status/robotHP', 10)
        self.game_mode_pub = self.node.create_publisher(
            Int8, '/status/game_mode', 10)

    def non_realtime_status(self) -> None:
        '''The function defines data publish rules of 
        robot's non-realtime status. It will executed perodically by timer. 
        The period is usually 1 second.
            该函数定义数据发布规则
            机器人的非实时状态。它将由定时器定期执行。
            周期通常为 1 秒。
        '''

        game_mode_msg = Int8()
        game_mode_msg.data = int(
            self.status["game_mode"]["game_mode"][IDX_VAL])
        self.game_mode_pub.publish(game_mode_msg)

        manifold_ctrl_msg = Int8()
        manifold_ctrl_msg.data = int(
            self.status["manifold_ctrl"]["mode_ctrl"][IDX_VAL])
        self.manifold_ctrl_pub.publish(manifold_ctrl_msg)

        gameStatus_msg = game_msgs.msg.GameStatus()
        gameStatus_msg.game_type = int(
            self.status["game_status"]["game_type"][IDX_VAL])
        gameStatus_msg.game_progress = int(
            self.status["game_status"]["game_progress"][IDX_VAL])
        gameStatus_msg.stage_remain_time = self.status["game_status"]["stage_remain_time"][IDX_VAL]
        self.gameStatus_pub.publish(gameStatus_msg)

        zone_msg = game_msgs.msg.Zone()
        zone_msg.f1_zone_status = self.status["ICRA_buff_debuff_zone"]["F1_zone_status"][IDX_VAL]
        zone_msg.f1_zone_buff_debuff_status = self.status[
            "ICRA_buff_debuff_zone"]["F1_zone_buff_debuff_status"][IDX_VAL]
        zone_msg.f2_zone_status = self.status["ICRA_buff_debuff_zone"]["F2_zone_status"][IDX_VAL]
        zone_msg.f2_zone_buff_debuff_status = self.status[
            "ICRA_buff_debuff_zone"]["F2_zone_buff_debuff_status"][IDX_VAL]
        zone_msg.f3_zone_status = self.status["ICRA_buff_debuff_zone"]["F3_zone_status"][IDX_VAL]
        zone_msg.f3_zone_buff_debuff_status = self.status[
            "ICRA_buff_debuff_zone"]["F3_zone_buff_debuff_status"][IDX_VAL]
        zone_msg.f4_zone_status = self.status["ICRA_buff_debuff_zone"]["F4_zone_status"][IDX_VAL]
        zone_msg.f4_zone_buff_debuff_status = self.status[
            "ICRA_buff_debuff_zone"]["F4_zone_buff_debuff_status"][IDX_VAL]
        zone_msg.f5_zone_status = self.status["ICRA_buff_debuff_zone"]["F5_zone_status"][IDX_VAL]
        zone_msg.f5_zone_buff_debuff_status = self.status[
            "ICRA_buff_debuff_zone"]["F5_zone_buff_debuff_status"][IDX_VAL]
        zone_msg.f6_zone_status = self.status["ICRA_buff_debuff_zone"]["F6_zone_status"][IDX_VAL]
        zone_msg.f6_zone_buff_debuff_status = self.status[
            "ICRA_buff_debuff_zone"]["F6_zone_buff_debuff_status"][IDX_VAL]
        zone_msg.red1_bullet_left = self.status["ICRA_buff_debuff_zone"]["red1_bullet_left"][IDX_VAL]
        zone_msg.red2_bullet_left = self.status["ICRA_buff_debuff_zone"]["red2_bullet_left"][IDX_VAL]
        zone_msg.blue1_bullet_left = self.status["ICRA_buff_debuff_zone"]["blue1_bullet_left"][IDX_VAL]
        zone_msg.blue2_bullet_left = self.status["ICRA_buff_debuff_zone"]["blue2_bullet_left"][IDX_VAL]
        self.zone_pub.publish(zone_msg)

        robotHP_msg = game_msgs.msg.RobotHP()
        robotHP_msg.red_1_robot_hp = int(
            self.status["robot_HP"]["red_1_robot_HP"][IDX_VAL])
        robotHP_msg.red_2_robot_hp = int(
            self.status["robot_HP"]["red_2_robot_HP"][IDX_VAL])
        robotHP_msg.red_3_robot_hp = int(
            self.status["robot_HP"]["red_3_robot_HP"][IDX_VAL])
        robotHP_msg.red_4_robot_hp = int(
            self.status["robot_HP"]["red_4_robot_HP"][IDX_VAL])
        robotHP_msg.red_5_robot_hp = int(
            self.status["robot_HP"]["red_5_robot_HP"][IDX_VAL])
        robotHP_msg.red_7_robot_hp = int(
            self.status["robot_HP"]["red_7_robot_HP"][IDX_VAL])
        robotHP_msg.red_outpost_hp = int(
            self.status["robot_HP"]["red_outpost_HP"][IDX_VAL])
        robotHP_msg.red_base_hp = int(
            self.status["robot_HP"]["red_base_HP"][IDX_VAL])
        robotHP_msg.blue_1_robot_hp = int(
            self.status["robot_HP"]["blue_1_robot_HP"][IDX_VAL])
        robotHP_msg.blue_2_robot_hp = int(
            self.status["robot_HP"]["blue_2_robot_HP"][IDX_VAL])
        robotHP_msg.blue_3_robot_hp = int(
            self.status["robot_HP"]["blue_3_robot_HP"][IDX_VAL])
        robotHP_msg.blue_4_robot_hp = int(
            self.status["robot_HP"]["blue_4_robot_HP"][IDX_VAL])
        robotHP_msg.blue_5_robot_hp = int(
            self.status["robot_HP"]["blue_5_robot_HP"][IDX_VAL])
        robotHP_msg.blue_7_robot_hp = int(
            self.status["robot_HP"]["blue_7_robot_HP"][IDX_VAL])
        robotHP_msg.blue_outpost_hp = int(
            self.status["robot_HP"]["blue_outpost_HP"][IDX_VAL])
        robotHP_msg.blue_base_hp = int(
            self.status["robot_HP"]["blue_base_HP"][IDX_VAL])
        self.hp_pub.publish(robotHP_msg)
