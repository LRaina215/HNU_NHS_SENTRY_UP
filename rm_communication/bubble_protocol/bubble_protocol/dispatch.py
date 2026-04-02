'''
Robot communication dispatch layer, 
this module is the bridge between DDS and serial port hardware. 
Through the module, 
a BCP frame can be initialized to receive the data transmitted by DDS 
and send the data to MCU. 
At the same time, the data received by the onboard computer will also be 
transmitted to the DDS through this module 
to be received by other subscribers.
机器人通信调度层，该模块是DDS与串口硬件之间的桥梁。
通过模块，可以初始化一个BCP帧来接收DDS传输的数据并将数据发送给MCU。
同时，车载电脑接收到的数据也会通过该模块传输到DDS，由其他订户接收。
'''
import math
from math import pi
import time
import serial

import os
import sys
import subprocess

import geometry_msgs
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import PoseWithCovariance, Twist, Quaternion
from std_msgs.msg import Int8
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from bubble_protocol.hardware import RobotSerial
from bubble_protocol.robot_status import RobotStatus
from rmctrl_msgs.msg import Chassis, Gimbal, Shooter
from rm_interfaces.msg import GimbalCmd
from rm_interfaces.msg import SerialReceiveData
import tf2_ros
from tf2_ros import TransformBroadcaster
from rclpy.time import Time

\

class RobotAPI(Node):
    """Generate a new BCP core.

    Attributes
    ----------
    robot_status: `RobotStatus`
        The robot current status.
    robot_serial: `Serial`
        Onboard serial port object.

    Examples
    --------
    >>> from bubble_protocol.dispatch import RobotAPI
    >>> core = RobotAPI()
    """

    def __init__(self, name="standard") -> None:
        """
        Generate a new BCP core.
        初始化一个新的 BCP core类
        """
        super().__init__("BCP_Core")

        # Get params from ROS2 launch system
        self.declare_parameter('robot_type', 'None')
        self.declare_parameter('serial_port', 'None')
        self.name = self.get_parameter(
            'robot_type').get_parameter_value().string_value
        self.serial_port = self.get_parameter(
            'serial_port').get_parameter_value().string_value
        if self.name != 'None':
            self.get_logger().info(f'Robot {self.name} has been initialized...')
        else:
            self.get_logger().error(
                f'No robot has been initialized, please check the parameter Settings!')
            self.get_logger().error(
                f'Robot type:{name}')
            raise ValueError(
                'No robot has been initialized, please check the parameter Settings!')

        # Init robot core hardware
        serial_opened = False
        while not serial_opened:
            try:
                self.robot_serial = RobotSerial(self.name, port=self.serial_port)
                self.get_logger().info(f'Serial Port {self.serial_port} has been initialized...')
                serial_opened = True
            except Exception as e:
                self.get_logger().error(
                    f'Open serial port error, try to reopen port:{self.serial_port}, info: {e}')
                time.sleep(3)

        #init port check param
        self.last_yaw = 0.0
        self.last_pitch = 0.0
        self.last_timestamp = 0.0
        #初始化imu数据
        self.get_yaw = 0.0
        self.get_pitch = 0.0
        self.get_roll = 0.0
        self.fire_advice = 0.0
        # 发布机器人状态模块
        self.robot_status = RobotStatus(self.robot_serial.status, self)
        self.robot_serial.realtime_pub = self.robot_status.realtime_callback
        self.robot_serial.serial_done = True
        #创建tf广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.01, self.broadcast_transform)
        # init core api
        self.api_init()
        # init expanded api
        self.init_robot()
        # Init robot tx/rx/heartbeat timer
        period = 100
        # self.red_blue_timer = self.create_timer(1/period, self.robot_serial.red_blue_info_callback)
        self.imu_gimbal_timer = self.create_timer(1/period, self.robot_serial.imu_gimbal_callback)
        self.uart_timer = self.create_timer(1/period, self.robot_serial.process)
        self.uartrx_timer = self.create_timer(
            1/period, self.robot_serial.rx_function)
        

        # 心跳模块
        # self.heartbeat_timer = self.create_timer(0.5, self.heartbeat)
        #
        # self.test_timer = self.create_timer(1,self.test_callback)
        
        
    def test_callback(self):
        mode = 1
        yaw = 30
        pitch = 20
        roll = 10
        self.get_logger().info(f"sending:{yaw},{pitch},{roll}")
        self.robot_serial.send_data(
            "gimbal",
            [mode,math.degrees(yaw),math.degrees(pitch),math.degrees(roll),0,0,0])
	    
    def api_init(self) -> None:
        '''General information API of robot definition.
            机器人定义通用信息API。
        '''
        # heartbeat data initalized
        self.heartbeat_time = 0
    
        # 0120_LJH 合并自瞄导航数据 
        # # subscriber api 订阅模式控制信息
        # if self.name == "infantry" or self.name == "sentry_up":
        #     # self.gimbal_sub = self.create_subscription(
        #     #     Target, '/processor/target', self.gimbal_callback, qos.qos_profile_sensor_data)
        #     # self.gimbal_sub = self.create_subscription(
        #     #     Target, '/processor/target', self.gimbal_callback, 10)
            
        #     #set the compatible QoS quality
        #     qos_profile = QoSProfile(
        #         reliability=QoSReliabilityPolicy.RELIABLE,
        #         durability=QoSDurabilityPolicy.VOLATILE,
        #         depth=20
        #     )
            
        #     # 訂閱純視覺(無IMU)得到的雲台轉動角度信息
        #     self.gimbal_sub = self.create_subscription(
        #         GimbalCmd, 'armor_solver/cmd_gimbal', self.gimbal_callback, qos_profile)
        #     # 訂閱純視覺下的開火狀態信息
        #     self.barrel_sub = self.create_subscription(
        #         Shooter, '/core/shooter_api', self.barrel_callback, 10)
        #     self.imu_tf_sub = self.create_subscription(SerialReceiveData, 'serial/receive', self.getImu_callback, 10)
        #     # print("Starting subscriber!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

        # elif self.name == "sentry_down":
        #     self.chassis_sub = self.create_subscription(
        #         Twist, '/cmd_vel', self.ex_chassis_callback, 10)

        # self.gimbal_sub = self.create_subscription(
        #     Target, '/processor/target', self.gimbal_callback, qos.qos_profile_sensor_data)
        # self.gimbal_sub = self.create_subscription(
        #     Target, '/processor/target', self.gimbal_callback, 10)
        
        #set the compatible QoS quality
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=20
        )
        
        # 訂閱純視覺(無IMU)得到的雲台轉動角度信息
        self.gimbal_sub = self.create_subscription(
           GimbalCmd, 'armor_solver/cmd_gimbal', self.gimbal_callback, qos_profile)
        
        # 訂閱純視覺下的開火狀態信息
        self.barrel_sub = self.create_subscription(
           Shooter, '/core/shooter_api', self.barrel_callback, 10)
        
        # 0120 优化自收发，直接读取另一个类中的值(回退)
        # 订阅IMU上发信息
        self.imu_tf_sub = self.create_subscription(SerialReceiveData, 'serial/receive', self.getImu_callback, 10)
        # print("Starting subscriber!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

        # 订阅导航下发给底盘的数据
        self.chassis_sub = self.create_subscription(
            Twist, '/cmd_vel', self.ex_chassis_callback, 10)

        # 哨兵姿态话题订阅者
        self.posture_sub = self.create_subscription(
            Int8, 
            '/manual/sentry_posture', 
            self.posture_callback, 
            10
        )
        self.get_logger().info("已启动手动姿态监听 (/manual/sentry_posture)")


    def heartbeat(self):
        '''heartbeat function, send heartbeat frames periodically.
            心跳函数，周期性发送心跳帧。
        '''
        self.robot_serial.send_data("heartbeat", [self.heartbeat_time])
        # self.get_logger().info("heartbeat_time: {},time:{}".format(self.heartbeat_time,time.time()))
        self.heartbeat_time = 1 if self.heartbeat_time == 0 else 0

    def mode_ctrl_callback(self, msg: Int8) -> None:
        '''mode control function, send mode infomation to MCU.
            模式控制函数,发送模式信息给MCU。
        Parameters
        ------------
        msg: `Int8`
            A mode message is received
        '''
        # self.get_logger().info("recived data mode value: {}".format(msg.data))
        self.robot_serial.send_data("mode", [msg.data])

    def posture_callback(self, msg: Int8):
        """
        接收命令行发来的姿态切换指令，并通过串口下发给 MCU。
        1: 进攻姿态, 2: 防御姿态, 3: 移动姿态
        """
        posture_cmd = msg.data
        if posture_cmd in [1, 2, 3]:
            # 调用 hardware.py 中的 send_data，打包成 BCP 帧发送
            self.robot_serial.send_data("sentry_posture_cmd", [posture_cmd])
            
            pose_dict = {1: "进攻(Attack)", 2: "防御(Defense)", 3: "移动(Moving)"}
            self.get_logger().info(f"成功向 C 板下发姿态切换指令: {pose_dict[posture_cmd]}")
        else:
            self.get_logger().warn(f"收到无效指令: {posture_cmd}，拒绝下发！")

    def gimbal_callback(self, msg: GimbalCmd) -> None:
        """gimbal function, send gimbal infomation to MCU.
            云台功能,发送云台信息给MCU。
        Parameters
        ----------
        msg: `Gimbal`
            A mode message is received
        """

        # pitch = math.atan2(msg.position.z, math.sqrt(msg.position.x**2 + msg.position.y**2))
        # yaw = math.atan2(msg.position.y, msg.position.x)    
            
        mode = 1 # 确认这里：如果 msg.yaw 是相对误差，用 1；如果是绝对角度，用 0 (且需要协议支持)

        if msg.fire_advice == False:
            self.fire_advice = 0.0
        else:
            self.fire_advice = 1.0

        # 12.30
        # 修改点：移除 -10，并转换弧度为角度
        # 假设 msg.yaw 和 msg.pitch 是弧度制
        send_yaw = msg.yaw
        send_pitch = msg.pitch
        dist = msg.distance

        # 0311 开辟数据位用于传输是否检测到裝甲板
        if dist > 0:
            has_target = 1
        else:
            has_target = 0

        self.robot_serial.send_data(
            "gimbal",
            [mode, send_yaw, send_pitch, self.fire_advice, has_target, 0, 0])
        # self.get_logger().info(f"Y:{send_yaw:.2f}, P:{send_pitch:.2f}, Dist:{msg.distance:.2f}, Target:{has_target}")

    # 0120 优化IMU数据自收发（回退）
    # transform odom info
    def getImu_callback(self, msg: SerialReceiveData) -> None:
        self.get_yaw = msg.yaw
        self.get_pitch = msg.pitch - 1.0
        self.get_roll = msg.roll

        # # 判断是否需要重启串口
        # current_time = time.time()
        # current_yaw = self.get_yaw
        # current_pitch = self.get_pitch

        # if current_yaw == self.last_yaw and current_pitch == self.last_pitch:
        #     time_diff = current_time - self.last_timestamp
        #     if time_diff >= 1.0:
        #         self.get_logger().warn("IMU data unchanged for 1s. Restarting serial port...")
        #         # try:
        #         #     # 使用系统命令安全重启串口（需配置免密 sudo）
        #         #     subprocess.run(
        #         #         ["sudo", "sh", "-c", "echo 0 > /sys/bus/usb/devices/usb3/3-1/authorized"],  # 示例：禁用设备
        #         #         check=True
        #         #     )
        #         #     time.sleep(2)  # 等待设备重新枚举
        #         #     subprocess.run(
        #         #         ["sudo", "sh", "-c", "echo 1 > /sys/bus/usb/devices/usb3/3-1/authorized"],  # 示例：启用设备
        #         #         check=True
        #         #     )
        #         #     time.sleep(2)
        #         #     # 重新初始化串口（无需手动 open，依赖硬件自动连接）
        #         #     self.robot_serial.init_device(
        #         #         port="/dev/ttyrobomaster",
        #         #         baudrate=self.robot_serial.baudrate,
        #         #         timeout_T=0
        #         #     )
                    
        #         #     self.last_timestamp = time.time()
        #         #     self.get_logger().info("Serial port restarted via hardware reset.")
                    
        #         # except subprocess.CalledProcessError as e:
        #         #     self.get_logger().error(f"Hardware reset failed: {e}")
        #         # except Exception as e:
        #         #     self.get_logger().error(f"Serial init error: {e}")
        # else:
        #     # 更新最新数据和时间戳
        #     self.last_yaw = current_yaw
        #     self.last_pitch = current_pitch
        #     self.last_timestamp = current_time
        # #print(f"received pub info {self.get_yaw}")

    def broadcast_transform(self):
        def euler_to_quaternion(roll, pitch, yaw):
            qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
            qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
            qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
            qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
            return Quaternion(x=qx, y=qy, z=qz, w=qw)
        
        # 0120 优化自收发（回退）
        # 将角度转换为弧度
        roll_rad = self.get_roll * pi / 180.0
        pitch_rad = -self.get_pitch * pi / 180.0
        yaw_rad = self.get_yaw * pi / 180.0
        # roll_rad = self.robot_serial.imu_roll * pi / 180.0
        # pitch_rad = -self.robot_serial.imu_pitch * pi / 180.0
        # yaw_rad = self.robot_serial.imu_yaw * pi / 180.0

        # 使用自定义的函数生成四元数
        q = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        # 创建 TransformStamped 消息
        transform_stamped = geometry_msgs.msg.TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "gimbal_odom"  # 父坐标系
        transform_stamped.child_frame_id = "gimbal_link"  # 子坐标系

        # 填充变换数据
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = q.x
        transform_stamped.transform.rotation.y = q.y
        transform_stamped.transform.rotation.z = q.z
        transform_stamped.transform.rotation.w = q.w

        # 发布第一个变换
        self.tf_broadcaster.sendTransform(transform_stamped)

        # 修正四元数：保留 roll，忽略 pitch 和 yaw
        q_fixed = euler_to_quaternion(roll_rad, 0.0, 0.0)

        # # 设置第二个变换：odom -> odom_rectify
        # transform_stamped.header.stamp = self.get_clock().now().to_msg()
        # transform_stamped.header.frame_id = "odom"  # 父坐标系
        # transform_stamped.child_frame_id = "odom_rectify"  # 子坐标系

        # # 填充修正后的变换数据
        # transform_stamped.transform.rotation.x = q_fixed.x
        # transform_stamped.transform.rotation.y = q_fixed.y
        # transform_stamped.transform.rotation.z = q_fixed.z
        # transform_stamped.transform.rotation.w = q_fixed.w

        # # 发布第二个变换
        # self.tf_broadcaster.sendTransform(transform_stamped)

    def barrel_callback(self, msg: Shooter) -> None:
        """Shooter function, send enable shooter infomation to MCU.
            射击功能，发送使能射击信息给MCU。
        Parameters
        ----------
        msg: `Shooter`
            A mode message is received
        """
        # self.get_logger().info("recived data barrel change, barrel: {}".format(msg.data))
        self.robot_serial.send_data(
            "barrel", [msg.is_shoot, msg.bullet_vel, msg.remain_bullet])

    def init_robot(self):
        """Expanded api definition, API required by the special robot is initialized.
            扩展api定义，初始化特殊机器人所需的API。
        Parameters
        ----------
        name: `str`
            robot name, name option reference 
            `The Chinese-English comparison table <https://birdiebot.github.io/bubble_documentation/guide/%E6%9C%AF%E8%AF%AD%E4%B8%AD%E8%8B%B1%E6%96%87%E5%AF%B9%E7%85%A7%E8%A1%A8.html>`__ .
        """
        if self.name == "sentry":
            pass
        elif self.name == "infantry":
            pass

    def ex_chassis_callback(self, msg: Twist) -> None:
        """Chassis function, send chassis infomation to MCU.
            底盘功能，发送底盘信息给MCU
        Parameters
        ----------
        msg: `Chassis`
            A chassis message is received
        """
        # print("recived data chassis")
        # self.robot_serial.send_data("chassis_ctrl", [msg.linear.x, msg.linear.y, msg.linear.z,
        #                                              msg.angular.x, msg.angular.y, msg.angular.z])
        if msg.linear.x == 0 and msg.linear.y == 0 and msg.angular.z == 0:
            self.robot_serial.send_data("chassis_ctrl", [0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        
        else:
            # 正常行驶，第三位发 msg.linear.z (通常为0)
            self.robot_serial.send_data("chassis_ctrl", [
                msg.linear.y, 
                -msg.linear.x, 
                msg.linear.z,
                msg.angular.x, 
                msg.angular.y, 
                msg.angular.z
                    ])

    def ex_odom_callback(self, msg: PoseWithCovariance):
        """Odom function, send chassis infomation to MCU.
            Odom函数，发送底盘信息给MCU。
        Parameters
        ----------
        msg: `PoseWithCovariance`
            A chassis message is received
        """
        odom_list = []
        odom_list.append(msg.pose.position.x)
        odom_list.append(msg.pose.position.y)
        odom_list.append(msg.pose.position.z)
        odom_list.append(msg.pose.orientation.x)
        odom_list.append(msg.pose.orientation.y)
        odom_list.append(msg.pose.orientation.z)
        odom_list.append(msg.pose.orientation.w)
        self.robot_serial.send_data("odom", odom_list)
