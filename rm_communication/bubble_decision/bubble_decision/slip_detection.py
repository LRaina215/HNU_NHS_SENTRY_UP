'''
Copyright (c) 2025 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: AI Assistant
Date: 2025-11-09
FilePath: /bubble/src/rm_communication/bubble_decision/bubble_decision/slip_detection.py
Description: 打滑检测模块
'''

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rmctrl_msgs.msg import Chassis, Odom
from std_msgs.msg import Bool, Float32
import math

class SlipDetection(Node):
    def __init__(self):
        super().__init__('slip_detection')
        
        # 参数声明
        self.declare_parameter('slip_threshold', 0.3)  # 打滑阈值 (0-1之间)
        self.declare_parameter('window_size', 10)      # 滑动窗口大小
        self.declare_parameter('min_speed_threshold', 0.1)  # 最小速度阈值 (m/s)
        
        # 获取参数
        self.slip_threshold = self.get_parameter('slip_threshold').value
        self.window_size = self.get_parameter('window_size').value
        self.min_speed_threshold = self.get_parameter('min_speed_threshold').value
        
        # 状态变量
        self.target_velocities = []  # 目标速度历史
        self.actual_velocities = []  # 实际速度历史
        self.slip_ratios = []        # 打滑率历史
        self.is_slipping = False     # 当前打滑状态
        self.slip_severity = 0.0     # 打滑严重程度 (0-1)
        
        # 订阅器
        self.chassis_sub = self.create_subscription(
            Chassis, '/chassis', self.chassis_callback, 10)
        self.odom_sub = self.create_subscription(
            Odom, '/odom', self.odom_callback, 10)
        
        # 发布器
        self.slip_status_pub = self.create_publisher(
            Bool, '/slip_status', 10)
        self.slip_ratio_pub = self.create_publisher(
            Float32, '/slip_ratio', 10)
        self.slip_severity_pub = self.create_publisher(
            Float32, '/slip_severity', 10)
        
        # 定时器用于定期发布状态
        self.timer = self.create_timer(0.1, self.publish_slip_status)  # 10Hz
        
        self.get_logger().info('Slip detection node initialized')
    
    def chassis_callback(self, msg):
        """处理底盘目标速度消息"""
        # 计算目标线速度大小
        target_linear_speed = math.sqrt(
            msg.chassis_target_linear_x ** 2 + 
            msg.chassis_target_linear_y ** 2
        )
        
        # 添加到历史记录
        self.target_velocities.append(target_linear_speed)
        if len(self.target_velocities) > self.window_size:
            self.target_velocities.pop(0)
    
    def odom_callback(self, msg):
        """处理里程计消息"""
        # 计算实际线速度大小（通过位置差分）
        if len(self.actual_velocities) > 0:
            # 这里简化处理，实际应用中应该使用更精确的速度计算
            # 例如通过IMU数据或编码器数据
            actual_speed = self.estimate_actual_speed(msg)
            self.actual_velocities.append(actual_speed)
        else:
            self.actual_velocities.append(0.0)
        
        if len(self.actual_velocities) > self.window_size:
            self.actual_velocities.pop(0)
        
        # 计算打滑率
        self.calculate_slip_ratio()
    
    def estimate_actual_speed(self, odom_msg):
        """估计实际速度（简化版本）"""
        # 在实际应用中，这里应该使用更精确的速度估计方法
        # 例如通过IMU、编码器或视觉里程计数据
        if len(self.actual_velocities) < 2:
            return 0.0
        
        # 使用最近的速度估计（简化处理）
        return self.actual_velocities[-1]
    
    def calculate_slip_ratio(self):
        """计算打滑率"""
        if len(self.target_velocities) < 2 or len(self.actual_velocities) < 2:
            return
        
        # 使用滑动窗口的平均值
        avg_target_speed = np.mean(self.target_velocities)
        avg_actual_speed = np.mean(self.actual_velocities)
        
        # 检查是否在运动
        if avg_target_speed < self.min_speed_threshold:
            self.slip_ratio = 0.0
            self.is_slipping = False
            self.slip_severity = 0.0
            return
        
        # 计算打滑率
        if avg_target_speed > 0:
            slip_ratio = max(0, 1 - (avg_actual_speed / avg_target_speed))
        else:
            slip_ratio = 0.0
        
        self.slip_ratio = slip_ratio
        self.slip_ratios.append(slip_ratio)
        
        if len(self.slip_ratios) > self.window_size:
            self.slip_ratios.pop(0)
        
        # 判断是否打滑
        self.is_slipping = slip_ratio > self.slip_threshold
        
        # 计算打滑严重程度
        if self.is_slipping:
            self.slip_severity = min(1.0, slip_ratio / self.slip_threshold)
        else:
            self.slip_severity = 0.0
    
    def publish_slip_status(self):
        """发布打滑状态"""
        # 发布打滑状态
        slip_status_msg = Bool()
        slip_status_msg.data = self.is_slipping
        self.slip_status_pub.publish(slip_status_msg)
        
        # 发布打滑率
        slip_ratio_msg = Float32()
        slip_ratio_msg.data = float(self.slip_ratio)
        self.slip_ratio_pub.publish(slip_ratio_msg)
        
        # 发布打滑严重程度
        slip_severity_msg = Float32()
        slip_severity_msg.data = float(self.slip_severity)
        self.slip_severity_pub.publish(slip_severity_msg)
    
    def get_slip_info(self):
        """获取打滑信息"""
        return {
            'is_slipping': self.is_slipping,
            'slip_ratio': self.slip_ratio,
            'slip_severity': self.slip_severity,
            'target_speed': np.mean(self.target_velocities) if self.target_velocities else 0.0,
            'actual_speed': np.mean(self.actual_velocities) if self.actual_velocities else 0.0
        }


class AdvancedSlipDetection(SlipDetection):
    """高级打滑检测，包含更多传感器融合"""
    
    def __init__(self):
        super().__init__()
        
        # 额外参数
        self.declare_parameter('acceleration_threshold', 2.0)  # 加速度阈值 (m/s²)
        self.declare_parameter('yaw_rate_threshold', 1.0)     # 偏航率阈值 (rad/s)
        
        self.acceleration_threshold = self.get_parameter('acceleration_threshold').value
        self.yaw_rate_threshold = self.get_parameter('yaw_rate_threshold').value
        
        # 额外状态
        self.acceleration_history = []
        self.yaw_rate_history = []
        
        self.get_logger().info('Advanced slip detection node initialized')
    
    def calculate_slip_ratio(self):
        """增强的打滑率计算，包含更多传感器数据"""
        super().calculate_slip_ratio()
        
        # 这里可以添加更多传感器融合逻辑
        # 例如使用IMU数据检测异常加速度
        # 或使用视觉数据检测地面条件
        
        # 如果检测到异常加速度，增加打滑严重程度
        if self.detect_abnormal_acceleration():
            self.slip_severity = min(1.0, self.slip_severity + 0.2)
    
    def detect_abnormal_acceleration(self):
        """检测异常加速度"""
        # 在实际应用中，这里应该使用IMU数据
        # 简化版本：如果目标速度变化但实际速度不变，可能打滑
        if len(self.target_velocities) < 3:
            return False
        
        target_accel = abs(self.target_velocities[-1] - self.target_velocities[-2])
        actual_accel = abs(self.actual_velocities[-1] - self.actual_velocities[-2])
        
        return target_accel > self.acceleration_threshold and actual_accel < 0.5


def main(args=None):
    rclpy.init(args=args)
    slip_detection = SlipDetection()
    
    try:
        rclpy.spin(slip_detection)
    except KeyboardInterrupt:
        pass
    finally:
        slip_detection.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
