'''
Copyright (c) 2025 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: AI Assistant
Date: 2025-11-09
FilePath: /bubble/src/rm_communication/bubble_decision/bubble_decision/slip_integration.py
Description: 打滑检测集成模块 - 在现有决策系统中使用打滑检测
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from rmctrl_msgs.msg import Chassis
import time

class SlipResponse(Node):
    """打滑响应模块 - 根据打滑状态调整机器人行为"""
    
    def __init__(self):
        super().__init__('slip_response')
        
        # 订阅打滑状态
        self.slip_status_sub = self.create_subscription(
            Bool, '/slip_status', self.slip_status_callback, 10)
        self.slip_ratio_sub = self.create_subscription(
            Float32, '/slip_ratio', self.slip_ratio_callback, 10)
        self.slip_severity_sub = self.create_subscription(
            Float32, '/slip_severity', self.slip_severity_callback, 10)
        
        # 发布调整后的底盘控制
        self.chassis_pub = self.create_publisher(
            Chassis, '/chassis_adjusted', 10)
        
        # 状态变量
        self.is_slipping = False
        self.slip_ratio = 0.0
        self.slip_severity = 0.0
        self.last_slip_time = 0.0
        
        # 参数
        self.declare_parameter('recovery_time', 2.0)  # 打滑恢复时间 (秒)
        self.declare_parameter('speed_reduction_factor', 0.5)  # 打滑时速度降低因子
        
        self.recovery_time = self.get_parameter('recovery_time').value
        self.speed_reduction_factor = self.get_parameter('speed_reduction_factor').value
        
        self.get_logger().info('Slip response module initialized')
    
    def slip_status_callback(self, msg):
        """处理打滑状态消息"""
        self.is_slipping = msg.data
        if self.is_slipping:
            self.last_slip_time = time.time()
            self.get_logger().warning(f'Slip detected! Ratio: {self.slip_ratio:.3f}, Severity: {self.slip_severity:.3f}')
    
    def slip_ratio_callback(self, msg):
        """处理打滑率消息"""
        self.slip_ratio = msg.data
    
    def slip_severity_callback(self, msg):
        """处理打滑严重程度消息"""
        self.slip_severity = msg.data
    
    def adjust_chassis_command(self, chassis_msg):
        """根据打滑状态调整底盘控制命令"""
        adjusted_msg = Chassis()
        
        # 复制原始消息
        adjusted_msg.chassis_target_linear_x = chassis_msg.chassis_target_linear_x
        adjusted_msg.chassis_target_linear_y = chassis_msg.chassis_target_linear_y
        adjusted_msg.chassis_target_linear_z = chassis_msg.chassis_target_linear_z
        adjusted_msg.chassis_target_angular_x = chassis_msg.chassis_target_angular_x
        adjusted_msg.chassis_target_angular_y = chassis_msg.chassis_target_angular_y
        adjusted_msg.chassis_target_angular_z = chassis_msg.chassis_target_angular_z
        
        # 如果正在打滑，降低速度
        if self.is_slipping:
            reduction_factor = 1.0 - (self.slip_severity * self.speed_reduction_factor)
            
            adjusted_msg.chassis_target_linear_x *= reduction_factor
            adjusted_msg.chassis_target_linear_y *= reduction_factor
            adjusted_msg.chassis_target_angular_z *= reduction_factor
            
            self.get_logger().info(f'Reducing speed due to slip. Factor: {reduction_factor:.3f}')
        
        # 如果在恢复期内，仍然稍微降低速度
        elif time.time() - self.last_slip_time < self.recovery_time:
            time_since_slip = time.time() - self.last_slip_time
            recovery_factor = min(1.0, time_since_slip / self.recovery_time)
            
            adjusted_msg.chassis_target_linear_x *= recovery_factor
            adjusted_msg.chassis_target_linear_y *= recovery_factor
            adjusted_msg.chassis_target_angular_z *= recovery_factor
            
            self.get_logger().info(f'Gradually recovering speed. Factor: {recovery_factor:.3f}')
        
        return adjusted_msg


class SlipAwareGameAction:
    """具有打滑感知的游戏动作类 - 可以集成到现有的GameAction中"""
    
    def __init__(self, node, game_action):
        self.node = node
        self.game_action = game_action
        
        # 打滑状态
        self.is_slipping = False
        self.slip_severity = 0.0
        
        # 订阅打滑状态
        self.slip_status_sub = self.node.create_subscription(
            Bool, '/slip_status', self.slip_status_callback, 10)
        self.slip_severity_sub = self.node.create_subscription(
            Float32, '/slip_severity', self.slip_severity_callback, 10)
        
        self.node.get_logger().info('Slip-aware game action initialized')
    
    def slip_status_callback(self, msg):
        self.is_slipping = msg.data
    
    def slip_severity_callback(self, msg):
        self.slip_severity = msg.data
    
    def should_avoid_movement(self):
        """判断是否应该避免移动（严重打滑时）"""
        return self.is_slipping and self.slip_severity > 0.7
    
    def get_movement_priority(self):
        """获取移动优先级（打滑时降低优先级）"""
        if self.is_slipping:
            return max(0.1, 1.0 - self.slip_severity)
        return 1.0
    
    def get_safe_speed_factor(self):
        """获取安全速度因子"""
        if self.is_slipping:
            return max(0.3, 1.0 - self.slip_severity)
        return 1.0


def main(args=None):
    """示例：运行打滑响应模块"""
    rclpy.init(args=args)
    slip_response = SlipResponse()
    
    try:
        rclpy.spin(slip_response)
    except KeyboardInterrupt:
        pass
    finally:
        slip_response.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
