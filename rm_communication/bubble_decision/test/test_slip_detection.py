'''
Copyright (c) 2025 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: AI Assistant
Date: 2025-11-09
FilePath: /bubble/src/rm_communication/bubble_decision/test/test_slip_detection.py
Description: 打滑检测测试脚本
'''

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Bool, Float32
from rmctrl_msgs.msg import Chassis, Odom

class SlipDetectionTester(Node):
    """打滑检测测试节点 - 模拟底盘控制和里程计数据"""
    
    def __init__(self):
        super().__init__('slip_detection_tester')
        
        # 发布模拟数据
        self.chassis_pub = self.create_publisher(Chassis, '/chassis', 10)
        self.odom_pub = self.create_publisher(Odom, '/odom', 10)
        
        # 订阅打滑检测结果
        self.slip_status_sub = self.create_subscription(
            Bool, '/slip_status', self.slip_status_callback, 10)
        self.slip_ratio_sub = self.create_subscription(
            Float32, '/slip_ratio', self.slip_ratio_callback, 10)
        
        # 测试状态
        self.test_scenario = 0
        self.slip_detected = False
        self.current_slip_ratio = 0.0
        
        # 定时器
        self.timer = self.create_timer(0.1, self.run_test)  # 10Hz
        
        self.get_logger().info('Slip detection tester started')
    
    def slip_status_callback(self, msg):
        self.slip_detected = msg.data
    
    def slip_ratio_callback(self, msg):
        self.current_slip_ratio = msg.data
    
    def run_test(self):
        """运行测试场景"""
        if self.test_scenario == 0:
            self.normal_movement()
        elif self.test_scenario == 1:
            self.slight_slip()
        elif self.test_scenario == 2:
            self.severe_slip()
        elif self.test_scenario == 3:
            self.recovery_test()
        else:
            self.get_logger().info('All tests completed')
            self.timer.cancel()
            return
        
        # 每5秒切换测试场景
        if int(time.time()) % 5 == 0:
            self.test_scenario += 1
            self.get_logger().info(f'Switching to test scenario {self.test_scenario}')
    
    def normal_movement(self):
        """正常移动测试 - 无打滑"""
        chassis_msg = Chassis()
        chassis_msg.chassis_target_linear_x = 1.0  # 1 m/s 前进
        chassis_msg.chassis_target_linear_y = 0.0
        chassis_msg.chassis_target_linear_z = 0.0
        self.chassis_pub.publish(chassis_msg)
        
        odom_msg = Odom()
        odom_msg.odom_position_x = time.time() * 1.0  # 模拟实际位置变化
        odom_msg.odom_position_y = 0.0
        odom_msg.odom_position_z = 0.0
        self.odom_pub.publish(odom_msg)
        
        if self.slip_detected:
            self.get_logger().warning(f'False positive slip detected! Ratio: {self.current_slip_ratio:.3f}')
        else:
            self.get_logger().info(f'Normal movement - No slip detected')
    
    def slight_slip(self):
        """轻微打滑测试"""
        chassis_msg = Chassis()
        chassis_msg.chassis_target_linear_x = 1.0  # 目标速度 1 m/s
        chassis_msg.chassis_target_linear_y = 0.0
        chassis_msg.chassis_target_linear_z = 0.0
        self.chassis_pub.publish(chassis_msg)
        
        odom_msg = Odom()
        # 模拟实际速度只有目标速度的70%
        odom_msg.odom_position_x = time.time() * 0.7
        odom_msg.odom_position_y = 0.0
        odom_msg.odom_position_z = 0.0
        self.odom_pub.publish(odom_msg)
        
        if self.slip_detected:
            self.get_logger().info(f'Slight slip correctly detected! Ratio: {self.current_slip_ratio:.3f}')
        else:
            self.get_logger().warning(f'Failed to detect slight slip')
    
    def severe_slip(self):
        """严重打滑测试"""
        chassis_msg = Chassis()
        chassis_msg.chassis_target_linear_x = 1.0  # 目标速度 1 m/s
        chassis_msg.chassis_target_linear_y = 0.0
        chassis_msg.chassis_target_linear_z = 0.0
        self.chassis_pub.publish(chassis_msg)
        
        odom_msg = Odom()
        # 模拟实际速度只有目标速度的20% (严重打滑)
        odom_msg.odom_position_x = time.time() * 0.2
        odom_msg.odom_position_y = 0.0
        odom_msg.odom_position_z = 0.0
        self.odom_pub.publish(odom_msg)
        
        if self.slip_detected:
            self.get_logger().info(f'Severe slip correctly detected! Ratio: {self.current_slip_ratio:.3f}')
        else:
            self.get_logger().warning(f'Failed to detect severe slip')
    
    def recovery_test(self):
        """恢复测试 - 从打滑状态恢复"""
        chassis_msg = Chassis()
        chassis_msg.chassis_target_linear_x = 0.5  # 降低目标速度
        chassis_msg.chassis_target_linear_y = 0.0
        chassis_msg.chassis_target_linear_z = 0.0
        self.chassis_pub.publish(chassis_msg)
        
        odom_msg = Odom()
        # 模拟恢复正常移动
        odom_msg.odom_position_x = time.time() * 0.5
        odom_msg.odom_position_y = 0.0
        odom_msg.odom_position_z = 0.0
        self.odom_pub.publish(odom_msg)
        
        if not self.slip_detected:
            self.get_logger().info(f'Successfully recovered from slip')
        else:
            self.get_logger().warning(f'Still detecting slip during recovery')


def main(args=None):
    rclpy.init(args=args)
    tester = SlipDetectionTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
