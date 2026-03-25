#!/usr/bin/env python3
"""
智能脱困功能测试脚本

使用方法:
  ros2 run smart_escape test_smart_escape.py
  
或:
  python3 test_smart_escape.py
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import SmartEscape
import sys


class SmartEscapeTester(Node):
    def __init__(self):
        super().__init__('smart_escape_tester')
        self._action_client = ActionClient(self, SmartEscape, 'smart_escape')
        self.get_logger().info('Smart Escape Tester initialized')

    def send_goal(self):
        """发送脱困请求"""
        self.get_logger().info('Waiting for action server...')
        
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return False
            
        self.get_logger().info('Sending smart escape goal...')
        
        goal_msg = SmartEscape.Goal()
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """Goal响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """结果回调"""
        result = future.result().result
        
        if result.success:
            self.get_logger().info(f'Smart escape succeeded: {result.message}')
            self.get_logger().info(f'Escape pose:')
            self.get_logger().info(f'  Position: [{result.escape_pose.pose.position.x:.3f}, '
                                   f'{result.escape_pose.pose.position.y:.3f}, '
                                   f'{result.escape_pose.pose.position.z:.3f}]')
            self.get_logger().info(f'  Orientation: [{result.escape_pose.pose.orientation.x:.3f}, '
                                   f'{result.escape_pose.pose.orientation.y:.3f}, '
                                   f'{result.escape_pose.pose.orientation.z:.3f}, '
                                   f'{result.escape_pose.pose.orientation.w:.3f}]')
            self.get_logger().info(f'  Frame: {result.escape_pose.header.frame_id}')
        else:
            self.get_logger().error(f'Smart escape failed: {result.message}')

    def feedback_callback(self, feedback_msg):
        """反馈回调"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.status} ({feedback.progress*100:.1f}%)')


def main(args=None):
    rclpy.init(args=args)
    
    tester = SmartEscapeTester()
    
    if tester.send_goal():
        try:
            rclpy.spin(tester)
        except KeyboardInterrupt:
            tester.get_logger().info('Test interrupted')
    else:
        tester.get_logger().error('Failed to send goal')
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
