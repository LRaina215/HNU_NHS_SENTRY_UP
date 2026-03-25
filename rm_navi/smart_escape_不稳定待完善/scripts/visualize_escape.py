#!/usr/bin/env python3
"""
智能脱困可视化节点

在RViz中显示：
- 机器人当前位置
- 搜索半径范围
- 无障碍区域
- 计算的脱困目标点

使用方法:
  ros2 run smart_escape visualize_escape.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import SmartEscape
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
import colorsys


class SmartEscapeVisualizer(Node):
    def __init__(self):
        super().__init__('smart_escape_visualizer')
        
        # 参数
        self.declare_parameter('marker_lifetime', 5.0)
        self.declare_parameter('show_search_area', True)
        self.declare_parameter('show_free_space', True)
        
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        self.show_search_area = self.get_parameter('show_search_area').value
        self.show_free_space = self.get_parameter('show_free_space').value
        
        # 订阅代价地图
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10)
        
        # 发布可视化标记
        self.marker_pub = self.create_publisher(MarkerArray, '/smart_escape/markers', 10)
        
        # Action客户端
        self.action_client = ActionClient(self, SmartEscape, 'smart_escape')
        
        # 定时器：定期发送脱困请求并更新可视化
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        self.current_costmap = None
        self.escape_pose = None
        
        self.get_logger().info('Smart Escape Visualizer started')
        self.get_logger().info('Run RViz and add MarkerArray topic: /smart_escape/markers')

    def costmap_callback(self, msg):
        """代价地图回调"""
        self.current_costmap = msg

    def timer_callback(self):
        """定时器回调：发送脱困请求"""
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('Smart escape server not available')
            return
        
        goal_msg = SmartEscape.Goal()
        
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Goal响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """结果回调"""
        result = future.result().result
        
        if result.success:
            self.escape_pose = result.escape_pose
            self.publish_markers()

    def publish_markers(self):
        """发布可视化标记"""
        if not self.escape_pose:
            return
        
        marker_array = MarkerArray()
        
        # 1. 脱困目标点（红色球体）
        escape_marker = Marker()
        escape_marker.header = self.escape_pose.header
        escape_marker.ns = 'smart_escape'
        escape_marker.id = 0
        escape_marker.type = Marker.SPHERE
        escape_marker.action = Marker.ADD
        escape_marker.pose = self.escape_pose.pose
        escape_marker.scale.x = 0.2
        escape_marker.scale.y = 0.2
        escape_marker.scale.z = 0.2
        escape_marker.color.r = 1.0
        escape_marker.color.g = 0.0
        escape_marker.color.b = 0.0
        escape_marker.color.a = 1.0
        escape_marker.lifetime.sec = int(self.marker_lifetime)
        marker_array.markers.append(escape_marker)
        
        # 2. 从机器人指向目标点的箭头（绿色）
        arrow_marker = Marker()
        arrow_marker.header = self.escape_pose.header
        arrow_marker.ns = 'smart_escape'
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        
        # 起点：假设机器人在(0,0)或从TF获取
        arrow_marker.points.append(Point(x=0.0, y=0.0, z=0.1))
        arrow_marker.points.append(self.escape_pose.pose.position)
        
        arrow_marker.scale.x = 0.05  # 箭杆直径
        arrow_marker.scale.y = 0.1   # 箭头直径
        arrow_marker.scale.z = 0.1   # 箭头长度
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0
        arrow_marker.lifetime.sec = int(self.marker_lifetime)
        marker_array.markers.append(arrow_marker)
        
        # 3. 搜索区域（半透明圆柱）
        if self.show_search_area:
            search_marker = Marker()
            search_marker.header = self.escape_pose.header
            search_marker.ns = 'smart_escape'
            search_marker.id = 2
            search_marker.type = Marker.CYLINDER
            search_marker.action = Marker.ADD
            search_marker.pose.position.x = 0.0
            search_marker.pose.position.y = 0.0
            search_marker.pose.position.z = 0.0
            search_marker.pose.orientation.w = 1.0
            search_marker.scale.x = 1.0  # 直径
            search_marker.scale.y = 1.0
            search_marker.scale.z = 0.01
            search_marker.color.r = 0.0
            search_marker.color.g = 0.5
            search_marker.color.b = 1.0
            search_marker.color.a = 0.3
            search_marker.lifetime.sec = int(self.marker_lifetime)
            marker_array.markers.append(search_marker)
        
        # 4. 文本标签
        text_marker = Marker()
        text_marker.header = self.escape_pose.header
        text_marker.ns = 'smart_escape'
        text_marker.id = 3
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = self.escape_pose.pose.position
        text_marker.pose.position.z += 0.3
        text_marker.scale.z = 0.1  # 字体大小
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = f"Escape Target\n({self.escape_pose.pose.position.x:.2f}, {self.escape_pose.pose.position.y:.2f})"
        text_marker.lifetime.sec = int(self.marker_lifetime)
        marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} markers')


def main(args=None):
    rclpy.init(args=args)
    visualizer = SmartEscapeVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
