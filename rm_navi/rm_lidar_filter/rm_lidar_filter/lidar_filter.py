#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')

        self.input_topic = '/livox/lidar'
        # 【改名】输出话题改为 "无车身点云"
        self.output_topic = '/livox/lidar_no_body' 
        
        # 过滤半径 (米) - 只要小于这个半径的点全删掉
        self.body_radius = 0.4 

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.listener_callback, 10)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.get_logger().info(f'车身过滤器启动! 输出话题: {self.output_topic}')

    def listener_callback(self, msg):
        points = pc2.read_points_list(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        if not points: return
        
        np_points = np.array(points, dtype=np.float32)
        x = np_points[:, 0]
        y = np_points[:, 1]
        z = np_points[:, 2] # 依然读取z，但不做过滤，只用来发布
        
        # === 核心逻辑：只扣掉车身圆柱 ===
        dist_sq = x**2 + y**2
        mask_remove = (dist_sq < self.body_radius**2)
        
        np_filtered = np_points[~mask_remove]

        # 重新封装 (保留4个字段)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        filtered_msg = pc2.create_cloud(msg.header, fields, np_filtered)
        self.pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()