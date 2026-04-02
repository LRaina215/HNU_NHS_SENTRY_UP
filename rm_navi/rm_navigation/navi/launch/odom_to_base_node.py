#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

def q_inverse(q):
    return [-q[0], -q[1], -q[2], q[3]]

def q_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ]

def qv_mult(q1, v1):
    q2 = [v1[0], v1[1], v1[2], 0.0]
    return q_mult(q_mult(q1, q2), q_inverse(q1))[0:3]

class OdomToBaseTF(Node):
    def __init__(self):
        super().__init__('odom_to_base_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.cached_tf = None  
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # 发布专供 Nav2 控制器读取的“底盘真实速度”
        self.odom_pub = self.create_publisher(Odometry, '/odom_base', 10)

    def odom_callback(self, msg):
        if self.cached_tf is None:
            try:
                self.cached_tf = self.tf_buffer.lookup_transform(
                    'livox_frame', 'base_link', rclpy.time.Time())
                self.get_logger().info("✅ 成功缓存 URDF 外参，开始同步 TF 与 底盘速度！")
            except Exception as e:
                return

        # 1. 计算 TF 位置
        pos_ol = msg.pose.pose.position
        q_ol = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        t_ol = [pos_ol.x, pos_ol.y, pos_ol.z]

        pos_lb = self.cached_tf.transform.translation
        q_lb = [self.cached_tf.transform.rotation.x, self.cached_tf.transform.rotation.y, 
                self.cached_tf.transform.rotation.z, self.cached_tf.transform.rotation.w]
        t_lb = [pos_lb.x, pos_lb.y, pos_lb.z]

        q_ob = q_mult(q_ol, q_lb)
        t_rotated = qv_mult(q_ol, t_lb)
        t_ob = [t_ol[0] + t_rotated[0], t_ol[1] + t_rotated[1], t_ol[2] + t_rotated[2]]

        # 2. 发布同步系统时间的 TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = t_ob[0]
        t.transform.translation.y = t_ob[1]
        t.transform.translation.z = t_ob[2]
        t.transform.rotation.x = q_ob[0]
        t.transform.rotation.y = q_ob[1]
        t.transform.rotation.z = q_ob[2]
        t.transform.rotation.w = q_ob[3]
        self.tf_broadcaster.sendTransform(t)

        # 3. 用逆矩阵把雷达的速度旋转回底盘！
        v_l = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        w_l = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]

        q_bl = q_inverse(q_lb) 
        v_b = qv_mult(q_bl, v_l) 
        w_b = qv_mult(q_bl, w_l) 

        # 4. 组装并发布纯净的底盘 Odometry
        odom_base = Odometry()
        odom_base.header.stamp = t.header.stamp
        odom_base.header.frame_id = 'odom'
        odom_base.child_frame_id = 'base_link'
        
        odom_base.pose.pose.position.x = t_ob[0]
        odom_base.pose.pose.position.y = t_ob[1]
        odom_base.pose.pose.position.z = t_ob[2]
        odom_base.pose.pose.orientation.x = q_ob[0]
        odom_base.pose.pose.orientation.y = q_ob[1]
        odom_base.pose.pose.orientation.z = q_ob[2]
        odom_base.pose.pose.orientation.w = q_ob[3]
        
        odom_base.twist.twist.linear.x = v_b[0]
        odom_base.twist.twist.linear.y = v_b[1]
        odom_base.twist.twist.linear.z = v_b[2]
        odom_base.twist.twist.angular.x = w_b[0]
        odom_base.twist.twist.angular.y = w_b[1]
        odom_base.twist.twist.angular.z = w_b[2]

        self.odom_pub.publish(odom_base)

# 👇 就是漏了下面这几行救命的启动代码 👇
def main(args=None):
    rclpy.init(args=args)
    node = OdomToBaseTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()