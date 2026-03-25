import rclpy
from rclpy.node import Node
import math
import time

# 自动适配你本地的消息包
try:
    from auto_aim_interfaces.msg import Target
except ImportError:
    from rm_interfaces.msg import Target

class MockTargetNode(Node):
    def __init__(self):
        super().__init__('mock_target_node')
        self.pub = self.create_publisher(Target, '/armor_solver/target', 10)
        self.timer = self.create_timer(0.1, self.timer_cb) # 10Hz 发送
        self.start_time = time.time()
        self.get_logger().info("假装甲板已上线！正在生成动态移动的目标...")

    def timer_cb(self):
        msg = Target()
        msg.tracking = True
        
        # 用正弦函数模拟敌人来回移动：距离在 1.0m 到 5.0m 之间变化
        elapsed = time.time() - self.start_time
        distance = 3.0 + 2.0 * math.sin(elapsed * 0.5) 
        
        # 假设敌人在正前方 (x方向)
        msg.position.x = distance
        msg.position.y = 0.0
        msg.position.z = 0.0
        
        self.pub.publish(msg)
        self.get_logger().info(f"发送假目标: 距离 = {distance:.2f} m")

def main():
    rclpy.init()
    node = MockTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()