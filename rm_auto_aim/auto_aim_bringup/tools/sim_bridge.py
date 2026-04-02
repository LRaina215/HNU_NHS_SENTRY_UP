#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rm_interfaces.msg import GimbalCmd 
import math

class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge_node')
        
        # 1. 订阅与发布
        self.sub = self.create_subscription(
            GimbalCmd, '/armor_solver/cmd_gimbal', self.callback, 10)
        self.pub = self.create_publisher(
            Twist, '/cmd_gimbal', 10)
        
        # ==========================================
        # 🛠️ [校准区域] (保留了你的参数)
        # ==========================================
        self.MANUAL_PITCH_OFFSET = 0
        self.MANUAL_YAW_OFFSET = 0
        # ==========================================

        # --- [PD 参数] (保留了你的参数) ---
        self.kp_yaw = 5.0    
        self.kd_yaw = 0.2    
        
        self.kp_pitch = 4.0  # 你修改后的值
        self.kd_pitch = 0.2

        # --- [状态变量] ---
        self.last_yaw_err = 0.0
        self.last_pitch_err = 0.0
        
        # [新增] 目标记忆与看门狗
        self.target_yaw_rad = 0.0
        self.target_pitch_rad = 0.0
        self.last_msg_time = self.get_clock().now() # 上次收到消息的时间

        # 创建高频控制定时器 (100Hz)
        # 将控制逻辑从 callback 移到了这里，以实现超时归零
        self.create_timer(0.01, self.control_loop)

    def callback(self, msg):
        # 1. 收到消息，刷新“看门狗”时间
        self.last_msg_time = self.get_clock().now()

        # 2. 更新目标值 (叠加手动补偿)
        # 注意：这里我们只更新“目标”，不直接发指令。指令由 control_loop 统一发。
        target_yaw_deg = msg.yaw_diff + self.MANUAL_YAW_OFFSET
        target_pitch_deg = msg.pitch_diff + self.MANUAL_PITCH_OFFSET

        # 转弧度存储
        self.target_yaw_rad = math.radians(target_yaw_deg)
        self.target_pitch_rad = math.radians(target_pitch_deg)

    def control_loop(self):
        # 1. 检查超时 (看门狗逻辑)
        # 如果超过 0.5 秒没收到新指令，说明自瞄丢了或者模拟器刚重启
        current_time = self.get_clock().now()
        time_since_last_msg = (current_time - self.last_msg_time).nanoseconds / 1e9

        if time_since_last_msg > 0.5:
            # === 超时归零逻辑 ===
            self.target_yaw_rad = 0.0
            self.target_pitch_rad = 0.0
            # 清除微分历史，防止“刹车”过猛
            self.last_yaw_err = 0.0
            self.last_pitch_err = 0.0
            
            # (可选) 如果你想完全断电而不是回正，可以用下面这就话代替：
            # self.pub.publish(Twist()) 
            # return

        # --- 2. PD 控制逻辑 ---
        
        dt = 0.01 # 定时器周期固定为 0.01s

        # 这里的 error 本身就是目标值 
        # (因为模拟器是增量式/速度式控制，且目标已经是 diff)
        yaw_err = self.target_yaw_rad
        pitch_err = self.target_pitch_rad

        # 计算微分 (D项)
        d_yaw = (yaw_err - self.last_yaw_err) / dt
        d_pitch = (pitch_err - self.last_pitch_err) / dt

        twist = Twist()
        direction = 1.0 
        
        # PD 公式
        twist.angular.z = direction * (self.kp_yaw * yaw_err + self.kd_yaw * d_yaw)
        twist.angular.y = direction * (self.kp_pitch * pitch_err + self.kd_pitch * d_pitch)
        
        # 更新历史
        self.last_yaw_err = yaw_err
        self.last_pitch_err = pitch_err
        
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()