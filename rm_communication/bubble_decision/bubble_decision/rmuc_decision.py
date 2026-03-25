# rmuc_decision.py
import rclpy
from rclpy.node import Node
import time
import math
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from game_msgs.msg import RobotHP, GameStatus
from bubble_decision.rmuc_action import RMUCAction

try:
    from auto_aim_interfaces.msg import Target
except ImportError:
    try:
        from rm_interfaces.msg import Target
    except ImportError:
        pass 

class RMUCDecisionNode(Node):
    def __init__(self):
        super().__init__('rmuc_decision_node')
        
        # 声明参数
        self.declare_parameter('robot_type', 'sentry_down')
        self.declare_parameter('team_color', 'red')
        self.declare_parameter('is_test_field', True)
        
        robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        team_color = self.get_parameter('team_color').get_parameter_value().string_value
        is_test_field = self.get_parameter('is_test_field').get_parameter_value().bool_value
        
        self.get_logger().info(f"启动 RMUC 决策节点 | 阵营: {team_color} | 场地模式: {'实验室测试' if is_test_field else 'RMUC赛场'}")
        
        # 实例化 Action，把自身 Node 传进去以挂载发布者
        self.action = RMUCAction(self, team_color, is_test_field)
        
        # ================= 订阅者 =================
        self.hp_sub = self.create_subscription(RobotHP, '/status/robotHP', self.hp_callback, 10)
        self.referee_sub = self.create_subscription(GameStatus, '/referee/game_status', self.referee_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        self.target_sub = self.create_subscription(Target, '/tracker/target', self.target_callback, 10)
        
        # 订阅底层的真实姿态反馈
        self.actual_posture_sub = self.create_subscription(
            Int8, '/status/sentry_posture_feedback', self.actual_posture_callback, 10)
            
        # 核心决策循环 (10Hz)
        self.timer = self.create_timer(0.1, self.action.tick)

    # ================= 接收数据并写入黑板 =================
    def hp_callback(self, msg: RobotHP):
        if self.action.config.team_color == 'red':
            self.action.bb['hp'] = msg.red_7_robot_hp
            self.action.bb['outpost_hp'] = msg.red_outpost_hp
            self.action.bb['base_hp'] = msg.red_base_hp
        else:
            self.action.bb['hp'] = msg.blue_7_robot_hp
            self.action.bb['outpost_hp'] = msg.blue_outpost_hp
            self.action.bb['base_hp'] = msg.blue_base_hp

    def referee_callback(self, msg: GameStatus):
        # 留个口子，接收比赛阶段、金币数量等
        pass

    def odom_callback(self, msg: Odometry):
        self.action.bb['current_pose']['x'] = msg.pose.pose.position.x
        self.action.bb['current_pose']['y'] = msg.pose.pose.position.y

    def target_callback(self, msg: Target):
        if msg.tracking:
            self.action.bb['last_target_time'] = time.time()
            target_x = msg.position.x
            target_y = msg.position.y
            
            dx = target_x - self.action.bb['current_pose']['x']
            dy = target_y - self.action.bb['current_pose']['y']
            self.action.bb['target_distance'] = math.sqrt(dx**2 + dy**2)
            self.action.bb['pursuit_pose'] = {'x': target_x, 'y': target_y}

    def actual_posture_callback(self, msg: Int8):
        real_posture = msg.data
        if real_posture != self.action.bb['actual_posture']:
            self.action.bb['actual_posture'] = real_posture
            posture_dict = {0: "未知", 1: "进攻(Attack)", 2: "防御(Defense)", 3: "移动(Move)"}
            self.get_logger().info(f">>> [底层反馈] 裁判系统确认当前真实姿态已变为: {posture_dict.get(real_posture, '异常')} <<<")

def main(args=None):
    rclpy.init(args=args)
    node = RMUCDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()