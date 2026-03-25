# /bubble/src/bubble_contrib/bubble_decision/bubble_decision/rmuc_body.py
import rclpy
from rclpy.node import Node
import time
import math
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry
from game_msgs.msg import RobotHP
from geometry_msgs.msg import Twist, Vector3 
from visualization_msgs.msg import Marker      # 新增可视化导入
from std_msgs.msg import ColorRGBA             # 新增可视化颜色导入
from bubble_decision.rmuc_action import RMUCAction
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool 
from std_msgs.msg import Int8

try:
    from auto_aim_interfaces.msg import Target
except ImportError:
    try:
        from rm_interfaces.msg import Target
    except ImportError:
        pass 

class RMUCBodyNode(Node):
    def __init__(self):
        super().__init__('rmuc_body_node')
        
        # 声明参数
        self.declare_parameter('team_color', 'red')
        self.declare_parameter('is_test_field', True)
        team_color = self.get_parameter('team_color').value
        is_test_field = self.get_parameter('is_test_field').value

        # 实例化动作引擎 (Python 四肢)
        self.action_engine = RMUCAction(self, team_color=team_color, is_test_field=is_test_field)
        self.bb = self.action_engine.bb  # 拿到动作引擎的黑板引用
        
        # 1. 订阅 C++ 大脑发来的【宏观战术指令】
        self.cmd_sub = self.create_subscription(String, '/sentry/tactic_cmd', self.cmd_callback, 10)
            
        # 2. 恢复【微观传感器订阅】，让四肢知道该怎么发力
        self.hp_sub = self.create_subscription(RobotHP, '/status/robotHP', self.hp_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        
        # 专门为视觉高频数据配置 Best Effort QoS，迎合 /armor_solver/target
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.target_sub = self.create_subscription(
            Target, '/armor_solver/target', self.target_callback, sensor_qos)

        # 3. 向 C++ 大脑汇报情况的“神经”
        self.bb_pub = self.create_publisher(Float32MultiArray, '/sentry/blackboard_data', 10)
        self.report_timer = self.create_timer(0.1, self.report_to_brain)

        # 4. 新增：接管 3D 悬浮字可视化发布
        self.current_tactic = "WAITING"
        self.marker_pub = self.create_publisher(Marker, '/uc_fsm_visualization', 10)
        self.vis_timer = self.create_timer(0.1, self.publish_visualization)

        # 覆写锁：记录人类接管的到期时间 (Unix 时间戳)
        self.override_expire_time = 0.0
        # 订阅裁判系统 0x020D 哨兵信息同步 (请替换为实际的 Topic 和 Msg 类型)
        # self.sentry_info_sub = self.create_subscription(Int8, '/status/sentry_posture_feedback', self.sentry_info_callback, 10) # 比赛用
        self.sentry_info_sub = self.create_subscription(Int8, '/status/sentry_posture', self.sentry_info_callback, 10) # 裁判系统离线（测试用）
        # 新增：向 C++ 大脑汇报人类接管状态的话题
        self.override_pub = self.create_publisher(Bool, '/sentry/human_override', 10)
        # 创建一个定时器，以 10Hz 的频率向大脑汇报
        self.override_timer = self.create_timer(0.1, self.publish_override_state)

        self.get_logger().info("Python 动作执行层已启动，随时准备响应 C++ 决策树...")

    # ================= 更新微观环境感知 =================
    def hp_callback(self, msg: RobotHP):
        if self.action_engine.config.team_color == 'red':
            # 只有当裁判系统发来的血量大于 0 时，才认为是有效数据
            if msg.red_7_robot_hp > 0:
                self.bb['hp'] = msg.red_7_robot_hp
                self.bb['outpost_hp'] = msg.red_outpost_hp
                self.bb['base_hp'] = msg.red_base_hp
        else:
            if msg.blue_7_robot_hp > 0:
                self.bb['hp'] = msg.blue_7_robot_hp
                self.bb['outpost_hp'] = msg.blue_outpost_hp
                self.bb['base_hp'] = msg.blue_base_hp

    # ================= 修正后的感知层 =================
    def odom_callback(self, msg: Odometry):
        # 记录底盘在 map 中的绝对位置
        self.bb['current_pose']['x'] = msg.pose.pose.position.x
        self.bb['current_pose']['y'] = msg.pose.pose.position.y
        
        # 将四元数转换为欧拉角 (Yaw)，用于后续的坐标系旋转
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.bb['current_yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def target_callback(self, msg: Target):
        if msg.tracking:
            self.bb['last_target_time'] = time.time()
            
            # 1. 修正距离计算：自瞄发来的 position 本身就是相对于相机的距离向量
            rel_x = msg.position.x
            rel_y = msg.position.y
            self.bb['target_distance'] = math.sqrt(rel_x**2 + rel_y**2)
            
            # 2. 修正导航目标点：将【局部相对坐标】转换到【map 全局坐标】
            # 获取底盘当前位置和朝向 (如果在原地还没收到 odom，默认 yaw 为 0)
            yaw = self.bb.get('current_yaw', 0.0)
            curr_x = self.bb['current_pose']['x']
            curr_y = self.bb['current_pose']['y']
            
            # 核心：2D 齐次坐标变换 (旋转 + 平移)
            # 全局 X = 底盘 X + 相对 X * cos(yaw) - 相对 Y * sin(yaw)
            # 全局 Y = 底盘 Y + 相对 X * sin(yaw) + 相对 Y * cos(yaw)
            map_x = curr_x + (rel_x * math.cos(yaw) - rel_y * math.sin(yaw))
            map_y = curr_y + (rel_x * math.sin(yaw) + rel_y * math.cos(yaw))
            
            # 将算出的真实的敌方绝对坐标存入黑板，供 PURSUIT 姿态使用
            self.bb['pursuit_pose'] = {'x': map_x, 'y': map_y}

    # ================= 汇报大脑与可视化 =================
    def report_to_brain(self):
        msg = Float32MultiArray()
        # 判断当前是否有敌人 (0.5秒内更新过)
        has_enemy = 1.0 if (time.time() - self.bb['last_target_time'] < 0.5) else 0.0
        # 打包成数组: [血量, 是否有敌人, 敌人距离]
        msg.data = [float(self.bb['hp']), has_enemy, float(self.bb['target_distance'])]
        self.bb_pub.publish(msg)

    def publish_visualization(self):
        marker = Marker()
        # 测试时如果没开导航，可先用 map，真车联调建议绑在 base_link 或 map
        marker.header.frame_id = "map"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "uc_sentry_fsm"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 悬浮在机器人坐标上方 1.5 米处
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.5 
        marker.scale.z = 0.4
        
        marker.color = ColorRGBA()
        marker.color.a = 1.0
        
        # 获取动作引擎底层的真实姿态数字
        posture_str = {1: "Attack", 2: "Defense", 3: "Move"}.get(self.action_engine.current_posture_cmd, "Unknown")
        
        if self.current_tactic == "PATROL":
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0 # 绿
            marker.text = f"[{self.current_tactic}]\nPosture: {posture_str}\nHP: {self.bb['hp']}"
        elif self.current_tactic in ["ENGAGE", "MELEE", "PURSUIT"]:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0 # 红
            marker.text = f"[{self.current_tactic}]\nPosture: {posture_str}\nDist: {self.bb['target_distance']:.1f}m"
        elif self.current_tactic == "DEFEND":
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.6, 0.0 # 橙
            marker.text = f"[{self.current_tactic}]\nPosture: {posture_str}\nHP Crisis!"
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 1.0 # 白
            marker.text = f"[{self.current_tactic}]\nWaiting for C++ Brain..."

        self.marker_pub.publish(marker)

    # ================= 执行 C++ 大脑的命令 =================
    def cmd_callback(self, msg: String):
        # 终极拦截防火墙
        # 如果当前时间还没过“人类接管保护期”，直接丢弃 AI 的任何指令！
        if time.time() < self.override_expire_time:
            # 处于禁闭期，直接 Return 掉，底层小车将保持当前状态（或执行操作手用摇杆发的指令）
            return 
            
        # ====== 如果没被接管（禁闭期结束），才正常执行后续的 AI 逻辑 ======
        cmd = msg.data
        self.current_tactic = cmd  
        
        if cmd == "DEFEND":
            self.action_engine.switch_posture(2, "撤退")
            self.action_engine.publish_goal(self.action_engine.points.get('heal_zone', self.bb['current_pose']))
            # 绝对不发 Twist，让 Nav2 开车！
            
        elif cmd == "MELEE":
            self.action_engine.switch_posture(1, "近战爆发")
            # 只有近战不需要导航，我们自己接管底盘开启陀螺！
            # t = Twist()
            # t.angular.z = 2.0
            # self.action_engine.cmd_vel_pub.publish(t)
            
        elif cmd == "PURSUIT":
            self.action_engine.switch_posture(3, "移动追击")
            if self.bb.get('pursuit_pose'):
                self.action_engine.publish_goal(self.bb['pursuit_pose'])
            # 绝对不发 Twist，让 Nav2 追人！
            
        elif cmd == "PATROL":
            self.action_engine.execute_patrol()
            # 绝对不发 Twist，让 Nav2 巡逻！

    def sentry_info_callback(self, msg):
        """监听裁判系统下发的真实姿态 (0x020D)"""
        # 注意：msg.posture_id 需要换成你们实际 msg 里定义的名字
        actual_posture = msg.data  
        
        # 情况 A：期望 == 实际。大家达成了共识，相安无事
        if actual_posture == self.action_engine.expected_posture:
            return
            
        # 既然 期望 != 实际，先看是不是网络延迟？
        # 规则：如果 AI 在 1.0 秒内刚刚发过请求，裁判系统可能还没反应过来，忽略！
        if time.time() - self.action_engine.last_posture_req_time < 0.5:
            return 
            
        # ================== 🚨 情况 B：发现人类干预！ 🚨 ==================
        # 距离上次发请求已经很久了，但服务器的姿态竟然变了，必定是云台手改了数据！
        
        self.get_logger().warn(f" [终极警报] 发现人类干预！AI 期望姿态 {self.action_engine.expected_posture}，被强制篡改为 {actual_posture}！")
        
        # 1. 认怂：AI 的期望必须屈服于人类的实际指令
        self.action_engine.expected_posture = actual_posture
        self.action_engine.current_posture_cmd = actual_posture # 同步底层状态
        
        # 2. 触发最高防火墙：给 AI 大脑关 5 秒禁闭！
        self.override_expire_time = time.time() + 5.0
        self.current_tactic = "POSTURE_OVERRIDE (人类接管)"

    def publish_override_state(self):
        msg = Bool()
        # 判断当前是否处于 5 秒禁闭期内
        if time.time() < self.override_expire_time:
            msg.data = True
        else:
            msg.data = False
        self.override_pub.publish(msg)
            
def main(args=None):
    rclpy.init(args=args)
    node = RMUCBodyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()