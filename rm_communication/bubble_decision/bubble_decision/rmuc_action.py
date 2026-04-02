# rmuc_action.py
import time
import math
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int8
from bubble_decision.rmuc_config import RMUCConfig
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy



class RMUCAction:
    STATE_PATROL = "PATROL"
    STATE_ENGAGE = "ENGAGE"
    STATE_DEFEND = "DEFEND"

    def __init__(self, node, team_color: str, is_test_field: bool):
        self.node = node
        self.config = RMUCConfig(team_color, is_test_field)
        self.points = self.config.get_tactical_points()
        self.marker_pub = self.node.create_publisher(Marker, '/fsm_visualization', 10)
        
        # 核心黑板 (Blackboard)
        self.bb = {
            'hp': 400,
            'outpost_hp': 1500,     
            'base_hp': 5000,
            'target_distance': 999.0,
            'last_target_time': 0.0,
            'current_pose': {'x': 0.0, 'y': 0.0},
            'pursuit_pose': {'x': 0.0, 'y': 0.0},
            'patrol_idx': 0,
            'actual_posture': 0
        }
        
        latch_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL  # 🚨 关键：实现类似 ROS 1 的 latch (锁存)
        )

        self.current_state = self.STATE_PATROL
        
        # 姿态冷却机制
        # 姿态状态记录
        self.current_posture_cmd = 0
        self.expected_posture = 3          # 🚨 新增：AI 的期望值，默认给 3 (移动)
        self.last_posture_req_time = 0.0   # 🚨 新增：记录时间戳
        self.last_posture_switch_time = 0.0
        self.POSTURE_COOLDOWN = 5.0

        # ================== 新增：AI 的期望记忆 ==================
        self.expected_posture = 3          # AI 当前期望的姿态 (默认3为移动)
        self.last_posture_req_time = 0.0   # AI 上次主动发申请的时间戳
        # ========================================================
        
        # 将发布者挂载在传入的 node 上
        self.nav_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.last_published_goal = None
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.posture_pub = self.node.create_publisher(Int8, '/manual/sentry_posture', latch_qos)
        self.last_published_goal = None

        # ====== 新增下面这一行 ======
        # 专门用来给 RViz2 画箭头的“视觉替身”话题，绝对不会干扰 Nav2
        self.rviz_goal_pub = self.node.create_publisher(PoseStamped, '/ai_target_vis', 10)

    def publish_visualization_marker(self):
        """发布 Rviz2 的 3D 悬浮状态指示器"""
        marker = Marker()
        marker.header.frame_id = "base_link"  # 绑定在机器人底盘中心坐标系上
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "sentry_fsm"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 悬浮在机器人上方 1.5 米处
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.5 
        
        # 字体大小
        marker.scale.z = 0.4
        
        # 根据当前状态设置颜色和文字
        marker.color = ColorRGBA()
        marker.color.a = 1.0 # 不透明度
        
        posture_str = {1: "Attack", 2: "Defense", 3: "Move"}.get(self.current_posture_cmd, "Unknown")
        
        if self.current_state == self.STATE_PATROL:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0 # 绿色
            marker.text = f"[{self.current_state}]\nPosture: {posture_str}\nTarget: Searching..."
        elif self.current_state == self.STATE_ENGAGE:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0 # 红色
            marker.text = f"[{self.current_state}]\nPosture: {posture_str}\nDist: {self.bb['target_distance']:.1f}m"
        elif self.current_state == self.STATE_DEFEND:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.6, 0.0 # 橙色
            marker.text = f"[{self.current_state}]\nPosture: {posture_str}\nHP: {self.bb['hp']}"

        self.marker_pub.publish(marker)

    def switch_posture(self, target_posture_id, reason):
        """执行姿态切换（受 5 秒冷却保护）"""
        now = time.time()
        
        # 🚨 如果目标姿态和我们期望的姿态一样，说明已经发过请求了，直接跳过
        if target_posture_id == self.expected_posture:
            return
            
        if (now - self.last_posture_switch_time) < self.POSTURE_COOLDOWN:
            return

        # ================== 核心修改：记录期望和时间 ==================
        self.expected_posture = target_posture_id  # 更新 AI 的期望姿态
        self.last_posture_req_time = now           # 记录发起请求的绝对时间
        # ==============================================================

        msg = Int8()
        msg.data = target_posture_id
        self.posture_pub.publish(msg)
        self.current_posture_cmd = target_posture_id
        self.last_posture_switch_time = now
        
        posture_names = {1: "进攻(Attack)", 2: "防御(Defense)", 3: "移动(Move)"}
        self.node.get_logger().info(
            f"\n[RMUC 战术变更 (AI自主)] -> {posture_names.get(target_posture_id)}\n"
            f"触发原因: {reason}\n"
            f"冷却倒计时开始: 5.0s"
        )

    def publish_goal(self, point_dict):
        # 1. 距离滤波器（防重发，阈值设为 1.0 米）
        if self.last_published_goal is not None:
            dx = float(point_dict['x']) - self.last_published_goal['x']
            dy = float(point_dict['y']) - self.last_published_goal['y']
            if math.sqrt(dx**2 + dy**2) < 1.0:
                return

        self.last_published_goal = {'x': float(point_dict['x']), 'y': float(point_dict['y'])}
        
        # 2. 检查 Nav2 服务器是否活着
        if not self.nav_client.server_is_ready():
            self.node.get_logger().warn("Nav2 Action服务器未就绪，目标发送被挂起...")
            return

        # 3. 构造 Action 目标并直接发送（完美绕过 RViz！）
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map' 
        goal_msg.pose.pose.position.x = float(point_dict['x'])
        goal_msg.pose.pose.position.y = float(point_dict['y'])
        goal_msg.pose.pose.orientation.w = 1.0  
        
        # 异步发送，绝对不卡主线程
        self.nav_client.send_goal_async(goal_msg)

        # 组装一个普通的消息发给 RViz2 专属话题，用来画图
        vis_msg = PoseStamped()
        vis_msg.header = goal_msg.pose.header
        vis_msg.pose = goal_msg.pose.pose
        self.rviz_goal_pub.publish(vis_msg)
        # ==========================

        self.node.get_logger().info(f"🚀 [Action直连] 成功绕过RViz，直接向Nav2发送目标: X={point_dict['x']:.2f}, Y={point_dict['y']:.2f}")

    def tick(self):
        """核心状态机逻辑，由外层 Decision 定时调用"""
        has_enemy = (time.time() - self.bb['last_target_time']) < 0.5
        is_low_hp = self.bb['hp'] < 150
        is_home_attacked = self.bb['outpost_hp'] < 500 or self.bb['base_hp'] < 2000

        # 1. 状态转移判断
        if is_low_hp or is_home_attacked:
            self.current_state = self.STATE_DEFEND
        elif has_enemy:
            self.current_state = self.STATE_ENGAGE
        else:
            self.current_state = self.STATE_PATROL

        # 2. 状态动作执行
        if self.current_state == self.STATE_DEFEND:
            self.execute_defend(is_low_hp)
        elif self.current_state == self.STATE_ENGAGE:
            self.execute_engage()
        elif self.current_state == self.STATE_PATROL:
            self.execute_patrol()
        self.publish_visualization_marker()

    def execute_defend(self, is_low_hp):
        self.switch_posture(2, "血量危急或家被偷，切防御姿态获取护盾！")
        if is_low_hp:
            self.publish_goal(self.points['heal_zone'])
        else:
            self.publish_goal(self.points['base_guard'])
            
        # twist = Twist()
        # twist.angular.z = 6.0
        # self.cmd_vel_pub.publish(twist)

    def execute_engage(self):
        dist = self.bb['target_distance']
        # twist = Twist()
        
        if dist < 2.0:
            self.switch_posture(1, f"敌方近身 (距 {dist:.1f}m)，切进攻姿态倾泻火力！")
            self.publish_goal(self.bb['current_pose']) 
            # twist.angular.z = 2.0 
        else:
            self.switch_posture(3, f"发现敌方 (距 {dist:.1f}m)，切移动姿态追击！")
            target_pose = self.bb.get('pursuit_pose')
            if target_pose:
                self.publish_goal(target_pose)
            # twist.angular.z = 0.0 

        # self.cmd_vel_pub.publish(twist)

    def execute_patrol(self):
        self.switch_posture(3, "战区安全，进入高机动巡逻模式。")
        
        idx = self.bb['patrol_idx']
        target_pt = self.points['patrol_route'][idx]
        
        dx = self.bb['current_pose']['x'] - target_pt['x']
        dy = self.bb['current_pose']['y'] - target_pt['y']
        
        if math.sqrt(dx**2 + dy**2) < 1.0:
            self.bb['patrol_idx'] = (idx + 1) % len(self.points['patrol_route'])
            return
            
        self.publish_goal(target_pt)
        
        # twist = Twist()
        # twist.angular.z = 1.0
        # self.cmd_vel_pub.publish(twist)