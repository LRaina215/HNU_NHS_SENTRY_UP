import math
import time
from rclpy.node import Node
from game_msgs.msg import RobotHP, GameStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, Bool
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import rclpy                  
import tf2_ros                


try:
    from auto_aim_interfaces.msg import Target
except ImportError:
    try:
        from rm_interfaces.msg import Target
    except ImportError:
        pass 

class FieldTacticsPlanner:
    def __init__(self, team_color, is_test_mode=False):
        self.team_color = team_color.lower()
        self.is_test_mode = is_test_mode
        
        # 既然 YAML 已经把原点设在场地正中心，所有坐标直接相对于中心点即可
        # 假设蓝方在左，红方在右
        self.relative_points = {
            'center': (0.0, 1.0),
            # 以下坐标需要你在 RViz 里以中心原点重新打点记录一次
            'blue_heal': (-5.25, 3.0), # 蓝方回血点 (举例)
            'red_heal': (5.25, -3.0)   # 红方回血点 (蓝方的中心对称点)
        }

    def get_center_zone(self):
        if self.is_test_mode:
            return {'x': 1.3, 'y': 0.543}
        return {'x': self.relative_points['center'][0], 'y': self.relative_points['center'][1]}
        
    def get_heal_zone(self):
        # 根据阵营直接返回对应坐标，不需要再做加减法偏移
        if self.team_color == 'blue':
            return {'x': self.relative_points['blue_heal'][0], 'y': self.relative_points['blue_heal'][1]}
        else:
            return {'x': self.relative_points['red_heal'][0], 'y': self.relative_points['red_heal'][1]}


class SentryGameAction():
    def __init__(self, node: Node, team_color: str, is_test_mode: bool) -> None:
        self.node = node    
        self.team_color = team_color if team_color in ['red', 'blue'] else 'red'
            
        self.blackboard = {
            'hp': 400,
            'hp_low_threshold': 150,    
            'bullet': 750,             
            'shoot_heat': 0,           
            'game_progress': 4,        
            'stage_remain_time': 300,  
            
            'last_target_time': 0.0,    
            'target_distance': 999.0,   
            'current_pose': {'x': 0.0, 'y': 0.0}, 
            'current_yaw': 0.0,          
        }
        
        planner = FieldTacticsPlanner(self.team_color, is_test_mode)
        # 精简为两个核心战术点
        self.waypoints = {
            'center_zone': planner.get_center_zone(),
            'heal_zone': planner.get_heal_zone()
        }

        self.nav_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.last_published_goal = None
        self.rviz_goal_pub = self.node.create_publisher(PoseStamped, '/ai_target_vis', 10)
        
        latch_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL 
        )
        self.posture_pub = self.node.create_publisher(Int8, '/manual/sentry_posture', latch_qos)

        self.posture_timer = self.node.create_timer(0.1, self._timer_posture_publish)

        self.expected_posture = 0
        
        self.expected_posture = 3          
        self.current_posture_cmd = 3
        self.last_posture_req_time = 0.0   
        self.last_posture_switch_time = 0.0
        self.POSTURE_COOLDOWN = 5.0
        self.override_expire_time = 0.0 
        
        self.hp_sub = self.node.create_subscription(RobotHP, '/status/robotHP', self.hp_callback, 10)
        self.referee_sub = self.node.create_subscription(GameStatus, '/referee/game_status', self.referee_callback, 10)
        self.sentry_info_sub = self.node.create_subscription(Int8, '/status/sentry_posture', self.sentry_info_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.pose_timer = self.node.create_timer(0.1, self.update_pose_from_tf)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.target_sub = self.node.create_subscription(
            Target, '/armor_solver/target', self.target_callback, sensor_qos)


    def sentry_info_callback(self, msg: Int8):
        actual_posture = msg.data  
        if actual_posture == self.expected_posture:
            return
        if time.time() - self.last_posture_req_time < 0.5:
            return 
            
        self.node.get_logger().warn(f"[终极警报] 发现人类干预！AI 期望 {self.expected_posture}，被篡改为 {actual_posture}！")
        self.expected_posture = actual_posture
        self.current_posture_cmd = actual_posture
        self.override_expire_time = time.time() + 5.0

    def hp_callback(self, msg: RobotHP):
        if self.team_color == 'red':
            if msg.red_7_robot_hp > 0:
                self.blackboard['hp'] = msg.red_7_robot_hp
        else:
            if msg.blue_7_robot_hp > 0:
                self.blackboard['hp'] = msg.blue_7_robot_hp

    def referee_callback(self, msg: GameStatus):
        if hasattr(msg, 'bullet_remain'): self.blackboard['bullet'] = msg.bullet_remain
        if hasattr(msg, 'shooter_heat'): self.blackboard['shoot_heat'] = msg.shooter_heat
        if hasattr(msg, 'game_progress'): self.blackboard['game_progress'] = msg.game_progress
        if hasattr(msg, 'stage_remain_time'): self.blackboard['stage_remain_time'] = msg.stage_remain_time

    def update_pose_from_tf(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.blackboard['current_pose']['x'] = t.transform.translation.x
            self.blackboard['current_pose']['y'] = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.blackboard['current_yaw'] = math.atan2(siny_cosp, cosy_cosp)
        except Exception as e:
            pass

    def target_callback(self, msg: Target):
        if msg.tracking:
            self.blackboard['last_target_time'] = time.time()
            rel_x, rel_y = msg.position.x, msg.position.y
            self.blackboard['target_distance'] = math.sqrt(rel_x**2 + rel_y**2)

    def has_target(self):
        return (time.time() - self.blackboard['last_target_time']) < 0.5

    def switch_posture(self, target_posture_id, reason):
        now = time.time()

        # 只要更新变量即可，定时器会自动把它持续发出去
        if target_posture_id != self.expected_posture:
            self.expected_posture = target_posture_id
            
            # 这两行用于人类接管检测逻辑，必须保留
            self.last_posture_req_time = now
            self.last_posture_switch_time = now 
            
            self.node.get_logger().info(f"[指令变更] 目标设为: {target_posture_id} ({reason})")

        self.expected_posture = target_posture_id
        self.last_posture_req_time = now

        msg = Int8()
        msg.data = target_posture_id
        self.posture_pub.publish(msg)
        self.current_posture_cmd = target_posture_id
        self.last_posture_switch_time = now
        self.node.get_logger().info(f"[下位机指令] -> 姿态 {target_posture_id} ({reason})")

    def _timer_posture_publish(self):
        """定时器回调：持续向底层下发当前期望的姿态/扫描值"""
        msg = Int8()
        msg.data = self.expected_posture
        self.posture_pub.publish(msg)

    def publish_nav_goal(self, point_dict, force=False):
        if not self.nav_client.server_is_ready():
            return

        now = time.time()
        dx, dy = 999.0, 999.0
        if self.last_published_goal is not None:
            dx = float(point_dict['x']) - self.last_published_goal['x']
            dy = float(point_dict['y']) - self.last_published_goal['y']

        last_time = getattr(self, 'last_goal_send_time', 0.0)
        
        # 如果不是强制发送，才走防抖逻辑
        if not force:
            if math.sqrt(dx**2 + dy**2) < 1.0 and (now - last_time) < 3.0:
                return

        self.last_published_goal = {'x': float(point_dict['x']), 'y': float(point_dict['y'])}
        self.last_goal_send_time = now
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map' 
        goal_msg.pose.pose.position.x = float(point_dict['x'])
        goal_msg.pose.pose.position.y = float(point_dict['y'])
        
        # 全向轮核心：保持当前朝向
        current_yaw = self.blackboard.get('current_yaw', 0.0)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(current_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(current_yaw / 2.0)
        
        self.nav_client.send_goal_async(goal_msg)

        vis_msg = PoseStamped()
        vis_msg.header = goal_msg.pose.header
        vis_msg.pose = goal_msg.pose.pose
        self.rviz_goal_pub.publish(vis_msg)

        self.node.get_logger().info(f"🚀 [Action] 下达坐标点: X={point_dict['x']:.2f}, Y={point_dict['y']:.2f}")

    # ================= 极简业务执行流 =================
    
    def execute_retreat_to_heal(self):
        """血量过低：开启护盾回撤"""
        self.switch_posture(0, "防御姿态：逃命中") 
        self.publish_nav_goal(self.waypoints['heal_zone'])

    def execute_center_guard(self, combat_mode=False):
        """核心业务：冲向中心并死守阵地"""
        target_pt = self.waypoints['center_zone']
        
        curr_x = self.blackboard['current_pose']['x']
        curr_y = self.blackboard['current_pose']['y']
        
        dx = curr_x - target_pt['x']
        dy = curr_y - target_pt['y'] 
        dist = math.sqrt(dx**2 + dy**2)
        
        self.node.get_logger().info(
            f"[阵地监控] 机器坐标:({curr_x:.2f}, {curr_y:.2f}) | 距中心: {dist:.2f}m", 
            throttle_duration_sec=2.0
        )
        
        # 判定：距离中心 1.0 米内视为“已落位”
        if dist < 1.0: 
            if combat_mode:
                self.switch_posture(0, "发现敌人！开启战斗爆发(小陀螺)！")
            else:
                self.switch_posture(1, "战区安全，进入警戒姿态！")
                
            # 重点：只在刚刚落位的那一刻，强制下发一次急刹指令
            if not getattr(self, 'is_guarding_locked', False):
                self.node.get_logger().info("踏入阵地范围，强制截断导航，执行原地急刹！")
                # 使用 force=True 绕过防抖，把当前坐标直接拍给 Nav2
                self.publish_nav_goal(self.blackboard['current_pose'], force=True)
                self.is_guarding_locked = True
                
        else:
            # 还没落位，坚决赶路，同时解除驻守锁
            self.is_guarding_locked = False
            self.switch_posture(0, "脱离阵地！全速向中心交战区靠拢！")
            self.publish_nav_goal(target_pt)