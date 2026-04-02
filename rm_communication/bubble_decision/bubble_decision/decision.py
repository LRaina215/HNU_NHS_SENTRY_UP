import time
import rclpy
from rclpy.node import Node
from bubble_decision.gameAction import SentryGameAction

class Decision():
    STATE_GUARD = "GUARD (中心抢位/驻守)"
    STATE_COMBAT = "COMBAT (站桩输出)"
    STATE_DEFEND = "DEFEND (残血回撤)"
    STATE_OVERRIDE = "OVERRIDE (人类接管)"

    def __init__(self, node, robot_type, team_color, is_test_mode) -> None:
        self.node = node
        self.robot_type = robot_type
        self.team_color = team_color 
        self.is_test_mode = is_test_mode
        self.game = None
        self.current_state = "INIT"
        
        self.initRobot(robot_type)
        
        self.tick_rate = 0.1 
        self.timer = self.node.create_timer(self.tick_rate, self.tick)

    def initRobot(self, name):
        if name in ["sentry_up", "sentry_down"]:
            self.game = SentryGameAction(self.node, self.team_color, self.is_test_mode)
        elif name == "infantry":
            pass
        elif name == "hero":
            pass

    def set_state(self, new_state, reason=""):
        if self.current_state != new_state:
            self.node.get_logger().info(f"\n🚀 [UL 战术决策变更] {self.current_state} -> {new_state}")
            if reason:
                self.node.get_logger().info(f"   💡 触发原因: {reason}")
            self.current_state = new_state

    def tick(self):
        if not self.game or not hasattr(self.game, 'blackboard'):
            return

        # ==========================================
        # 🚨 终极拦截防火墙: 人类接管
        # ==========================================
        if time.time() < getattr(self.game, 'override_expire_time', 0.0):
            self.set_state(self.STATE_OVERRIDE, "云台手干预中，AI 强制进入 5 秒静默期！")
            return

        bb = self.game.blackboard

        # ==========================================
        # 🛡️ 优先级 1: 生存评估 (血量过低，返回补血)
        # ==========================================
        if bb['hp'] < bb['hp_low_threshold']:
            self.set_state(self.STATE_DEFEND, f"血量告急 (HP: {bb['hp']})，放弃阵地强制回撤！")
            self.game.execute_retreat_to_heal()
            return

        # ==========================================
        # ⚔️ 优先级 2: 战斗评估 (发现敌人，原地站桩打靶)
        # ==========================================
        if self.game.has_target():
            self.set_state(self.STATE_COMBAT, "发现敌方目标！开启原地站桩输出模式！")
            # 传入 True 表示进入战斗状态，下位机可开启小陀螺等攻击姿态
            self.game.execute_center_guard(combat_mode=True)
            return

        # ==========================================
        # 🚶 优先级 3: 常态驻守 (开局抢中 / 战区安全)
        # ==========================================
        self.set_state(self.STATE_GUARD, "战区安全或开局，前往/保持在中心交战区。")
        self.game.execute_center_guard(combat_mode=False)

class RobotAPI(Node):
    def __init__(self):
        super().__init__("Decision")
        self.declare_parameter('robot_type', 'sentry_down')
        self.declare_parameter('team_color', 'red')
        self.declare_parameter('is_test_mode', False)  
        
        name = self.get_parameter('robot_type').get_parameter_value().string_value
        team_color = self.get_parameter('team_color').get_parameter_value().string_value
        is_test_mode = self.get_parameter('is_test_mode').get_parameter_value().bool_value
        
        self.get_logger().info(f"✅ 成功启动 UL 决策节点 | 兵种: {name} | 阵营: {team_color} | 测试模式: {is_test_mode}")
        self.robot_decision = Decision(self, name, team_color, is_test_mode)

def main(args=None):
    rclpy.init(args=args)
    robot_api = RobotAPI()
    try:
        rclpy.spin(robot_api)
    except KeyboardInterrupt:
        pass
    finally:
        robot_api.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()