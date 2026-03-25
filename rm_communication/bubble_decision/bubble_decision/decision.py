import rclpy
from rclpy.node import Node
from bubble_decision.gameAction import SentryGameAction

class Decision():
    def __init__(self, node, robot_type, team_color) -> None:
        self.node = node
        self.robot_type = robot_type
        self.team_color = team_color 
        self.game = None
        
        self.initRobot(robot_type)
        
        # 核心决策循环 (10Hz)
        self.tick_rate = 0.1 
        self.timer = self.node.create_timer(self.tick_rate, self.tick)

    def initRobot(self, name):
        if name in ["sentry_up", "sentry_down"]:
            # 把 team_color 传给 GameAction
            self.game = SentryGameAction(self.node, self.team_color)
        elif name == "infantry":
            pass
        elif name == "hero":
            pass

    def tick(self):
        if not self.game or not hasattr(self.game, 'blackboard'):
            return

        # 1. 生存评估 (最高优先级)
        if self.game.blackboard['hp'] < self.game.blackboard['hp_low_threshold']:
            self.game.execute_retreat_to_heal()
            return

        # 2. 战斗评估 (次高优先级)
        if self.game.has_target():
            self.game.execute_combat()
            return

        # 3. 巡逻 (默认状态)
        self.game.execute_patrol()

class RobotAPI(Node):
    def __init__(self):
        super().__init__("Decision")
        self.declare_parameter('robot_type', 'sentry_down')
        self.declare_parameter('team_color', 'red')
        
        name = self.get_parameter('robot_type').get_parameter_value().string_value
        team_color = self.get_parameter('team_color').get_parameter_value().string_value
        
        self.get_logger().info(f"成功启动决策节点，兵种: {name}, 阵营: {team_color}")
        self.robot_decision = Decision(self, name, team_color)

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
