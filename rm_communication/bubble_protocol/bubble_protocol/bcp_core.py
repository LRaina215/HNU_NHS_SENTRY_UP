import rclpy

from bubble_protocol.dispatch import RobotAPI

def main(args=None):
    """BCP core entrance.
        BCP core入口。
    """
    rclpy.init(args=args)
    bcp_core = RobotAPI()
    rclpy.spin(bcp_core)
    bcp_core.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()