import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import threading
import sys

class PostureTeleop(Node):
    def __init__(self):
        super().__init__('posture_teleop_node')
        # 创建发布者，话题名要和 dispatch.py 里的订阅名对上
        self.publisher_ = self.create_publisher(Int8, '/manual/sentry_posture', 10)
        
        self.get_logger().info('\n==================================\n'
                               '哨兵手动姿态控制终端已启动！\n'
                               '[1] -> 进攻姿态\n'
                               '[2] -> 防御姿态\n'
                               '[3] -> 移动姿态\n'
                               '[q] -> 退出程序\n'
                               '==================================')

        # 开启独立后台线程用于读取键盘输入，防止 input() 阻塞 ROS 2 的主循环
        self.input_thread = threading.Thread(target=self.read_keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def read_keyboard_input(self):
        while rclpy.ok():
            try:
                # 监听命令行输入
                cmd = input("请输入指令 (1/2/3): ")
                
                if cmd.lower() == 'q':
                    self.get_logger().info('退出姿态控制终端。')
                    rclpy.shutdown()
                    break
                
                cmd_int = int(cmd)
                if cmd_int in [1, 2, 3]:
                    msg = Int8()
                    msg.data = cmd_int
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'>> 已发布指令: {cmd_int}')
                else:
                    self.get_logger().warn('无效输入！只能输入 1, 2, 或 3。')
            except ValueError:
                self.get_logger().warn('无效格式！请输入纯数字 1, 2, 或 3。')
            except EOFError:
                break

def main(args=None):
    rclpy.init(args=args)
    node = PostureTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()