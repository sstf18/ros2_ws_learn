import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class TurtleCircleParam(Node):
    def __init__(self):
        super().__init__('turtle_circle_param')

        # 声明参数（带默认值）
        self.declare_parameter('linear_speed', 2.0)    # 线速度
        self.declare_parameter('angular_speed', 1.0)   # 角速度
        self.declare_parameter('duration', 2 * math.pi)  # 运动时间（秒），默认大约转一圈

        # 读取参数
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.duration = self.get_parameter('duration').value

        self.get_logger().info(
            f'linear_speed={self.linear_speed}, angular_speed={self.angular_speed}, '
            f'duration={self.duration}'
        )

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 要发布的速度指令
        self.cmd = Twist()
        self.cmd.linear.x = self.linear_speed
        self.cmd.angular.z = self.angular_speed

        # 记录开始时间
        self.start_time = time.time()

        # 50ms 调用一次回调，大约 20Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        elapsed = time.time() - self.start_time

        if elapsed < self.duration:
            # 还没到指定时间，继续转圈
            self.publisher_.publish(self.cmd)
        else:
            # 到时间了，发送停止指令，然后关闭节点
            stop_cmd = Twist()
            self.publisher_.publish(stop_cmd)
            self.get_logger().info('Finished circle, stopping turtle.')
            # 这里直接关掉 rclpy，结束程序
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCircleParam()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

