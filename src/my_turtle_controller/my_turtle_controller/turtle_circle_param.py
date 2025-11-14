import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class TurtleCircleParam(Node):
    def __init__(self):
        super().__init__('turtle_circle_param')

        #declare a specific parameter for linear, angular
        self.declare_parameter('linear_speed', 2.0)    # linear speed
        self.declare_parameter('angular_speed', 1.0)   # angualr speed
        self.declare_parameter('duration', 2 * math.pi)  # duration

        # get_parameter().vaue: Retrieves the current value for the declared parameter and stores it in a python class attribute
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.duration = self.get_parameter('duration').value

        #ROS2 standard way to print an informational message to the console
        self.get_logger().info(
            f'linear_speed={self.linear_speed}, angular_speed={self.angular_speed}, '
            f'duration={self.duration}'
        )

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Creates the Twist message object
        self.cmd = Twist()
        
        # Sets the message's forward speed using the value read from the parameter
        self.cmd.linear.x = self.linear_speed
        
        
        self.cmd.angular.z = self.angular_speed

        # Records the exact time the node started running
        self.start_time = time.time()

        # 
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        elapsed = time.time() - self.start_time

        if elapsed < self.duration:
            self.publisher_.publish(self.cmd)
        else:
            stop_cmd = Twist()
            self.publisher_.publish(stop_cmd)
            self.get_logger().info('Finished circle, stopping turtle.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCircleParam()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

