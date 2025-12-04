import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import math


class TurtleTFGoToGoal(Node):
    def __init__(self):
        super().__init__('turtle_tf_listener_go_to_goal')

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # TF Buffer + Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Controller parameters
        self.declare_parameter('linear_gain', 1.0)
        self.declare_parameter('angular_gain', 4.0)
        self.declare_parameter('distance_tolerance', 0.1)

        self.k_lin = self.get_parameter('linear_gain').value
        self.k_ang = self.get_parameter('angular_gain').value
        self.dist_tol = self.get_parameter('distance_tolerance').value

        self.timer = self.create_timer(0.05, self.control_loop)

    def control_loop(self):
        try:
            # get goal in turtle1 frame
            tf = self.tf_buffer.lookup_transform(
                'turtle1',      # target frame
                'goal',         # source frame
                rclpy.time.Time()   # latest available
            )

            # get relative position
            dx = tf.transform.translation.x
            dy = tf.transform.translation.y
            dist = math.sqrt(dx*dx + dy*dy)

            cmd = Twist()

            if dist < self.dist_tol:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.get_logger().info("Goal reached via TF!")
                return

            # angle in turtle1 frame → simply atan2(dy, dx)
            angle_to_goal = math.atan2(dy, dx)

            cmd.angular.z = self.k_ang * angle_to_goal
            cmd.linear.x = self.k_lin * dist * max(0.0, math.cos(angle_to_goal))

            self.cmd_pub.publish(cmd)

        except (LookupException, ConnectivityException, ExtrapolationException):
            # wait for TF
            return
            
def main(args=None):
    rclpy.init(args=args)
    node = TurtleTFGoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

