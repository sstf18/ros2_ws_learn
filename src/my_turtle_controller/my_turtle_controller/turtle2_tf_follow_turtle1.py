import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException


class Turtle2FollowTurtle1(Node):
    def __init__(self):
        super().__init__('turtle2_tf_follow_turtle1')

        # declare parameter 
        self.declare_parameter('linear_gain', 1.0)
        self.declare_parameter('angular_gain', 6.0)
	
        self.k_lin =    self.get_parameter('linear_gain').get_parameter_value().double_value
        self.k_ang = self.get_parameter('angular_gain').get_parameter_value().double_value
	
        # pubulish to turtle2 cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.follow)
        
        self.get_logger().info(
            f"Turtle2 following turtle1 with gains: "
            f"linear_gain={self.k_lin}, angular_gain={self.k_ang}"
        )

    def follow(self):
        try:
            # lookup turtle1 relative to turtle2
            tf = self.tf_buffer.lookup_transform(
                'turtle2',     # target frame
	        'turtle1',     # source frame
	        rclpy.time.Time()
            )

            dx = tf.transform.translation.x
            dy = tf.transform.translation.y

            dist = math.sqrt(dx*dx + dy*dy)
            angle = math.atan2(dy, dx)

            cmd = Twist()

            # stop when very close
            if dist < 0.3:
                self.cmd_pub.publish(cmd)
                return

            # simple P control
            cmd.angular.z = self.k_ang * angle
            cmd.linear.x = self.k_lin * dist * max(0.0, math.cos(angle))

            self.cmd_pub.publish(cmd)

        except LookupException:
            # TF not ready yet
            return


def main(args=None):
    rclpy.init(args=args)
    node = Turtle2FollowTurtle1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

