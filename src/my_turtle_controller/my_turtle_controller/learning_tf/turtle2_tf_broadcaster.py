import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class Turtle2TFBroadcaster(Node):
    def __init__(self):
        super().__init__('turtle2_tf_broadcaster')

        self.br = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.handle_pose,
            10
        )

        self.get_logger().info("Broadcasting TF: world → turtle2")

    def handle_pose(self, msg: Pose):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'turtle2'

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        t.transform.rotation.z = math.sin(msg.theta / 2.0)
        t.transform.rotation.w = math.cos(msg.theta / 2.0)

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Turtle2TFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

