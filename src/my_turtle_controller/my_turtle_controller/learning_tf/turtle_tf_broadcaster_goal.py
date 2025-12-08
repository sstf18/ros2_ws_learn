import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class GoalTFBroadcaster(Node):
    def __init__(self):
        super().__init__('goal_tf_broadcaster')

        # Declare parameters for goal position
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.br = TransformBroadcaster(self)

        # publish at fixed 20Hz
        self.timer = self.create_timer(0.05, self.broadcast_goal_tf)

        self.get_logger().info(f"Broadcasting world→goal at ({self.goal_x}, {self.goal_y})")

    def broadcast_goal_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "goal"

        t.transform.translation.x = float(self.goal_x)
        t.transform.translation.y = float(self.goal_y)
        t.transform.translation.z = 0.0

        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GoalTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

