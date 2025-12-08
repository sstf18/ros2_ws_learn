import math

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

'''
1. sub /turtle1/pose
2. convert pose into TF transform 
3. When 'pose' updateed --- publish TF --- 'turtule1' frame will update in the TF tree 
'''

class TurtleTFBroadcaster(Node):
    def __init__(self):
        super().__init__('turtle_tf_broadcaster')

        # create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # sub pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.handle_turtle_pose,
            10
        )

        self.get_logger().info("TF Broadcaster started: world -> turtle1")

    def handle_turtle_pose(self, msg: Pose):
        t = TransformStamped()

        # header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'     # parent 
        t.child_frame_id = 'turtle1'    # child (following turtle)

        # translation（pos）
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # rotation（Z）
        qz = math.sin(msg.theta / 2.0)
        qw = math.cos(msg.theta / 2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        # pub TF
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleTFBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

