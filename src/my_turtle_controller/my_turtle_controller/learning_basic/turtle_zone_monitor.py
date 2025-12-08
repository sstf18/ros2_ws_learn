import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose #this message contain the turtle's x, y, theta(orientation), and velocity


class TurtleZoneMonitor(Node):
    def __init__(self):
        super().__init__('turtle_zone_monitor')

        # create subscription / pose
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.subscription # this line prevents a potential warning in some python linters that self.subscription is defined but never used. 

        # declear zone boundaries
        self.x_min = 5.0
        self.x_max = 6.0
        self.y_min = 5.0
        self.y_max = 6.0

        # state variable to keep track of whether the turtle is currently inside the zone. Initalied to None because the initial state is unknown tuntil the first pose message is recived.
        self.inside_zone = None

        self.get_logger().info(
            f"turtle_zone_monitor started. "
            f"Zone: x in [{self.x_min}, {self.x_max}], "
            f"y in [{self.y_min}, {self.y_max}]"
        )

    #callback automatically whenever a Pose message is received on subscribed topic. 
    def pose_callback(self, msg: Pose):
    
        # check current position, boolean variable in_zone is set to 'True', otherwise, it's False
        in_zone = (
            self.x_min <= msg.x <= self.x_max and
            self.y_min <= msg.y <= self.y_max
        )

        # first message, init postion is not in the zone
        if self.inside_zone is None:
            self.inside_zone = in_zone
            state_str = "INSIDE" if in_zone else "OUTSIDE"
            self.get_logger().info(
                f"Initial state: {state_str} the zone. "
                f"(x={msg.x:.2f}, y={msg.y:.2f})"
            )
            return

        # Checks if the previous that(self.inside_zone) was 'False' (outside), current is 'in_zone'
        if (not self.inside_zone) and in_zone:
            self.inside_zone = True
            self.get_logger().warn(
                f"⚠️ Turtle ENTERED the zone! "
                f"(x={msg.x:.2f}, y={msg.y:.2f})"
            )

        # Check if the previous was 'True'(inside), current is 'not in_zone'
        elif self.inside_zone and (not in_zone):
            self.inside_zone = False
            self.get_logger().info(
                f"ℹ️ Turtle EXITED the zone. "
                f"(x={msg.x:.2f}, y={msg.y:.2f})"
            )

        # 状态没变，可选：偶尔打印一下位置（这里简单略过）
        # else:
        #     self.get_logger().debug(
        #         f"x={msg.x:.2f}, y={msg.y:.2f}, in_zone={in_zone}"
        #     )


def main(args=None):
    rclpy.init(args=args)            # init ROS
    node = TurtleZoneMonitor()       # create a instance
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

