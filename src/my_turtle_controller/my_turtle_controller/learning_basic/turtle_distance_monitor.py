import math

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtleDistanceMonitor(Node):
    def __init__(self):
        super().__init__('turtle_distance_monitor')

        # subscrption /turtle1/pose
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.subscription

        # record the last postion
        self.last_x = None
        self.last_y = None

        # accumilate distance
        self.total_distance = 0.0

        # set a milestone
        self.milestone = 5.0
        self.milestone_reached = False

        self.get_logger().info('turtle_distance_monitor node has started, begining record the total distance of turtle.')

    def pose_callback(self, msg: Pose):
        # the first time, only recaord the postion 
        if self.last_x is None:
            self.last_x = msg.x
            self.last_y = msg.y
            return

        # calculate the distance between the current position and last position
        dx = msg.x - self.last_x
        dy = msg.y - self.last_y
        step_distance = math.sqrt(dx * dx + dy * dy)

        # update total distance
        self.total_distance += step_distance

        # update last position
        self.last_x = msg.x
        self.last_y = msg.y

        # print current status
        self.get_logger().info(
            f"x={msg.x:.2f}, y={msg.y:.2f}, "
            f"theta={msg.theta:.2f}, "
            f"total distance = {self.total_distance:.2f}"
        )

        # if milestone reached，print "Milestone reached"
        if (not self.milestone_reached) and (self.total_distance >= self.milestone):
            self.milestone_reached = True
            self.get_logger().warn(
                f'🎯 Milestone reahced: The total distance is {self.milestone:.2f} ！'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TurtleDistanceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

