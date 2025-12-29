import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeScanPublisher(Node):
    """
    Publishes a fake 2D LaserScan in lidar_link frame.
    This is for learning TF/map->odom concept without Gazebo sensor issues.
    """

    def __init__(self):
        super().__init__("fake_scan_publisher")

        # Parameters
        self.declare_parameter("frame_id", "lidar_link")
        self.declare_parameter("topic", "/fake_scan")
        self.declare_parameter("rate_hz", 10.0)

        # A simple "world": one circular wall around robot
        self.declare_parameter("range_min", 0.12)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("circle_radius", 3.0)   # meters, fake obstacle ring

        self.frame_id = self.get_parameter("frame_id").value
        self.topic = self.get_parameter("topic").value
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.circle_radius = float(self.get_parameter("circle_radius").value)

        self.pub = self.create_publisher(LaserScan, self.topic, 10)

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"FakeScanPublisher started: topic={self.topic}, frame_id={self.frame_id}, rate={self.rate_hz}Hz"
        )

    def on_timer(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # 360-degree scan
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.radians(1.0)  # 1 degree resolution

        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.rate_hz

        msg.range_min = self.range_min
        msg.range_max = self.range_max

        n = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        ranges = []

        # Fake "circular wall": same distance in all directions
        for _ in range(n):
            r = self.circle_radius
            # clamp to [min,max]
            r = max(self.range_min, min(self.range_max, r))
            ranges.append(r)

        msg.ranges = ranges
        msg.intensities = []

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

