import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class DiffDriveCircle(Node):
    def __init__(self):
        super().__init__('diff_drive_circle')

        # parameter：wheel separation and wheel radius（unit：m）
        self.declare_parameter('wheel_separation', 0.4)   # wheel separation
        self.declare_parameter('wheel_radius', 0.12)      # wheel radius

        self.wheel_separation = self.get_parameter(
            'wheel_separation').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter(
            'wheel_radius').get_parameter_value().double_value

        # new added 
        self.FIXED_LINEAR_VELOCITY = 0.2 
        self.FIXED_ANGULAR_VELOCITY = 0.2

        # initializes the robot's odometry pose (orientation yaw)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # wheel position
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # subscriotion topic /cmd_vel
        #self.cmd_sub = self.create_subscription(
        #    Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # publisher topic /odom
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # publish topic /joint_states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # TF broadcaster：publish odom -> base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # timer（50 Hz）
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.update)

        self.get_logger().info('DIffDriveCircle node started.')

    # method calld when a new Twist message arrives on the "cmd_vel" topic 
    #def cmd_vel_callback(self, msg: Twist):
    #    self.cmd_lin = msg.linear.x
    #    self.cmd_ang = msg.angular.z
        
    # the main simulation method, called by the timer every 0.02 seconds
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0:
            return

        v = self.FIXED_LINEAR_VELOCITY
        w = self.FIXED_ANGULAR_VELOCITY

        # speed diff：update robot position in odometry 
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w * dt

        # normalize yaw to ensure it stays within the range [-pi, pi] radians
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # based on v, w calculate wheel separation of left wheel and right wheel' s v
        # (v+-w*L/2) is linear velocity, the the total formal is angualr velocity
        w_left = (v - w * self.wheel_separation / 2.0) / self.wheel_radius
        w_right = (v + w * self.wheel_separation / 2.0) / self.wheel_radius

        # get new wheel postion
        self.left_wheel_pos += w_left * dt
        self.right_wheel_pos += w_right * dt

        # publish odom
        self.publish_odom(now, v, w)

        # publish joint_states
        self.publish_joint_states(now)

        # publish TF：odom -> base_link
        self.publish_tf(now)

    def publish_odom(self, stamp: Time, v: float, w: float):
        # initializes the Odometry message, setting the timestamp and defining the transform: odom frame -> base_link frame
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # sets the position(x,y) of the robot in the odom frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # calculates the sets the orientation as a quaternion
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

    def publish_joint_states(self, stamp: Time):
        js = JointState()
        js.header.stamp = stamp.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_pos, self.right_wheel_pos]
        self.joint_pub.publish(js)

    def publish_tf(self, stamp: Time):
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

