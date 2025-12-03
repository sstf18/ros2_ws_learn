import math

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


def normalize_angle(angle):
    """ Normalize the angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TurtleGoToGoal(Node):
    def __init__(self):
        super().__init__('turtle_go_to_goal')

        # declare parameter (can overwrite in terminal)
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)
        self.declare_parameter('linear_gain', 1.0)
        self.declare_parameter('angular_gain', 4.0)
        self.declare_parameter('distance_tolerance', 0.1)

	# red the actual values into member variables that will be used in the control loop. 
	# k_lin and k_kang are the proportuinal control gains for linear and angular velocity.
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.k_lin = self.get_parameter('linear_gain').get_parameter_value().double_value
        self.k_ang = self.get_parameter('angular_gain').get_parameter_value().double_value
        self.dist_tol = self.get_parameter(
            'distance_tolerance').get_parameter_value().double_value

        self.waypoints = [
            (2.0, 2.0),
            (8.0, 2.0),
            (8.0, 8.0),
            (2.0, 8.0)
        ]
        
        self.current_wp_index = 0 
        self.goal_x, self.goal_y = self.waypoints[self.current_wp_index]
        
        self.get_logger().info(
            f"Starting waypoint patrol. First goal: "
            f"({self.goal_x}, {self.goal_y})"
        )


        self.get_logger().info(
            f"Waypoint follower started. "
            f"First Goal = ({self.goal_x:.2f}, {self.goal_y:.2f}), "
            f"k_lin={self.k_lin}, k_ang={self.k_ang}, "
            f"dist_tol={self.dist_tol}"
        )

        # current pose（update from subscriber）
        self.current_pose = None

        # publish /turtle1/cmd_vel
        self.cmd_pub = self.create_publisher(
		Twist, 
		'/turtle1/cmd_vel', 
		10
	)

        # subscriber /turtle1/pose
        self.pose_sub = self.create_subscription(
        	Pose,
            	'/turtle1/pose',
            	self.pose_callback,
            	10
        )

        # timer：
        self.timer = self.create_timer(0.05, self.control_loop)

        # check if the goal was reached
        self.goal_reached = False
       
    def switch_to_next_waypoint(self):
        
        self.current_wp_index += 1

        if self.current_wp_index >= len(self.waypoints):
            self.current_wp_index = 0  

        self.goal_x, self.goal_y = self.waypoints[self.current_wp_index]
        self.goal_reached = False

        self.get_logger().info(
            f"Switch to waypoint #{self.current_wp_index}: "
            f"({self.goal_x:.2f}, {self.goal_y:.2f})"
        )
        


    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    #it first checks if a pose message has been received yet; if not, it exits early
    def control_loop(self):
        # dont recive current position from subscriber, do nothing
        if self.current_pose is None:
            return

	#calculate error: 
        x = self.current_pose.x
        y = self.current_pose.y
        theta = self.current_pose.theta

        dx = self.goal_x - x
        dy = self.goal_y - y

        dist = math.sqrt(dx * dx + dy * dy)

        cmd = Twist()

	# goal check 
        if dist < self.dist_tol:
            # reach the goal, stop, then pubulish 
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info(
                    f"Goal reached! (x={x:.2f}, y={y:.2f}, dist={dist:.3f})"
                )
                self.switch_to_next_waypoint()
            return

        # calcualte target angle/error
        #the difference between this target angle and the turtle's current heading(theta) gives the angle_error. 
        target_angle = math.atan2(dy, dx) 
        angle_error = normalize_angle(target_angle - theta)

        # angualr control
        cmd.angular.z = self.k_ang * angle_error

        # linear control with heading lock. 
        # example|angle_error| > 90° canot move
        angle_factor = max(0.0, math.cos(angle_error))
        cmd.linear.x = self.k_lin * dist * angle_factor

        # limits the maximum linear speed to 2.0 mls, maixmum angular speed to 4.0 rad/s to ensure stable and physically movement
        cmd.linear.x = max(min(cmd.linear.x, 2.0), -2.0)
        cmd.angular.z = max(min(cmd.angular.z, 4.0), -4.0)

        self.cmd_pub.publish(cmd)

        self.get_logger().debug(
            f"x={x:.2f}, y={y:.2f}, dist={dist:.2f}, "
            f"angle_error={angle_error:.2f}, "
            f"vx={cmd.linear.x:.2f}, wz={cmd.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TurtleGoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

