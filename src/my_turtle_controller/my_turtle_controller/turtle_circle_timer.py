import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from rclpy.duration import Duration
import math


class TurtleCircleTimer(Node):
	def __init__(self):
		super().__init__('turtle_circle_timer') 
		self.publisher_ = self.create_publisher(Twist,'/turtle1/cmd_vel', 10)
		
		#get_clock().now() is a "Time Object" which is not a "float"
		# rclpy.time.Time
		self.start_time = self.get_clock().now()
		self.timer = self.create_timer(0.05, self.timer_callback)
		
		self.cmd = Twist()
		self.cmd.linear.x = 2.0
		self.cmd.angular.z = -1.0
		
		num_circles = 5
		
		# this duration is "float"
		float_one_circle_duration = 2 * math.pi / abs(self.cmd.angular.z)
		
		total_seconds = float_one_circle_duration * num_circles
		
		nanoseconds = int(total_seconds * 1_000_000_000)
		self.rclpy_duration = Duration(nanoseconds = nanoseconds)
		
	def timer_callback(self):
	
		elapsed = self.get_clock().now() - self.start_time
		if elapsed < self.rclpy_duration: 
			self.publisher_.publish(self.cmd)
		else: 
			stop_cmd = Twist()
			self.publisher_.publish(stop_cmd)
			self.get_logger().info('Finished circle, stopping turtle.')
			self.timer.cancel()
            		
def main(args=None):
	rclpy.init(args=args)
	node = TurtleCircleTimer()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
