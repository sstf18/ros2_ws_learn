import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from rclpy.duration import Duration

import math
import threading # handle keyboard input concurrently with the main ROS event loop (rclpy.spin())
import sys
import time
import termios # modify terminal settings so that individual keystrokes are raed immediately
import tty

class TurtleKeyboardTeleop(Node):
	def __init__(self):
	
		# intergit from ros
		super().__init__('turtle_keyboard_teleop')
		
		# create a publisher
		self.publisher_=self.create_publisher(Twist, '/turtle1/cmd_vel',10)
		
		# parameters 
		self.declare_parameter('linear_speed', 1.0)
		self.declare_parameter('angular_speed', 1.5)
		
		self.linear_speed = self.get_parameter(
			'linear_speed').get_parameter_value().double_value
		self.angular_speed = self.get_parameter(
			'angular_speed').get_parameter_value().double_value
			
		# state variables
		self.current_linear = 0.0 
		self.current_angular = 0.0
		
		self.auto_stop_timeout = 0.5
		self.last_key_time = self.get_clock().now()
		
		self.running = True
		
		# timer: 20 Hz
		self.timer = self.create_timer(0.05, self.timer_callback)
		
		self.get_logger().info("Starting keyboard teleop. "
					"Use WASD to move, SPACE to stop, Q to quit.")
		# keyboard thread		
		self.keyboard_thread = threading.Thread(
			target = self.keyboard_loop, 
			daemon = True
		)
		self.keyboard_thread.start()
		
		
	def timer_callback(self):
		#auto stop check
		now = self.get_clock().now()
		dt = (now -self.last_key_time).nanoseconds / 1e9
		
		if dt > self.auto_stop_timeout: 
			self.current_linear = 0.0 
			self.current_angular = 0.0 
			
		# publish Twist
		msg = Twist()
		msg.linear.x = self.current_linear 
		msg.angular.z = self.current_angular 
		self.publisher_.publish(msg)
		
	
	
	def keyboard_loop(self):
		# Save terminal settings 
		old_settings = termios.tcgetattr(sys.stdin)
		
		try: 
			# Switch terminal to raw mode (no Enter needed)
			tty.setraw(sys.stdin.fileno())
			
			while self.running: 
				key = sys.stdin.read(1)
				self.process_key(key)
				
		except Exception as e: 
			self.get_logger().error(f"Keyboard loop error: {e}")
		finally: 
			# Restore normal terminal settings
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
	
	def process_key(self, key: str):
		self.last_key_time = self.get_clock().now()
		
		lin = 0.0 
		ang = 0.0 
		
		if key == 'w':
			lin = self.linear_speed 
			ang = 0.0 
		elif key == 's':
			lin =- self.linear_speed 
			ang = 0.0
		elif key == 'a':
			lin = 0.0 
			ang = self.angular_speed 
		elif key == 'd': 
			lin = 0.0 
			ang =- self.angular_speed 
		elif key == ' ':
			lin = 0.0 
			ang = 0.0 
		elif key == 'q': 
			self.get_logger().info("Quit key pressed.")
			self.running = False 
			#rclpy.shutdown()
			return 
		self.current_linear = lin 
		self.current_angular = ang
			
	

def main(args = None):
	rclpy.init(args = args)
	node = TurtleKeyboardTeleop()
	try: 
		rclpy.spin(node)
	except KeyboardInterrupt: 
		pass
	finally: 
		node.get_logger().info("Shutting down keyboard teleop.")
		node.running = False
		node.destroy_node()
		rclpy.shutdown()
		
if __name__ == '__main__':
	main()
	
	
		
