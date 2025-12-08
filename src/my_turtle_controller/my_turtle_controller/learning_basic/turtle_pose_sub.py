import rclpy 
from rclpy.node import Node 
from turtlesim.msg import Pose 

class TurtlePoseSubscriber(Node): 
	def __init__(self):
		super().__init__('turtle_pose_subscriber')
		
		self.subscription = self.create_subscription(
			Pose,   # message type
			'/turtle1/pose', # topic
			self.pose_callback,  #callback founction
			10   # Qos queue 
		) 
		self.subscription
		
		self.get_logger().info("Turtle pose subscriber started.")
		
	def pose_callback(self, msg: Pose): 
		"""
		when /turtle1/pose has new message, this founction will be used 
		"""

		self.get_logger().info(
			f"x={msg.x:.2f}, y={msg.y:.2f},"
			f"theta={msg.theta:.2f}, "
			f"linear={msg.linear_velocity:.2f}, "
			f"angular={msg.angular_velocity:.2f} "
		)
	
def main(args=None): 
	rclpy.init(args=args)
	node = TurtlePoseSubscriber()
	try: 
		rclpy.spin(node)
	except KeyboardInterrupt: 
		pass
	finally: 
		node.destroy_node 
		rclpy.shutdown()
			
if __name__ == '__main__': 
	main()
