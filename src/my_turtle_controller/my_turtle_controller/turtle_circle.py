import rclpy #python client library for ROS2
from rclpy.node import Node
from geometry_msgs.msg import Twist #import specific message type used to send velocity commands
import time
import math

class TurtleCircle(Node): 
#inherits from rclpy.node.Node

    #constructor, run when TurtuleCircle object is created
    def __init__(self): 
    
    	#call paretn Node class's constructor, naming our ROS node 'turtle_circle'
    	
        super().__init__('turtle_circle') 
	
	#it creates a publisher that sends message of type "Twist" to the ROS topic named '/turle1/cmd_vel'
	#topic: think of a topic as a named chaneel. the 'turlesim' program listens on this specific channel for instructions;
	# Queue size (10): quality of service setting that limits the number of messages held in a queue if the network gets busy. 
	
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    # Defines the function that executes the movement logic
    def move_circle(self): 
    
    	#creates an empty "Twist" message object
        msg = Twist()

        #change the velocit of robot
        msg.linear.x = 4.0      # change the size of circle, bigger circle bigger number (m/s)
        msg.angular.z = 2.0     # turning speed, (rad/s)

        # time required for a robot to complete a full 360-degree rotation
        duration = 2 * math.pi / abs(msg.angular.z)
        
        #recrods the current system time to track how long the movement loop has been running
        start_time = time.time()

        #loop, stop when complete one rotation
        while time.time() - start_time < duration:
        
            #Sends the Twist message to the turtlesim node
            self.publisher_.publish(msg)
            
            #pauses the python script for 0.05 seconds. contorls the publishing frequency. 
            time.sleep(0.05)   # 20Hz 

        # stop
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):

    #Initializes the ROS2 client library. this handshake is required before creating nodes
    rclpy.init(args=args)
    
    #creates an instance of our TurtleCircle class, which starts the ROS node
    node = TurtleCircle()
    
    
    time.sleep(1.0)   # wait untill turtlesim is ready
    
    #calls the function that starts the circular movement logic
    node.move_circle()
    
    rclpy.shutdown()

#Standard python entry point to run the main function
if __name__ == '__main__':
    main()


