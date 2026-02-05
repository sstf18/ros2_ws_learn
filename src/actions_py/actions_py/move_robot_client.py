#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient 
from rclpy.action.client import ClientGoalHandle, GoalStatus 
from my_robot_interfaces.action import MoveRobot
from example_interfaces.msg import Empty 

class MoveRobotClientNode(Node):
    def __init__(self):
        super().__init__("move_robot_client")
        self.goal_handle_ = None 
        # Create the action client 
        # This object handles the communication with the action server 
        # Arguments: (self, ActionType, action_topic_name)
        self.move_robot_client_ = ActionClient(self, MoveRobot, "move_robot")
        self.cancel_subscriber_ = self.create_subscription(
            Empty, "cancel_move", self.callback_cancel_move, 10) 
        
    def send_goal(self, position, velocity):
        # Wait for the server 
        self.move_robot_client_.wait_for_server()
        # Create a goal object based on the .action definition
        goal = MoveRobot.Goal()
        goal.position = position
        goal.velocity = velocity

        self.get_logger().info(f"Goal position: {position}, velocity: {velocity}")
        # Send the goal 
        self.get_logger().info("Sending goal")
        self.move_robot_client_. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback).\
                add_done_callback(self.goal_response_callback)
        
        # Send a cancel request 2 seconds later 
        # self.timer_ = self.create_timer(2.0, self.cancel_goal)
        
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted: 
            self.get_logger().info("Goal accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else: 
            self.get_logger().info("Goal rejected")
        
    def goal_result_callback(self, future):
        status = future.result().status 
        result = future.result().result 
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Goal was aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal was canceled")
        self.get_logger().info("Position:" + str(result.position)) 
        self.get_logger().info("Message: " + str(result.message))

    def goal_feedback_callback(self, feedback_msg):
        position = feedback_msg.feedback.current_position
        self.get_logger().info("Feedback received: " + str(position))

    def callback_cancel_move(self, msg):
        self.cancel_goal()

    def cancel_goal(self):
        if self.goal_handle_ is not None:
            self.get_logger().info("Send a cancel request")
            self.goal_handle_.cancel_goal_async()

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotClientNode()
    node.send_goal(76, 1)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
