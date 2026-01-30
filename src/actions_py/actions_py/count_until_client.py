#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient 
from rclpy.action.client import ClientGoalHandle, GoalStatus 
from my_robot_interfaces.action import CountUntil

class CountUntilClientNode(Node):
    def __init__(self):
        super().__init__("count_until_client")

        # Create the action client 
        # This object handles the communication with the action server 
        # Arguments: (self, ActionType, action_topic_name)
        self.count_until_client_ = ActionClient(self, CountUntil, "count_until")
        
    def send_goal(self, target_number, period):
        # Wait for the server 
        self.count_until_client_.wait_for_server()

        # Create a goal object based on the .action definition
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        # Send the goal 
        self.get_logger().info("Sending goal")
        self.count_until_client_. \
            send_goal_async(goal, feedback_callback=self.goal_feedback_callback).\
                add_done_callback(self.goal_response_callback)
        
        # Send a cancel request 2 seconds later 
        # self.timer_ = self.create_timer(2.0, self.cancel_goal)

    def cancel_goal(self):
        self.get_logger().info("Sending cancel request")
        self.goal_handle.cancel_goal_async()
        self.timer_.cancel()  # Cancel the timer to avoid multiple calls 
        
    def goal_response_callback(self, future):
        self.goal_handle: ClientGoalHandle = future.result()
        if self.goal_handle.accepted: 
            self.get_logger().info("Goal accepted")
            self.goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
        else: 
            self.get_logger().warn("Goal rejected")
        
    def goal_result_callback(self, future):
        status = future.result().status 
        result = future.result().result 
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Goal was aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal was canceled")
        self.get_logger().info("Result:" + str(result.reached_number))

    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_number
        self.get_logger().info("Feedback received: " + str(number))


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode()
    node.send_goal(6, 1.0)
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()
