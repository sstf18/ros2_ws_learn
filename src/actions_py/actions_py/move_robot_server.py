#!/usr/bin/env python3
import rclpy
import time 
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import MoveRobot
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup   

class MoveRobotServerNode(Node):
    def __init__(self):
        super().__init__("move_robot_server")
        self.goal_handle_: ServerGoalHandle = None

        # Threading Lock, since we use multi-threading, we need to protect shared variables
        # (like self.goal_handle_) from being accessed by two processes at the same time
        self.goal_lock_ = threading.Lock()
        # self.goal_queue_ = []

        self.robot_position_ = 50
        # Action server setup 
        self.move_robot_server_ = ActionServer(
            self, 
            MoveRobot, 
            "move_robot",
            goal_callback= self.goal_callback, 
            #handle_accepted_callback= self.handle_accepted_callback,
            cancel_callback= self.cancel_callback,
            execute_callback = self.execute_callback, 
            # ROS2 callbacks mutually exclusive( one waits for another). 
            # we use 'Reentrant' so that the 'cancel_callback' can be processed 
            # WHILE the 'execute_callback' is running its long couning loop. 
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Action Server has been started.")   
        self.get_logger().info("Robot position: " + str(self.robot_position_))

    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Reveived a goal")

        # Validate the goal request 
        if goal_request.position not in range(0, 100) or goal_request.velocity <= 0:            
            self.get_logger().warn("Invalid position/velocity, Rejected goal")
            return GoalResponse.REJECT
        
        # New goal is valid, abort previous goal and accept new goal 
        if self.goal_handle_ is not None and self.goal_handle_.is_active:
            self.get_logger().info("Aborting previous goal")
            self.goal_handle_.abort()

        self.get_logger().info("Accepted goal request")
        return GoalResponse.ACCEPT

    # def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
    #     with self.goal_lock_: 
    #         if self.goal_handle_ is not None: 
    #             self.goal_queue_.append(goal_handle)
    #         else: 
    #             goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT # or REJECT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # Store the handle safely
        with self.goal_lock_:
            self.goal_handle_= goal_handle

        # Get request from goal 
        goal_position = goal_handle.request.position
        velocity = goal_handle.request.velocity

        # Execute the action 
        self.get_logger().info(f"Executing goal: Moving to position {goal_position} with velocity {velocity}.")

        result = MoveRobot.Result() 
        feedback = MoveRobot.Feedback()

        while rclpy.ok(): 
            if not goal_handle.is_active: 
                result.position = self.robot_position_
                result.message = "Preempted by another goal"
                return result 
            
            if goal_handle.is_cancel_requested:
                result.position = self.robot_position_
                if goal_position == self.robot_position_: 
                    result.message = "Success" 
                    goal_handle.succeed() 
                else: 
                    result.message = "Canceled" 
                    goal_handle.canceled()
                return result

            diff = goal_position - self.robot_position_

            if diff == 0: 
                result.position = self.robot_position_
                result.message = "Reached goal position" 
                goal_handle.succeed()
                return result 
            elif diff > 0: 
                if diff >= velocity:
                    self.robot_position_ += velocity
                else: 
                    self.robot_position_ += diff 
            else: 
                if abs(diff) >= velocity:
                    self.robot_position_ -= velocity
                else:
                    self.robot_position_ -= abs(diff)
            
            self.get_logger().info("Robot position: " + str(self.robot_position_))
            feedback.current_position = self.robot_position_
            goal_handle.publish_feedback(feedback)

            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServerNode()
    # We use MultiThreadedExecutor to allow the "ReentrantCallbackGroup" to worl properly "
    # the spawns a pool of threads to process callbacks can run parallel. 
    rclpy.spin(node, MultiThreadedExecutor())
    #rclpy.spin(node)    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
