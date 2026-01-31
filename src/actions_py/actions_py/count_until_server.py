#!/usr/bin/env python3
import rclpy
import time 
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup    

class CountUntilServer(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle_: ServerGoalHandle = None

        # Threading Lock, since we use multi-threading, we need to protect shared variables
        # (like self.goal_handle_) from being accessed by two processes at the same time
        self.goal_lock_ = threading.Lock()

        self.goal_queue_ = []
        # Action server setup 
        self.count_until_server_ = ActionServer(
            self, 
            CountUntil, 
            "count_until",
            goal_callback= self.goal_callback, 
            handle_accepted_callback= self.handle_accepted_callback,
            cancel_callback= self.cancel_callback,
            execute_callback = self.execute_callback, 
            # ROS2 callbacks mutually exclusive( one waits for another). 
            # we use 'Reentrant' so that the 'cancel_callback' can be processed 
            # WHILE the 'execute_callback' is running its long couning loop. 
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Action Server has been started.")

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Reveived a goal")

        #Policy: refuse new goal if current goal still active
        # Check if there is already a goal AND if it is currently active. 
        # with self.goal_lock_:  
        #   if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #     self.get_logger().info("A goal is already active, rejecting new goal")
        #     return GoalResponse.REJECT 


        
        # Validate the goal request 
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejected goal request")
            return GoalResponse.REJECT
        
        # Policy: preempt existing goal when receiving new goal
        # with self.goal_lock_: 
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().info("Abort the current goal and accept new goal")
        #         self.goal_handle_.abort()

        self.get_logger().info("Accepted goal request")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_: 
            if self.goal_handle_ is not None: 
                self.goal_queue_.append(goal_handle)
            else: 
                goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT # or REJECT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # Store the handle safely
        with self.goal_lock_:
            self.goal_handle_= goal_handle

        # Get request from goal 
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        # Execute the action 
        self.get_logger().info(f"Executing goal: Counting until {target_number} with period {period} seconds.")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0 
        for i in range(target_number): 
            if not goal_handle.is_active: 
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result 
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Canceling the goal")
                goal_handle.canceled()
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            counter += 1 
            self.get_logger().info(str(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)   

        # Once done, set goal final state 
        goal_handle.succeed()

        # and send the result 
        result.reached_number = counter
        self.process_next_goal_in_queue()
        return result 
    
    def process_next_goal_in_queue(self):
        with self.goal_lock_: 
            if len(self.goal_queue_) > 0: 
                self.goal_queue_.pop(0).execute()
            else: 
                self.goal_handle_ = None

def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServer()
    # We use MultiThreadedExecutor to allow the "ReentrantCallbackGroup" to worl properly "
    # the spawns a pool of threads to process callbacks can run parallel. 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
