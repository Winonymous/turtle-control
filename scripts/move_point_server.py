#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from geometry_msgs.msg import Twist
from turtle_control.action import Move

import time

class MoveActionServer(Node):
    def __init__(self):
        super().__init__("goal_action_server")

        self.action_server_ = ActionServer(
            self,
            Move,
            "action_server",
            self.execute_callback)
        
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def execute_callback(self, handle_goal):
        self.get_logger().info("Executing goal")

        speed = 2.0
        speed_msg = Twist()
        is_forward = True

        if is_forward:
            speed_msg.linear.x = abs(speed)
        else:
            speed_msg.linear.x = -1 * abs(speed)

        speed_msg.linear.y = 0.0
        speed_msg.linear.z = 0.0
        speed_msg.angular.x = 0.0
        speed_msg.angular.y = 0.0
        speed_msg.angular.z = 0.0

        time_ = self.get_clock().now().seconds_nanoseconds()
        t0 = time_[0]
        current_dist = 0.0

        feedback_msg = Move.Feedback()
        feedback_msg.feedback = "Moving Forward"

        self.get_logger().info(f"Current dist: {current_dist}")
        self.get_logger().info(f"Desired dist: {handle_goal.request.desired_dist}")
        

        while (current_dist < handle_goal.request.desired_dist)  and (rclpy.ok() is True):
            time_ = self.get_clock().now().seconds_nanoseconds()
            t1 = time_[0]

            if (t1 - t0) > 0:
                current_dist = speed_msg.linear.x * (t1 - t0)
            else:
                current_dist = 0

            self.pub.publish(speed_msg)
            handle_goal.publish_feedback(feedback_msg)

            time.sleep(0.1)

        handle_goal.succeed()
        speed_msg.linear.x = 0.0

        self.pub.publish(speed_msg)

        result = Move.Result()
        result.status = "Finished action server"

        return result
    
def main(args = None):
    rclpy.init(args=args)
    move_action_server = MoveActionServer()
    move_action_server.get_logger().info("Action Server running")
    rclpy.spin(move_action_server)

if "__name__" == "__name__":
    main()