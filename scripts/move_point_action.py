#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from turtle_control.srv import AngleBetweenPoint
from turtle_control.action import MovePosition
from example_interfaces.srv import AddTwoInts


from math import radians, degrees, pow, atan, pi
import time
import numpy as np

class MoveServer(Node):
    x_1 = x_2 = y_1 = y_2 = 0.0

    ang_velocity = 2.0

    def __init__(self):
        super().__init__("move_point_action") # type: ignore
        
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.twist = Twist()

        self.speed_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.action_server_ = ActionServer(
            self,
            MovePosition,
            "positon_move_action",
            self.execute_callback)
        
        self.pose_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback,  10)
        
    def pose_callback(self, turtle_pose):
        self.x_1 = turtle_pose.x
        self.y_1 = turtle_pose.y
        self.angle = turtle_pose.theta

    def execute_callback(self, handle_goal):
        self.get_logger().info("Executing goal")

        self.x_2 = handle_goal.request.x
        self.y_2 = handle_goal.request.y

        angle = self.get_angle(self.x_2, self.y_2)

        self.speed = Twist()

        self.get_logger().info("Turning")
        feedback_msg = MovePosition.Feedback()
        feedback_msg.feedback = "Turning"
        handle_goal.publish_feedback(feedback_msg)

        self.turn(angle)

        self.get_logger().info("Moving Forward")

        distance = self.get_distance()
        self.forward(distance)
        
        feedback_msg = MovePosition.Feedback()
        feedback_msg.feedback = "Moving Forward"
        handle_goal.publish_feedback(feedback_msg)

        handle_goal.succeed()
        
        result = MovePosition.Result()
        result.status = "Finished action server"

        return result

    def turn(self, angle):

        speed = 0.3
        angle = angle - self.angle

        is_forward = True
    

        if angle > 360:
            angle = 360 - angle

        if is_forward:
            self.speed.angular.z = abs(speed)
        else:
            self.speed.angular.z = -1 * abs(speed)

        self.speed.linear.x = 0.0
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0

        time_ = self.get_clock().now().seconds_nanoseconds()
        t0 = time_[0] + ((time_[1]) / pow(10, 9))
        current_angle = 0.0

        self.get_logger().info(f"Current time_: {t0}")
        self.get_logger().info(f"Current angle: {degrees(current_angle)}")
        self.get_logger().info(f"Desired angle: {degrees(angle)}")
    

        while (current_angle < angle)  and (rclpy.ok() is True):
            self.speed_pub_.publish(self.speed)
            time.sleep(0.1)
            self.get_logger().info(f"Current angle: {degrees(current_angle)}")
            self.get_logger().info(f"Desired angle: {degrees(angle)}")

            time_ = self.get_clock().now().seconds_nanoseconds()
            t1 = time_[0] + ((time_[1]) / pow(10, 9))

            if (t1 - t0) > 0:
                current_angle = self.speed.angular.z * ((t1 - t0))
            else:
                current_angle = 0

        
        self.speed.angular.z = 0.0

        self.pub.publish(self.speed)
        self.get_logger().info(f"Current angle: {degrees(current_angle)}")
        self.get_logger().info(f"Desired angle: {degrees(angle)}")
        self.get_logger().info(f"Done Turning")


    def forward(self, distance):
        speed = 2.0
        is_forward = True

        if is_forward:
            self.speed.linear.x = abs(speed)
        else:
            self.speed.linear.x = -1 * abs(speed)

        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0
        self.speed.angular.z = 0.0

        time_ = self.get_clock().now().seconds_nanoseconds()
        t0 = time_[0]
        current_dist = 0.0

        self.get_logger().info(f"Current Time: {time_}")
        self.get_logger().info(f"Current dist: {current_dist}")
        self.get_logger().info(f"Desired dist: {distance}")
        
        self.speed_pub_.publish(self.speed)

        while (int(current_dist) < int(distance))  and (rclpy.ok() is True):
            self.speed_pub_.publish(self.speed)
            
            time_ = self.get_clock().now().seconds_nanoseconds()
            t1 = time_[0]

            if (t1 - t0) > 0:
                current_dist = self.speed.linear.x * (t1 - t0)
            else:
                current_dist = 0            

            time.sleep(0.05)
        
        self.get_logger().info(f"Current Time: {time_}")
        self.get_logger().info(f"Current dist: {current_dist}")
        self.get_logger().info(f"Desired dist: {distance}")

        self.speed.linear.x = 0.0

        self.pub.publish(self.speed)

    def get_distance(self):
       self.get_logger().info("{} {} {} {}".format(self.x_2, self.x_1, self.y_2, self.y_1))
       distance = np.sqrt(np.power((self.x_2 - self.x_1), 2) + np.power((self.y_2 - self.y_1), 2)) + 0.6
       self.get_logger().info(f"distance {distance}")
       return distance

        
    def get_angle(self, x, y):
        self.x_2 = x
        self.y_2 = y

        adj = self.x_2 - self.x_1
        opp = self.y_2 - self.y_1

        if((self.x_2 == self.x_1) or (self.y_2  == self.y_1)):
            angle = 0
            return angle
        else:

            self.get_logger().info("Incoming coordinates  \n({}, {}), ({})".format(adj, opp, opp/adj))

            angle = atan(opp/adj)

            if (adj < 0):
                angle = pi + angle
            elif((adj > 0) and (opp < 0)):
                angle = (2 * pi) + angle
            else:
                angle = angle

            self.get_logger().info("Incoming coordinates  \n({}, {}), ({}, {})".format(self.x_1, self.y_1, self.x_2, self.y_2))
            self.get_logger().info("Sending angle : {}".format(degrees(angle)))

            return angle

    
def main(args = None):
    rclpy.init(args=args)
    turner = MoveServer()
    rclpy.spin(turner)
    # rclpy.shutdown()

if __name__ == '__main__':
    main()


