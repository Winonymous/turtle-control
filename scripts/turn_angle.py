#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist

from math import radians
import time

class TurnAngle(Node):
    def __init__(self):
        super().__init__("turn_angle") # type: ignore
        self.speed_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.twist = Twist()

        
    def call(self, angular_velocity, angle, direction):
        self.reset_turtle()
        angle = 2 * radians(angle)
        timetaken = angle/self.twist.angular.z   # Turn 120 degrees
        
        if direction == 1:
            self.twist.angular.z = -1 * self.twist.angular.z
        
        self.speed_pub_.publish(self.twist)
        time.sleep(timetaken)  # Sleep for 2 seconds to complete each leg of the triangle


    def reset_turtle(self):
        # Stop the turtle after drawing
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.speed_pub_.publish(self.twist)
    
def main(args = None):
    rclpy.init(args=args)

    ang_velocity = int(input("Input angular Velocity: "))
    angle = int(input("Turn Angle: "))
    clockwise = int(input("Clockwise 0, Anti-clockwise 1: "))

    turner = TurnAngle()

    turner.call(ang_velocity, angle, clockwise)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


