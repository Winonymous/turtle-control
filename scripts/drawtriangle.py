#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from example_interfaces.srv import AddTwoInts

from math import pi, radians
import time

class drawTriangle(Node):
    def __init__(self):
        super().__init__("drawTriangle")

        self.speed_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.twist = Twist()
        self.draw_triangle()


    def draw_triangle(self):
        for _ in range(3):
            self.turn(120)
            self.forward(int(sys.argv[1]))
        
    def turn(self, angle):
        self.reset_turtle()
        angle = 2 * radians(angle)
        timetaken = 2.0
        self.twist.angular.z = angle/timetaken  # Turn 120 degrees
        self.speed_pub_.publish(self.twist)
        time.sleep(timetaken)  # Sleep for 2 seconds to complete each leg of the triangle
        
    def forward(self, distance):
        self.reset_turtle()
        timetaken = 2.0
        self.twist.linear.x = distance/timetaken  # Move forward
        self.speed_pub_.publish(self.twist)
        time.sleep(timetaken)  # Sleep for 2 seconds to complete each leg of the triangle        

    def reset_turtle(self):
        # Stop the turtle after drawing
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.speed_pub_.publish(self.twist)

def main(args = None):
    rclpy.init(args=args)

    draw_triangle = drawTriangle()
    # rclpy.spin_once(draw_triangle)

    rclpy.shutdown()

if __name__ == "__main__":
    main()