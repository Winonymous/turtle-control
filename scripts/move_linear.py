#!/usr/bin/env python3

import rclpy
import time

from math import pi
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveLinear(Node):

    def __init__(self):
        super().__init__("move_linear") # type: ignore
        self.speed_pub_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.prev_time = 0

    def move_distance(self, speed, distance):
        mov = Twist()

        mov.linear.x = float(speed)

        expected_time = distance / speed
        print(expected_time)

        start_time = int(time.time())
        
        while(self.time_gone(start_time) < expected_time):
            self.speed_pub_.publish(mov)
    
    def backward(self):
        rot = Twist()

        direction = pi
        angular_velocity = 10.0

        rot.angular.z = direction   
        rot.angular.x = angular_velocity
        
        self.speed_pub_.publish(rot)

        time.sleep(0.1)
        
    def time_gone(self, start):
        time_now = int(time.time())
        time_gone = time_now - start

        return time_gone
        
def main(args = None):
    rclpy.init(args=args)
    
    MvLinaer = MoveLinear()

    speed = int(input("Enter speed: "))
    distance = int(input("Enter Distance speed: "))
    direction = int(input("Enter direction (1-Forward, 0-Backward): "))
    
    if (direction == 0):
        MvLinaer.backward()

    MvLinaer.move_distance(speed, distance)

    rclpy.shutdown()

if __name__ == '__main__':
    main()