#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.srv import Spawn, Kill

from turtlesim.msg import Pose
from turtle_control.msg import Turtle2Available

from random import randint
from math import degrees

import time

class TurtleCatcherGameMaster(Node):
    x_1 = x_2 = y_1 = y_2 = 0.0
    def __init__(self):
        super().__init__("turtle_catcher_game_master")
        self.get_logger().info("Game Master Started")

        self.turtle2available_pub_ = self.create_publisher(Turtle2Available, "/turtleavailable", 10)
        self.turtle1pose = self.create_subscription(Pose, "/turtle1/pose", self.turtle1_pose_callback,  10)
        self.catchpose = self.create_subscription(Pose, "/catch/pose", self.catch_pose_callback,  10)


        self.create_turtle = self.create_client(Spawn, "/spawn")
        self.kill_turtle_client = self.create_client(Kill, "/kill")

        self.random_turtle()

    def turtle1_pose_callback(self, turtle_pose):
        self.x_1 = turtle_pose.x
        self.y_1 = turtle_pose.y
        
        if self.x_2:
            if(round(self.x_1) == round(self.x_2) and round(self.y_1) == round(self.y_2)):
                isaval = Turtle2Available()
                isaval.isavailable = False
                self.kill_turtle()

                time.sleep(0.05)

    def kill_turtle(self):
            
            self.get_logger().info('Killing turtle')
            
            request = Kill.Request()
            request.name = "catch"

            self.x_2 = None
            self.y_2 = None

            service_future = self.kill_turtle_client.call_async(request)

            service_future.add_done_callback(self.create_new_turtle)

    
    def create_new_turtle(self, future):
        result = future.result()

        self.get_logger().info('Kill result: {}'.format(result))

        self.random_turtle()


    def catch_pose_callback(self, turtle_pose):
        self.x_2 = turtle_pose.x
        self.y_2 = turtle_pose.y

    def random_turtle(self): 
        isaval = Turtle2Available()
        isaval.isavailable = False
        self.turtle2available_pub_.publish(isaval)

        self.request = Spawn.Request()

        self.prev_x = randint(1, 10)
        self.prev_y = randint(1, 10)
        
        self.request.x = float(self.prev_x)
        self.request.y = float(self.prev_y)
        self.request.name = "catch"
        self.get_logger().info(f"Creating New Turtle at {self.request.x} {self.request.y}")

        self.result = self.create_turtle.call_async(request=self.request)
        
        self.result.add_done_callback(self.newturtlespawned)

        self.handleKill = True


    def newturtlespawned(self, future):
        result = future.result()
        self.get_logger().info('Result: {0}'.format(result))
        self.get_logger().info("New turtle Spawned")

        isaval = Turtle2Available()
        isaval.isavailable = True
        self.turtle2available_pub_.publish(isaval)
        

def main(args = None):
    rclpy.init(args=args)
    game_master = TurtleCatcherGameMaster()
    rclpy.spin(game_master)
    rclpy.shutdown()

if __name__ == "__main__":
    main()