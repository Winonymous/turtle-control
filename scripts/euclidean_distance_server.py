#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from example_interfaces.srv import AddTwoInts
import math
import numpy as np



class Server(Node):
    x_1 = x_2 = y_1 = y_2 = 0.0

    def __init__(self):
        super().__init__("euclidean_distance_server")
        pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback,  10)
        server_ = self.create_service(AddTwoInts, "dist_two_point", self.get_distance)
        self.get_logger().info("Ready to get distance between two points")
    
    def pose_callback(self, turtle_pose):
        self.x_1 = turtle_pose.x
        self.y_1 = turtle_pose.y

    def get_distance(self, request, response):
        self.x_2 = request.a
        self.y_2 = request.b

        response.sum = int(np.sqrt(np.power(self.x_2 - self.x_1, 2) + np.power(self.y_2 - self.y_1, 2)))

        self.get_logger().info("Incoming coordinates  \n({}, {}), ({}, {})".format(self.x_1, self.y_1, self.x_2, self.y_2))
        self.get_logger().info("Sending back response : {}".format(10))

        return response

def main(args = None):
    rclpy.init(args=args)

    dist_server = Server()

    rclpy.spin(dist_server)
    rclpy.shutdown()

if __name__ == "__main__":
    main()