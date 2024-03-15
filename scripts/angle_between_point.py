#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from math import atan, degrees, pi
from turtle_control.srv import AngleBetweenPoint
 
class Server(Node):
    x_1 = x_2 = y_1 = y_2 = 0.0

    def __init__(self):
        super().__init__("angle_point_server")
        
        pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback,  10)
        server_ = self.create_service(AngleBetweenPoint, "angle_two_point", self.get_angle)
        self.get_logger().info("Ready to get angle between two points")

    def pose_callback(self, turtle_pose):
        self.x_1 = turtle_pose.x
        self.y_1 = turtle_pose.y
    
    def get_angle(self, request, response):
        self.x_2 = request.x
        self.y_2 = request.y

        adj = self.x_2 - self.x_1
        opp = self.y_2 - self.y_1

        self.get_logger().info("Incoming coordinates  \n({}, {}), ({})".format(adj, opp, opp/adj))

        angle = atan(opp/adj)

        if (adj < 0):
            response.angle = pi + angle
        elif((adj > 0) and (opp < 0)):
            response.angle = (2 * pi) + angle
        else:
            response.angle = angle
            
        if adj < 0:
            response.hyp = False
        else:
            response.hyp = True

        if opp < 0:
            response.opp = False
        else:
            response.opp = True

        self.get_logger().info("Incoming coordinates  \n({}, {}), ({}, {})".format(self.x_1, self.y_1, self.x_2, self.y_2))
        self.get_logger().info("Sending angle : {}".format(degrees(response.angle)))

        return response
    
def main(args = None):
    rclpy.init(args=args)
    angle_server = Server()
    rclpy.spin(angle_server)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
