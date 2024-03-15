#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.srv import Spawn, Kill

from turtle_control.action import MovePosition
from geometry_msgs.msg import Pose


from random import randint

class turtle_catcher(Node):
    x_1 = x_2 = y_1 = y_2 = 0.0

    def __init__(self):
        super().__init__("goal_action_server")

        self.turtle1pose = self.create_subscription(Pose, "/turtle1/pose", self.turtle1_pose_callback,  10)
        self.turtle2pose = self.create_subscription(Pose, "/turtle2/pose", self.turtle2_pose_callback,  10)
        
        self.create_turtle = self.create_client(Spawn, "/spawn")
        self.kill_turtle_client = self.create_client(Kill, "/kill")

        self.move_client = ActionClient(self,
                                           MovePosition,
                                           "positon_move_action")
        
        self.random_turtle()
    
    def turtle1_pose_callback(self, turtle_pose):
        self.x_1 = turtle_pose.x
        self.y_1 = turtle_pose.y

        self.get_logger().info("Failed to call service {} {} {} {}".format(self.x_1, self.x_2, self.y_1, self.y_2))
        if ((round(self.x_1) == round(self.x_2)) and (round(self.y_1) == round(self.y_2))):
            self.random_turtle()   

    def kill_turtle(self):
        self.get_logger().info('Creating turtle')
        
        request = Kill.Request()
        request.name = "catch"

        service_future = self.kill_turtle_client.call_async(request)

        service_future.add_done_callback(self.create_new_turtle)

    def create_new_turtle(self, future):
        result = future.result()

        self.get_logger().info('Kill result: {}'.format(result))
        self.random_turtle()


    def turtle2_pose_callback(self, turtle_pose):
        self.x_2 = turtle_pose.x
        self.y_2 = turtle_pose.y

    def random_turtle(self):
        self.request = Spawn.Request()

        self.request.x = float(randint(1, 10))
        self.request.y = float(randint(1, 10))
        self.request.name = "catch"
        self.result = self.create_turtle.call_async(request=self.request)
        
        self.result.add_done_callback(self.turtlespawned)

    def turtlespawned(self, future):
        goal_handle = future.result()
        self.get_logger().info('Spawned = {}'.format(goal_handle))

        self.move_to_turtle(self.request.x, self.request.y)
        
    def move_to_turtle(self, x, y):
        goal = MovePosition.Goal()
        goal.x = int(x)
        goal.y = int(y)

        self.move_client.wait_for_server()

        self._send_goal_future =  self.move_client.send_goal_async(goal)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result 
        self.get_logger().info('Result: {0}'.format(result))
        self.kill_turtle()
        # rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    Catcher = turtle_catcher()
    rclpy.spin(Catcher)

if __name__ == "__main__":
    main() 