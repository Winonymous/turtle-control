#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtle_control.msg import MessageCounter # type: ignore

class Counter(Node):
    msg_counter = MessageCounter()
    msg_counter.count = 0
    msg_counter.state = False
    msg_counter.message = "Robot"

    def __init__(self):
        super().__init__("message_counter") # type: ignore
        self.publisher_ = self.create_publisher(MessageCounter, 'count_number', 10)    

        self.timer = self.create_timer(1, self.pub_count)

    def pub_count(self):
        print("Counter is " , self.msg_counter.count)
        print("State is ", self.msg_counter.state)

        self.publisher_.publish(self.msg_counter)

        if(self.msg_counter.count == 10):
            self.msg_counter.count = 0
            self.msg_counter.state = not self.msg_counter.state

            print("Hey Robot: Starting Again")
            print("State Changed")
        
        self.msg_counter.count+=1

def main(args = None):
    rclpy.init(args = args)
    counter = Counter()
    rclpy.spin(counter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()