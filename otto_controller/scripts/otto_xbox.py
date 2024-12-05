#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float64MultiArray

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.leg_cmd = self.create_publisher(Float64MultiArray, '/legcmd', 10)

        self.twist = Twist()
        self.v_max = 1.5
        self.w_max = 2

        self.max_height = -0.28
        self.biped = Float64MultiArray()
        self.biped.data = [0.0, -0.28, 0.0, -0.28]
    def joy_callback(self, msg):
        self.twist.linear.x = msg.axes[1] * self.v_max
        self.twist.angular.z = msg.axes[2] * self.w_max
        self.pub_cmd.publish(self.twist)

        self.biped.data[1] = (msg.axes[5] + 1)*0.5 * self.max_height
        self.biped.data[3] = (msg.axes[4] + 1)*0.5 * self.max_height
        
        if self.biped.data[1] > -0.18:
            self.biped.data[1] = -0.18
        if self.biped.data[3] > -0.18:
            self.biped.data[3] = -0.18
        self.leg_cmd.publish(self.biped)

def main(args=None):
    rclpy.init(args=args)
    joy_control = JoyControl()
    rclpy.spin(joy_control)
    joy_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()