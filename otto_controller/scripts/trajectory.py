#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('x_des_to_effort_node')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.effort_publisher = self.create_publisher(Twist,'/traj/cmd_vel',10)

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.cmd_vel_msg = Twist()


        self.traj_vx = 0.0
        self.traj_w = 0.0

        self.vmax = 3
        self.a = 3
        self.wmax = 3
        self.alpha = 4
        self.vx_d = 0.0
        self.w_d = 0.0

    def timer_callback(self):
        sig_vx = np.sign(self.vx_d - self.traj_vx)
        sig_w = np.sign(self.w_d - self.traj_w)

        if self.traj_vx != self.vx_d:
            self.traj_vx += sig_vx * self.a * self.dt
            if abs(self.traj_vx - self.vx_d) < 0.5:
                self.traj_vx = self.vx_d

        if self.traj_w != self.w_d:
            self.traj_w += sig_w * self.alpha * self.dt
            if abs(self.traj_w - self.w_d) < 0.5:
                self.traj_w = self.w_d

        self.cmd_vel_msg.linear.x = self.traj_vx
        self.cmd_vel_msg.angular.z = self.traj_w
        self.effort_publisher.publish(self.cmd_vel_msg)
        
        print(self.traj_vx, self.traj_w)

    def cmd_vel_callback(self, msg):
        self.vx_d = msg.linear.x
        self.w_d = msg.angular.z

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()
