#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState

class RobotKinematics(Node):
    def __init__(self):
        super().__init__('kinematics')
        self.dt = 0.001
        
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.create_subscription(JointState,'/joint_states',self.feedback_callback,10)
        self.create_subscription()
        self.l1 = 0.2
        self.l2 = 0.2

        self.kp = 1
        self.targ = [0.0] * 4
        self.q_de = [0.0] * 4
        self.joint_state = {}
    def timer_callback(self):
        self.q_de += self.controller([self.joint_state["q"][0], self.joint_state["q"][1], self.joint_state["q"][3], self.joint_state["q"][4]]) * self.dt

    def forward_kinematics(self, q1, q2):
        x = -self.l1*np.cos(np.deg2rad(45) - q1) + self.l2*np.cos(np.deg2rad(45) + q2 + q1)
        z = -self.l1*np.sin(np.deg2rad(45) - q1) - self.l2*np.sin(np.deg2rad(45) + q2 + q1)
        return x,z

    def fnc_jacobian(self, q1, q2, q3, q4):
        J = np.array([[-self.l1*np.sin(np.deg2rad(45) - q1) + self.l2*np.sin(np.deg2rad(45) + q1 + q2), self.l2*np.sin(np.deg2rad(45) + q1 + q2)], 
                      [self.l1*np.cos(np.deg2rad(45) - q1) + self.l2*np.cos(np.deg2rad(45) + q1 + q2), self.l2*np.cos(np.deg2rad(45) + q1 + q2)],
                      [-self.l1*np.sin(np.deg2rad(45) - q3) + self.l2*np.sin(np.deg2rad(45) + q3 + q4), self.l2*np.sin(np.deg2rad(45) + q3 + q4)], 
                      [self.l1*np.cos(np.deg2rad(45) - q3) + self.l2*np.cos(np.deg2rad(45) + q3 + q4), self.l2*np.cos(np.deg2rad(45) + q3 + q4)]])
        J_inv = np.linalg.inv(J)
        return J, J_inv
    
    def controller(self, curr):
        if abs(np.linalg.det(self.fnc_jacobian()[0])) < 0.01:
            print("Singularlity!!")
            return np.array([0, 0])
        err = self.targ - curr # 1 X 4
        v = err * self.kp
        q_dot = np.matmul(self.fnc_jacobian()[1], np.transpose(v))
        return q_dot

    def integrate(self, ):
        pass

    def feedback_callback(self, msg):
        joint_names = msg.name
        joint_q = msg.position

        order = [
            'knee_jointL', 'virtualhip_jointL', 'wheel_jointL',
            'knee_jointR', 'virtualhip_jointR', 'wheel_jointR'
        ]

        # Build the joint_data dictionary
        joint_data = {name: q for name, q in zip(joint_names, joint_q)}
        self.joint_state = {
            'names': order,
            'q': [joint_data[joint] for joint in order],
        }
        print(self.forward_kinematics(self.joint_state["q"][0], self.joint_state["q"][1]))

def main(args=None):
    rclpy.init(args=args)
    node = RobotKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
