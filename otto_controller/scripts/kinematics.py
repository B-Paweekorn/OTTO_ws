#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState

class RobotKinematics(Node):
    def __init__(self):
        super().__init__('kinematics')
        self.create_subscription(JointState,'/joint_states',self.feedback_callback,10)

        self.l1 = 0.2
        self.l2 = 0.2
        self.joint_state = {}

    def forward_kinematics(self, q1, q2):
        x = self.l1*np.cos(np.deg2rad(45) - q1) - self.l2*np.cos(np.deg2rad(45) + q2 + q1) + 0.03
        z = self.l1*np.sin(np.deg2rad(45) - q1) + self.l2*np.sin(np.deg2rad(45) + q2 + q1)
        return x,z

    def inv_jacobian(self, ):
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
