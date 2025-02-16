#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
class RobotKinematics(Node):
    def __init__(self):
        super().__init__('kinematics')
        self.dt = 0.0005
        
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.create_subscription(JointState,'/joint_states',self.feedback_callback,10)
        self.create_subscription(Float64MultiArray, '/legpose', self.legpose_callback, 10)
        self.pos_cmd_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        self.l1 = 0.2
        self.l2 = 0.2

        self.kp = 20.0
        self.targL = [0.0, -0.28]
        self.targR = [0.0, -0.28]
        self.prev_targ = self.targL + self.targR
        self.joint_state = {}
        self.pos_cmd_msg = Float64MultiArray()

        self.qL = [0.0, 0.0]
        self.qR = [0.0, 0.0]

        self.q1L_des = 0.0
        self.q2L_des = 0.0

        self.q1R_des = 0.0
        self.q2R_des = 0.0

    def timer_callback(self):
        try:
            qLd_des = self.controller(np.array([self.joint_state["q"][0], self.joint_state["q"][1]]), np.array(self.targL))
            self.q1L_des += qLd_des[0] * self.dt
            self.q2L_des += qLd_des[1] * self.dt

            qRd_des = self.controller(np.array([self.joint_state["q"][3], self.joint_state["q"][4]]), np.array(self.targR))
            self.q1R_des += qRd_des[0] * self.dt
            self.q2R_des += qRd_des[1] * self.dt

            self.pos_cmd_msg.data = [self.q1L_des, self.q2L_des, self.q1R_des, self.q2R_des]
            # self.pos_cmd_msg.data = [q1L_des*0, q2L_des*0, 0.0, 0.0]

            self.pos_cmd_pub.publish(self.pos_cmd_msg)
        except (TypeError, IndexError, KeyError) as e:
            self.get_logger().error(f"[ERROR]{e}")

    def forward_kinematics(self, q):
        x = -self.l1*np.cos(np.deg2rad(45) - q[0]) + self.l2*np.cos(np.deg2rad(45) + q[1] + q[0])
        z = -self.l1*np.sin(np.deg2rad(45) - q[0]) - self.l2*np.sin(np.deg2rad(45) + q[1] + q[0])
        return np.array([x,z])

    def fnc_jacobian(self, q):
        J = np.array([[-self.l1*np.sin(np.deg2rad(45)-q[0]) - self.l2*np.sin(np.deg2rad(45)+q[0]+q[1]), -self.l2*np.sin(np.deg2rad(45)+q[0]+q[1])], 
                      [self.l1*np.cos(np.deg2rad(45)-q[0]) - self.l2*np.cos(np.deg2rad(45)+q[0]+q[1]), -self.l2*np.cos(np.deg2rad(45)+q[0]+q[1])]])

        J_inv = np.linalg.inv(J)
        return J, J_inv
    
    def controller(self, curr, targ):
        if abs(np.linalg.det(self.fnc_jacobian(curr)[0])) < 0.0001:
            print("Singularlity!!")
        err = targ - self.forward_kinematics(curr) # 1 x 2
        
        v = np.where(err < 0, err * 30, err * 2.5)


        q_dot = np.matmul(self.fnc_jacobian(curr)[1], np.transpose(v))
        return q_dot # 2 x 1

    def legpose_callback(self, msg):
        self.targL[0] = msg.data[0]
        self.targL[1] = msg.data[1]
        self.targR[0] = msg.data[2]
        self.targR[1] = msg.data[3]
        if self.targL + self.targR != self.prev_targ:
            print("received new target")
            self.prev_targ = self.targL + self.targR
            self.qL = [self.joint_state["q"][0], self.joint_state["q"][1]]
            self.qR = [self.joint_state["q"][3], self.joint_state["q"][4]]

    def feedback_callback(self, msg):
        joint_names = msg.name
        joint_q = msg.position
        joint_qd = msg.velocity
        joint_eff = msg.effort

        order = [
            'knee_jointL', 'virtualhip_jointL', 'wheel_jointL',
            'knee_jointR', 'virtualhip_jointR', 'wheel_jointR'
        ]

        joint_data = {
            name: (q, qd, effort)
            for name, q, qd, effort in zip(joint_names, joint_q, joint_qd, joint_eff)
        }

        self.joint_state = {
            'names': order,
            'q': [joint_data[joint][0] for joint in order],
            'qd': [joint_data[joint][1] for joint in order],
            'efforts': [joint_data[joint][2] for joint in order]
        }
        # try:
        #     print(self.forward_kinematics(self.joint_state["q"][0], self.joint_state["q"][1]))
        # except (TypeError, IndexError, KeyError) as e:
        #     self.get_logger().error(f"[ERROR]{e}")
def main(args=None):
    rclpy.init(args=args)
    node = RobotKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
