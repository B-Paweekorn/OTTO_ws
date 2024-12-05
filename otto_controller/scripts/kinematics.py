#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import yaml
import signal
import time 
class RobotKinematics(Node):
    def __init__(self):
        super().__init__('kinematics')
        self.dt = 0.01
        
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.create_subscription(JointState,'/joint_states',self.feedback_callback,10)
        self.create_subscription(Float64MultiArray, '/legcmd', self.legpose_callback, 10)
        self.create_subscription(Float64MultiArray, 'robot_state', self.robot_state_callback, 10)
        self.pos_cmd_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.leg_pub = self.create_publisher(Float64MultiArray, '/legpose', 10)    
        self.l1 = 0.2
        self.l2 = 0.2

        self.kp = 15
        self.targL = [0.0, -0.28]
        self.targR = [0.0, -0.28]
        self.prev_targ = self.targL + self.targR
        self.joint_state = {}
        self.pos_cmd_msg = Float64MultiArray()
        self.leg_pos_msg = Float64MultiArray()
        self.qL = [0.0, 0.0]
        self.qR = [0.0, 0.0]

        self.q1L_des = 0.0
        self.q2L_des = 0.0
        self.q1R_des = 0.0
        self.q2R_des = 0.0

        self.targ = -0.28
        self.traj_leg = -0.28
        self.count = 0
        self.flag = 0
        self.time = 0

        self.data_to_save = []  # Initialize an empty list to store data
        
        self.x = 0
        self.theta = 0
        self.dtheta = 0
    def timer_callback(self):
        try:
            qLd_des = self.controller(np.array([self.joint_state["q"][0], self.joint_state["q"][1]]), np.array(self.targL))
            self.q1L_des += qLd_des[0] * self.dt
            self.q2L_des += qLd_des[1] * self.dt

            qRd_des = self.controller(np.array([self.joint_state["q"][3], self.joint_state["q"][4]]), np.array(self.targR))
            self.q1R_des += qRd_des[0] * self.dt
            self.q2R_des += qRd_des[1] * self.dt

            sig_targ = np.sign(self.targ - self.traj_leg)

            if self.traj_leg != self.targ:
                self.traj_leg += sig_targ * 0.5 * self.dt
            if sig_targ == 1 and self.traj_leg >= self.targ:
                self.traj_leg = self.targ
            elif sig_targ == -1 and self.traj_leg <= self.targ:
                self.traj_leg = self.targ
            
            if self.traj_leg == self.targ:
                self.time += self.dt
            if self.time >= 5:
                self.time = 0.0
                self.count += 1
                a = [-0.13, -0.28,-0.32, -0.28]
                self.targ = a[self.count%4]

            self.targL = [0.0, self.traj_leg]
            self.targR = [0.0, self.traj_leg]

            self.pos_cmd_msg.data = [self.q1L_des, self.q2L_des, self.q1R_des, self.q2R_des]
            # self.pos_cmd_msg.data = [q1L_des*0, q2L_des*0, 0.0, 0.0]
            self.pos_cmd_pub.publish(self.pos_cmd_msg)


            posl = self.forward_kinematics([self.joint_state["q"][0], self.joint_state["q"][1]])
            posr = self.forward_kinematics([self.joint_state["q"][3], self.joint_state["q"][4]])

                        # Save targ, posr, and posl to self.data_to_save
            self.data_to_save.append({
                'time': time.time(),  # You can log the current time or step
                'targx': 0.0,
                'targy': float(self.traj_leg),
                'poslx': float(posl[0]),
                'posrx': float(posr[0]),   
                'posly': float(posl[1]),  
                'posry': float(posr[1]),   
                'x' : float(self.x),
                'theta' : float(self.theta),
                'dtheta' : float(self.dtheta)
            })

            self.leg_pos_msg.data = [posl[0], posl[1], posr[0], posr[1]]
            self.leg_pub.publish(self.leg_pos_msg)
        except (TypeError, IndexError, KeyError) as e:
            self.get_logger().error(f"[ERROR]{e}")

    def robot_state_callback(self, msg):
        self.x = msg.data[0]
        self.theta = msg.data[1]
        self.dtheta = msg.data[2]

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
        v = err * self.kp

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

    def destroy_node(self):
        # Save data to a YAML file before shutting down
        with open('leg.yaml', 'w') as yaml_file:
            yaml.dump(self.data_to_save, yaml_file)
        self.get_logger().info("Data saved to controller_data.yaml")

        super().destroy_node()  # Call the parent destroy_node method
            
def main(args=None):
    rclpy.init(args=args)
    node = RobotKinematics()

    # Handle graceful shutdown on Ctrl+C
    def signal_handler(sig, frame):
        print("Caught SIGINT, stopping the robot...")
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
