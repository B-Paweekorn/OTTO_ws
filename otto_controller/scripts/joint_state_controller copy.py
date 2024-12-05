#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
import signal
import tf_transformations
import time
import numpy as np
import yaml
class Controller(Node):
    def __init__(self):
        super().__init__('x_des_to_effort_node')
        self.create_subscription(Float64MultiArray,'/x_des',self.x_des_callback,10)
        self.create_subscription(JointState,'/joint_states',self.feedback_callback,10)
        self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.create_subscription(Twist, 'traj/cmd_vel', self.cmd_vel_callback, 10)
        self.effort_publisher = self.create_publisher(Float64MultiArray,'/effort_controller/commands',10)

        self.dt = 0.001
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # HL, KL, VHL, WL, HR, KR, VHR, WR
        self.x_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state = {}
        self.ctrl_input = [0.0, 0.0]

        # self.kp = [8.4, 12.0, 12.0, 0, 8.4, 12.0, 12.0, 0]
        # self.kd = [0.1, 0.1, 0.5, 0, 0.1, 0.1, 0.5, 0]

        # self.lqrL = [-10.0, -10.019, -26.63, -5.5305, 10.1623, -8.1979]
        # self.lqrR = [-10.0, -10.019, -26.63, -5.5305, -10.1623, 8.1979]
        self.ki = [1, 1, 1, 0, 1, 1, 1, 0]
        self.integral_error = [0.0] * 8
        self.integral_limit = 10.0

        self.imu_angle = None
        self.gyro = [0.0, 0.0, 0.0]
        self.accel = [0.0, 0.0, 0.0]
        self.e_i = 0

        self.vx = 0
        self.w = 0
        self.effort_msg = Float64MultiArray()

        self.x_s = 0
        self.x_d = 0

        self.data_to_save = []
    def timer_callback(self):
        try:
            theta = self.imu_angle[1]
            dtheta = self.gyro[1]
            omega = (self.joint_state["qd"][1] - self.joint_state["qd"][0])*0.086/0.22584
            vx_s = (self.joint_state["qd"][0] + self.joint_state["qd"][1])*0.5*0.086

            self.x_d += self.vx * self.dt
            # if self.vx == 0 and abs():
            #     self.x_d = self.x_s

            self.x_s += vx_s * self.dt
            # print(omega)

            theta_d = 0*np.arcsin((2*0.01*self.vx/0.086)/(3*0.2828*9.81))
            ffw = (3*0.2828*9.81*np.sin(theta_d) - 2*0.01*self.vx/0.086)/2
            self.e_i += (self.x_d - self.x_s) * self.dt
            # print(np.rad2deg(theta_d))
            # tauL = (theta_d - theta) * -65.63 + dtheta * 12.5305 + (self.vx - vx_s)* -13.9589 + (self.w - omega) * -3.1891
            # tauR = (theta_d - theta) * -65.63 + dtheta * 12.5305 + (self.vx - vx_s)* -13.9589 + (self.w - omega) * +3.1891
            tauL = self.e_i*-0.707*0 + (self.x_d - self.x_s)* -3.25*0 + (0 - theta) * -24 + dtheta * 6.8 + (self.vx - vx_s)* -12.80 + (self.w - omega) * -4.16
            tauR = self.e_i*-0.707*0 + (self.x_d - self.x_s)* -3.25*0 + (0 - theta) * -24 + dtheta * 6.8 + (self.vx - vx_s)* -12.80 + (self.w - omega) * +4.16
            self.ctrl_input[0] = 1*ffw + tauL
            self.ctrl_input[1] = 1*ffw + tauR

            if self.ctrl_input[0] > 2.2:
                self.ctrl_input[0] = 2.2
            elif self.ctrl_input[0] < -2.2:
                self.ctrl_input[0] = -2.2
            
            if self.ctrl_input[1] > 2.2:
                self.ctrl_input[1] = 2.2
            elif self.ctrl_input[1] < -2.2:
                self.ctrl_input[1] = -2.2

            print(vx_s)
            # print(dtheta)
            # print(np.rad2deg(theta_d), np.rad2deg(theta))
            state_q_str = ", ".join(f"{q:.4f}" for q in self.joint_state["q"])
            state_qd_str = ", ".join(f"{qd:.4f}" for qd in self.joint_state["qd"])
            ctrl_input_str = ", ".join(f"{ci:.4f}" for ci in self.ctrl_input)

            # print("stateq: ", state_q_str)
            # print("stateqd: ", state_qd_str)
            # print("ctrl_input: ", ctrl_input_str)

            self.effort_msg.data = self.ctrl_input
            self.effort_publisher.publish(self.effort_msg)

            self.data_to_save.append({
                'time': time.time(),
                'v_xs': vx_s,
                'v_xd': self.vx,
                'w': omega,
                'wd': self.w
                # 'x_s': self.x_s,
                # 'theta': theta,
                # 'dtheta': dtheta
            })
        except (TypeError, IndexError, KeyError) as e:
            self.get_logger().error(f"Error updating control input: {e}")

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.w = msg.angular.z

    def feedback_callback(self, msg):
        joint_names = msg.name
        feedback_q = msg.position
        feedback_qd = msg.velocity
        feedback_efforts = msg.effort

        desired_order = [
            'wheel_jointL',
            'wheel_jointR'
        ]

        joint_data = {
            name: (q, qd, effort)
            for name, q, qd, effort in zip(joint_names, feedback_q, feedback_qd, feedback_efforts)
        }

        self.joint_state = {
            'names': desired_order,
            'q': [joint_data[joint][0] for joint in desired_order],
            'qd': [joint_data[joint][1] for joint in desired_order],
            'efforts': [joint_data[joint][2] for joint in desired_order]
        }
                
    def x_des_callback(self, msg):
        self.x_des = msg.data
    
    def imu_callback(self, msg):
        orientation_q = msg.orientation
        quaternion = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]
        
        self.imu_angle = tf_transformations.euler_from_quaternion(quaternion)
        self.gyro[0] = msg.angular_velocity.x
        self.gyro[1] = msg.angular_velocity.y
        self.gyro[2] = msg.angular_velocity.z

        self.accel[0] = msg.linear_acceleration.x
        self.accel[1] = msg.linear_acceleration.y
        self.accel[2] = msg.linear_acceleration.z

    def stop_robot(self):
        self.ctrl_input = [0.0] * 2
        zero_effort_msg = Float64MultiArray()
        zero_effort_msg.data = self.ctrl_input
        self.effort_publisher.publish(zero_effort_msg)
        print("Robot stopped with zero effort.")

    def destroy_node(self):
        self.stop_robot()

        with open('controller_data.yaml', 'w') as yaml_file:
            yaml.dump(self.data_to_save, yaml_file)
        print("Data saved to controller_data.yaml")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    # Handle graceful shutdown on Ctrl+C
    def signal_handler(sig, frame):
        print("Caught SIGINT, stopping the robot...")
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
