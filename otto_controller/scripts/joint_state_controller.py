#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
import signal
import tf_transformations
import numpy as np
from geometry_msgs.msg import Twist

class Controller(Node):
    def __init__(self):
        super().__init__('x_des_to_effort_node')
        self.create_subscription(Float64MultiArray,'/x_des',self.x_des_callback,10)
        self.create_subscription(JointState,'/joint_states',self.feedback_callback,10)
        self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.effort_publisher = self.create_publisher(Float64MultiArray,'/effort_controller/commands',10)

        self.dt = 0.001
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # HL, KL, VHL, WL, HR, KR, VHR, WR
        self.x_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state = {}
        self.ctrl_input = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.kp = [160.0, 300, 0, 160, 300, 0]
        self.kd = [5.0, 1.0, 0, 5.0, 1.0, 0]
        self.ki = [3.0, 0.0, 0, 3, 0, 0]
        self.integral_error = [0.0] * 6
        self.integral_limit = 15.0

        self.imu_angle = None
        self.gyro = [0.0, 0.0, 0.0]
        self.accel = [0.0, 0.0, 0.0]
        self.e_i = 0
        
        self.vx = 0
        self.w = 0

        self.x_d = 0
        self.x_s = 0
        self.neta_d = 0
    def timer_callback(self):
        try:
            self.integrate()
            theta = self.imu_angle[1]
            dtheta = self.gyro[1]
            omega = (self.joint_state["qd"][1] - self.joint_state["qd"][0])*0.086/0.22584
            vx_s = (self.joint_state["qd"][0] + self.joint_state["qd"][1])*0.5*0.086
            self.integrate()
            self.x_s += vx_s * self.dt

            theta_d = np.arcsin((2*0.01*self.vx/0.086)/(5*0.2828*9.81))
            self.e_i += (theta_d - theta) * self.dt

            for i, joint_name in enumerate(self.joint_state["names"]):
                # pos ctrl
                if joint_name != "wheel_jointL" and joint_name != "wheel_jointR":
                    pos_err = self.x_des[i] - self.joint_state["q"][i]
                    self.integral_error[i] += pos_err * self.dt
                    self.integral_error[i] = max(min(self.integral_error[i], self.integral_limit), -self.integral_limit)
                    if abs(pos_err) < 0.0001:
                         self.integral_error[i] = 0.0
                    p_term = pos_err * self.kp[i]
                    i_term = self.integral_error[i] * self.ki[i]
                    d_term = -self.joint_state["qd"][i] * self.kd[i]
                    
                    u_i = p_term + i_term + d_term
                    if u_i >= 5.0:
                        u_i = 5.0
                    elif u_i < -5.0:
                        u_i = -5.0
                    self.ctrl_input[i] = u_i
                elif joint_name == "wheel_jointL":
                    ffw = (5*0.2828*9.81*np.sin(theta_d) - 2*0.01*self.vx/0.086)/2
                    tauL = (self.x_d - self.x_s)*0 + (theta_d - theta) * -65.63 + dtheta * 12.5305 + (self.vx - vx_s) *-13.9589*1 + (self.w - omega) * 0* -3.1891
                    self.ctrl_input[i] = (ffw + tauL)*1
                    if self.ctrl_input[i] > 2.2:
                        self.ctrl_input[i] = 2.2
                    elif self.ctrl_input[i] < -2.2:
                        self.ctrl_input[i] = -2.2

                elif joint_name == "wheel_jointR":
                    ffw = (5*0.2828*9.81*np.sin(theta_d) - 2*0.01*self.vx/0.086)/2
                    tauR = (self.x_d - self.x_s)*0 + (theta_d - theta) * -65.63 + dtheta * 12.5305 + (self.vx - vx_s) *-13.9589*1 + (self.w - omega) * 0* 3.1891
                    self.ctrl_input[i] = (ffw + tauR)*1
                    if self.ctrl_input[i] > 2.2:
                        self.ctrl_input[i] = 2.2
                    elif self.ctrl_input[i] < -2.2:
                        self.ctrl_input[i] = -2.2


            state_q_str = ", ".join(f"{np.rad2deg(q):.4f}" for q in self.joint_state["q"])
            state_qd_str = ", ".join(f"{qd:.4f}" for qd in self.joint_state["qd"])
            ctrl_input_str = ", ".join(f"{ci:.4f}" for ci in self.ctrl_input)

            print("stateq: ", state_q_str)
            # print("stateqd: ", state_qd_str)
            # print("ctrl_input: ", ctrl_input_str)
            effort_msg = Float64MultiArray()
            effort_msg.data = self.ctrl_input
            self.effort_publisher.publish(effort_msg)

        except (TypeError, IndexError, KeyError) as e:
            self.get_logger().error(f"Error updating control input: {e}")
    
    def integrate(self):
        self.x_d += self.vx*self.dt
        self.neta_d += self.w*self.dt

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.w = msg.angular.z

    def feedback_callback(self, msg):
        joint_names = msg.name
        feedback_q = msg.position
        feedback_qd = msg.velocity
        feedback_efforts = msg.effort

        desired_order = [
            'knee_jointL', 'virtualhip_jointL', 'wheel_jointL',
            'knee_jointR', 'virtualhip_jointR', 'wheel_jointR'
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
        self.ctrl_input = [0.0] * 6
        zero_effort_msg = Float64MultiArray()
        zero_effort_msg.data = self.ctrl_input
        self.effort_publisher.publish(zero_effort_msg)
        print("Robot stopped with zero effort.")

    def destroy_node(self):
        self.stop_robot()
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
