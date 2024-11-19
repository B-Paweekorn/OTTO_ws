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
        super().__init__('q_d_to_effort_node')
        self.create_subscription(JointState,'/joint_states',self.feedback_callback,10)
        self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.effort_publisher = self.create_publisher(Float64MultiArray,'/effort_controller/commands',10)

        self.dt = 0.001
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.joint_state = {}
        self.dof = 2
        # KL, VHL, WL, KR, VHR, WR
        self.q_d = [0.0] * self.dof
        self.ctrl_input = [0.0] * self.dof

        # imu
        self.imu_angle = None
        self.gyro = [0.0, 0.0, 0.0]
        self.accel = [0.0, 0.0, 0.0]
        
        # command
        self.vx = 0
        self.w = 0

        # robot geometry
        self.r = 0.086
        self.d = 0.2254
        self.l = 0.2828
        self.g = 9.81
        self.m_b = 3.0
        self.mu = 0.01

        # LQR Gain (x, vx, th, dth, w, dw)
        self.lqrL = [-10.0, -14.019, -65.63, -12.5305, 3.1623, -3.1979]
        self.lqrR = [-10.0, -14.019, -65.63, -12.5305, -3.1623, 3.1979]

        self.effort_msg = Float64MultiArray()

    def timer_callback(self):
        try:
            th_s = self.imu_angle[1]
            dth_s = self.gyro[1]
            w_s = (self.joint_state["qd"][5] - self.joint_state["qd"][2])*self.r/self.d
            vx_s = (self.joint_state["qd"][2] + self.joint_state["qd"][5])*0.5*self.r

            th_d = np.arcsin((2*self.mu*self.vx/self.r)/(self.m_b*self.l*self.g))
            ffw = (self.m_b*self.l*self.g*np.sin(th_d) - 2*self.mu*self.vx/self.r)/2

            tauL = (th_d - th_s) * self.lqrL[2] + (0 - dth_s) * self.lqrL[3] + (self.vx - vx_s) * self.lqrL[1] + (self.w - w_s) * self.lqrL[5]
            tauR = (th_d - th_s) * self.lqrR[2] + (0 - dth_s) * self.lqrR[3] + (self.vx - vx_s) * self.lqrR[1] + (self.w - w_s) * self.lqrR[5]

            self.ctrl_input[0] = ffw + tauL
            self.ctrl_input[1] = ffw + tauR

            wheel_lim = 2.2

            if self.ctrl_input[0] > wheel_lim:
                self.ctrl_input[0] = wheel_lim
            elif self.ctrl_input[0] < -wheel_lim:
                self.ctrl_input[0] = -wheel_lim

            if self.ctrl_input[1] > wheel_lim:
                self.ctrl_input[1] = wheel_lim
            elif self.ctrl_input[1] < -wheel_lim:
                self.ctrl_input[1] = -wheel_lim

            self.effort_msg.data = self.ctrl_input
            self.effort_publisher.publish(self.effort_msg)
            state_q_str = ", ".join(f"{np.rad2deg(q):.4f}" for q in self.joint_state["q"])
            state_qd_str = ", ".join(f"{qd:.4f}" for qd in self.joint_state["qd"])
            ctrl_input_str = ", ".join(f"{ci:.4f}" for ci in self.ctrl_input)

            print("stateq: ", state_q_str)
            # print("stateqd: ", state_qd_str)
            # print("ctrl_input: ", ctrl_input_str)

        except (TypeError, IndexError, KeyError) as e:
            self.get_logger().error(f"Error updating control input: {e}")

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.w = msg.angular.z

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
        self.ctrl_input = [0.0] * self.dof
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

    def signal_handler(sig, frame):
        print("Caught SIGINT, stopping the robot...")
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
