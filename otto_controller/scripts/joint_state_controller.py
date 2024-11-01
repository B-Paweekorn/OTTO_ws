#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import signal

class Controller(Node):
    def __init__(self):
        super().__init__('x_des_to_effort_node')
        self.create_subscription(Float64MultiArray,'/x_des',self.x_des_callback,10)
        self.create_subscription(JointState,'/joint_states',self.feedback_callback,10)

        self.effort_publisher = self.create_publisher(Float64MultiArray,'/effort_controller/commands',10)

        self.dt = 0.002
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # HL, KL, VHL, WL, HR, KR, VHR, WR
        self.x_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state = {}
        self.ctrl_input = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.kp = [8.4, 12.0, 12.0, 0, 8.4, 12.0, 12.0, 0]
        self.kd = [0.1, 0.1, 0.5, 0, 0.1, 0.1, 0.5, 0]
        self.ki = [1, 1, 1, 0, 1, 1, 1, 0]
        self.integral_error = [0.0] * 8
        self.integral_limit = 10.0

    def timer_callback(self):
        for i, joint_name in enumerate(self.joint_state["names"]):
            # pos ctrl
            if joint_name != "wheel_jointL" and joint_name != "wheel_jointR":
                pos_err = self.x_des[i] - self.joint_state["q"][i]
                self.integral_error[i] += pos_err * self.dt
                self.integral_error[i] = max(min(self.integral_error[i], self.integral_limit), -self.integral_limit)
                
                p_term = pos_err * self.kp[i]
                i_term = self.integral_error[i] * self.ki[i]
                d_term = -self.joint_state["qd"][i] * self.kd[i]
                
                self.ctrl_input[i] = p_term + i_term + d_term
            else:
                self.ctrl_input[i] = 0.0

        state_q_str = ", ".join(f"{q:.4f}" for q in self.joint_state["q"])
        state_qd_str = ", ".join(f"{qd:.4f}" for qd in self.joint_state["qd"])
        ctrl_input_str = ", ".join(f"{ci:.4f}" for ci in self.ctrl_input)

        print("stateq: ", state_q_str)
        print("stateqd: ", state_qd_str)
        print("ctrl_input: ", ctrl_input_str)
        effort_msg = Float64MultiArray()
        effort_msg.data = self.ctrl_input
        self.effort_publisher.publish(effort_msg)

    def feedback_callback(self, msg):
        joint_names = msg.name
        feedback_q = msg.position
        feedback_qd = msg.velocity
        feedback_efforts = msg.effort

        desired_order = [
            'hip_jointL', 'knee_jointL', 'virtualhip_jointL', 'wheel_jointL',
            'hip_jointR', 'knee_jointR', 'virtualhip_jointR', 'wheel_jointR'
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
    

    def stop_robot(self):
        self.ctrl_input = [0.0] * 8
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
