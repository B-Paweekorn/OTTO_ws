#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
import signal
import tf_transformations

class Controller(Node):
    def __init__(self):
        super().__init__('x_des_to_effort_node')
        self.create_subscription(Float64MultiArray,'/x_des',self.x_des_callback,10)
        self.create_subscription(JointState,'/joint_states',self.feedback_callback,10)
        self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)

        self.effort_publisher = self.create_publisher(Float64MultiArray,'/effort_controller/commands',10)

        self.dt = 0.002
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # HL, KL, VHL, WL, HR, KR, VHR, WR
        self.x_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state = {}
        self.ctrl_input = [0.0, 0.0]

        self.kp = [8.4, 12.0, 12.0, 0, 8.4, 12.0, 12.0, 0]
        self.kd = [0.1, 0.1, 0.5, 0, 0.1, 0.1, 0.5, 0]
        self.ki = [1, 1, 1, 0, 1, 1, 1, 0]
        self.integral_error = [0.0] * 8
        self.integral_limit = 10.0

        self.imu_angle = None
        self.gyro = [0.0, 0.0, 0.0]
        self.accel = [0.0, 0.0, 0.0]
        self.e_i = 0
    def timer_callback(self):
        try:
            imu = self.imu_angle[1]
            for i in range(0,2):
                self.e_i += imu * self.dt
                self.ctrl_input[i] =imu * 100 + self.joint_state["qd"][i] * -0.3 + self.gyro[1] * 5 + self.joint_state["qd"][i] * 1  # self.joint_state["q"][i] * 0.0001
                if self.ctrl_input[i] > 10:
                    self.ctrl_input[i] = 10.0
                elif self.ctrl_input[i] < -10:
                    self.ctrl_input[i] = -10.0

                print(self.ctrl_input, imu)
            state_q_str = ", ".join(f"{q:.4f}" for q in self.joint_state["q"])
            state_qd_str = ", ".join(f"{qd:.4f}" for qd in self.joint_state["qd"])
            ctrl_input_str = ", ".join(f"{ci:.4f}" for ci in self.ctrl_input)

            # print("stateq: ", state_q_str)
            # print("stateqd: ", state_qd_str)
            # print("ctrl_input: ", ctrl_input_str)
            effort_msg = Float64MultiArray()
            effort_msg.data = self.ctrl_input
            self.effort_publisher.publish(effort_msg)
        except (TypeError, IndexError, KeyError) as e:
            self.get_logger().error(f"Error updating control input: {e}")
    def input_handler(self):
        while True:
            user_input = input("Enter 'kp' or 'kd' followed by the value (e.g., 'kp 100'): ").strip()
            try:
                param, value = user_input.split()
                value = float(value)
                if param.lower() == 'kp':
                    self.kp = value
                    self.get_logger().info(f'Kp set to {self.kp}')
                elif param.lower() == 'kd':
                    self.kd = value
                    self.get_logger().info(f'Kd set to {self.kd}')
                else:
                    self.get_logger().info("Invalid input. Use 'kp' or 'kd' followed by the value.")
            except ValueError:
                self.get_logger().info("Invalid input format. Please use 'kp 100' or 'kd 10'.")

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
