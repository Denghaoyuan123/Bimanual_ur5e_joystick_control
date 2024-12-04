#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from RobotiqGripper import RobotiqHand
import numpy as np
import sys
import os
from airo_robots.grippers.hardware.robotiq_2f85_urcap import Robotiq2F85

# script_dir = os.path.dirname(__file__)  # 获取当前脚本文件所在的目录
# airo_robots_dir = os.path.join(script_dir, 'airo-mono/airo-robots/airo_robots/grippers/hardware')
# sys.path.append(airo_robots_dir)

# # 现在可以导入 robotiq_2f85_urcap.py 中的类
# # from robotiq_2f85_urcap import Robotiq2F85  # 将 ClassName 替换为你需要导入的类名


class UR5eInverseKinematics1:
    def __init__(self):
        rospy.init_node('ur5e_inverse_kinematics2', anonymous=True)

        self.alpha = 0.1
        self.previous_velocities = np.zeros(6)
        self.gripper = RobotiqHand()
        # self.gripper = Robotiq2F85(host_ip="192.168.0.144", port=63352)
        # self.gripper = ()
        self.gripper.connect("192.168.0.144", 54321)
        self.gripper.reset()
        self.gripper.activate()
        self.gripper.wait_activate_complete()
        self.gripper_position = 0

        self.dh_d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]
        self.dh_a = [0, -0.425, -0.3922, 0, 0, 0]
        self.dh_alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
        self.joint_angles = [0, 0, 0, 0, 0, 0]  # Initial joint angles
        self.gripper_publisher = rospy.Publisher('/gripper_state2', Float64MultiArray, queue_size=1)

        self.joint_angles_received = False  # Flag to check if joint angles are received
        self.subscription_joy = rospy.Subscriber('/joy2', Joy, self.joystick_callback)
        self.subscription_joint_states = rospy.Subscriber('/robot2/joint_states', JointState, self.joint_states_callback)
        self.publisher = rospy.Publisher('/robot2/joint_group_vel_controller/command', Float64MultiArray, queue_size=1)
    
    # def publish_gripper_state(self):
    #     gripper_msg = Float64MultiArray()
    #     gripper_msg.data = [self.gripper_position]  # Publish the current gripper position
    #     self.gripper_publisher.publish(gripper_msg)

    def publish_gripper_state(self, actual_position):
        gripper_msg = Float64MultiArray()
        gripper_msg.data = [actual_position]  # Publish the actual gripper position
        self.gripper_publisher.publish(gripper_msg)

    def joint_states_callback(self, msg):
        joint_order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        joint_positions = dict(zip(msg.name, msg.position))
        self.joint_angles = [joint_positions[joint] for joint in joint_order]
        self.joint_angles_received = True

    def joystick_callback(self, msg):
        if not self.joint_angles_received:
            rospy.loginfo('Robot 2 Joint angles not received yet, skipping inverse kinematics calculation.')
            return

        x_velocity = -msg.axes[0] if abs(msg.axes[0]) >= 0.2 else 0
        y_velocity = msg.axes[1] if abs(msg.axes[1]) >= 0.01 else 0
        z_velocity = msg.axes[4] if abs(msg.axes[4]) >= 0.01 else 0
        roll_velocity = msg.axes[3] if abs(msg.axes[3]) >= 0.01 else 0
        pitch_velocity = msg.axes[6] if abs(msg.axes[6]) >= 0.01 else 0
        yaw_velocity = msg.axes[7] if abs(msg.axes[7]) >= 0.01 else 0

        # if msg.buttons[4] == 1:
        #     rospy.loginfo('Opening gripper')
        #     self.gripper_position += 10
        #     if self.gripper_position > 255:
        #         self.gripper_position = 255
        #     self.gripper.move(self.gripper_position, 0, 100)
        #     self.publish_gripper_state()  # Publish the new gripper state
        
        # if msg.buttons[5] == 1:
        #     rospy.loginfo('Closing gripper')
        #     self.gripper_position -= 10
        #     if self.gripper_position < 0:
        #         self.gripper_position = 0
        #     self.gripper.move(self.gripper_position, 0, 100)
        #     self.publish_gripper_state()  # Publish the new gripper state
        
        if msg.buttons[4] == 1:
            rospy.loginfo('Opening gripper')
            self.gripper_position += 200
            if self.gripper_position > 255:
                self.gripper_position = 255
            self.gripper.move(self.gripper_position, 170, 10)
            status, actual_position, force = self.gripper.wait_move_complete()
            self.publish_gripper_state(actual_position)  # Publish the actual gripper state

        if msg.buttons[5] == 1:
            rospy.loginfo('Closing gripper')
            self.gripper_position -= 200
            if self.gripper_position < 0:
                self.gripper_position = 0
            self.gripper.move(self.gripper_position, 170, 10)
            status, actual_position, force = self.gripper.wait_move_complete()
            self.publish_gripper_state(actual_position)  # Publish the actual gripper state
        

        # Base to end-effector velocity transformation
        eef_base_velocity = np.array([0.05 * x_velocity, 0.05 * y_velocity, 0.05 * z_velocity, 0.5 * roll_velocity, 0.5 * pitch_velocity, 0.5 * yaw_velocity])
        eef_base_velocity = np.where(np.abs(eef_base_velocity) < 0.008, 0, eef_base_velocity)
        rospy.loginfo('End-effector velocity: {}'.format(eef_base_velocity))
        joint_velocities = self.inverse_kinematics(self.joint_angles, eef_base_velocity)
        filtered_velocities = self.alpha * joint_velocities + (1 - self.alpha) * self.previous_velocities
        self.publish_joint_velocities(filtered_velocities)
        self.previous_velocities = filtered_velocities

    def compute_jacobian(self, theta):
        J = np.zeros((6, 6))  # Jacobian matrix
        T = np.eye(4)  # Transformation matrix from the base to the current joint

        # Lists to store z axes and origin points for each joint
        z_vectors = []
        p_vectors = []

        # Start with the base z-axis, which is [0, 0, 1] for the initial configuration
        z_base = np.array([0, 0, 1])  # This is z0
        p_base = np.array([0, 0, 0])  # This is p0

        # Initial base z-axis and origin point
        z_vectors.append(z_base)
        p_vectors.append(p_base)

        for i in range(6):
            alpha = self.dh_alpha[i]  # Twist angle
            a = self.dh_a[i]          # Link length
            d = self.dh_d[i]          # Link offset

            # Compute transformation matrix for joint i
            Ti = np.array([
                [np.cos(theta[i]), -np.sin(theta[i]) * np.cos(alpha), np.sin(theta[i]) * np.sin(alpha), a * np.cos(theta[i])],
                [np.sin(theta[i]), np.cos(theta[i]) * np.cos(alpha), -np.cos(theta[i]) * np.sin(alpha), a * np.sin(theta[i])],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]
            ])

            # Update transformation matrix from base to current joint
            T = np.dot(T, Ti)
            
            # Update and store z axis and origin point for the current joint
            z_vectors.append(T[:3, 2])
            p_vectors.append(T[:3, 3])

        p_end = p_vectors[-1]  # Position of the end effector

        for i in range(6):
            # Compute columns of the Jacobian matrix
            J[:3, i] = np.cross(z_vectors[i], p_end - p_vectors[i])  # Linear velocity component
            J[3:, i] = z_vectors[i]  # Angular velocity component

        return J

    def inverse_kinematics(self, theta, end_effector_velocity):
        J = self.compute_jacobian(theta)
        J_pinv = np.linalg.pinv(J)  # Pseudoinverse of Jacobian
        joint_velocities = np.dot(J_pinv, end_effector_velocity)
        return joint_velocities

    def publish_joint_velocities(self, joint_velocities):
        msg = Float64MultiArray()
        msg.data = joint_velocities.tolist()
        self.publisher.publish(msg)
        return

def main():
    try:
        ur5e_inverse_kinematics = UR5eInverseKinematics1()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
