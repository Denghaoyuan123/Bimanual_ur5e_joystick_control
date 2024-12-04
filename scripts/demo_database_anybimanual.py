#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import math
import argparse

def dh_matrix(theta, d, a, alpha):
    """Create the Denavit-Hartenberg transformation matrix."""
    return np.array(
        [
            [math.cos(theta), -math.sin(theta) * math.cos(alpha),  math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
            [math.sin(theta),  math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
            [0,                math.sin(alpha),                    math.cos(alpha),                   d],
            [0,                0,                                  0,                                 1],
        ]
    )

def forward_kinematics(joint_angles):
    """Compute the forward kinematics for the robot to get the transformation matrix from base to wrist."""
    dh_params = np.array(
        [
            [joint_angles[0], 0.1625, 0,          math.pi / 2],
            [joint_angles[1], 0,       -0.425,    0],
            [joint_angles[2], 0,       -0.3922,   0],
            [joint_angles[3], 0.1333,  0,         math.pi / 2],
            [joint_angles[4], 0.0997,  0,        -math.pi / 2],
            [joint_angles[5], 0.0996,  0,         0],
        ]
    )
    T = np.eye(4)
    for params in dh_params:
        T = np.dot(T, dh_matrix(*params))
    return T

def static_transform_wrist_to_gripper():
    """Provide the static transformation matrix from the wrist link to the gripper frame."""
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.16],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

class ImageSaver:
    def __init__(self, result_path):
        rospy.init_node('image_saver', anonymous=True)
        
        # Parameter initialization
        self.camera1_image_topic = rospy.get_param('~camera1_image_topic', '/camera1/color/image_raw')
        self.camera1_depth_topic = rospy.get_param('~camera1_depth_topic', '/camera1/aligned_depth_to_color/image_raw')
        self.camera2_image_topic = rospy.get_param('~camera2_image_topic', '/camera2/color/image_raw')
        self.camera2_depth_topic = rospy.get_param('~camera2_depth_topic', '/camera2/aligned_depth_to_color/image_raw')
        self.joint_topic_1 = rospy.get_param('~joint_topic1', '/robot1/joint_states')
        self.joint_topic_2 = rospy.get_param('~joint_topic2', '/robot2/joint_states')
        self.gripper_topic_1 = rospy.get_param('~gripper_state1', '/gripper_state1')
        self.gripper_topic_2 = rospy.get_param('~gripper_state2', '/gripper_state2')
        self.result_path = rospy.get_param('~result_path', os.path.expanduser(result_path))
        self.traj_file = None
        self.gripper_file = None

        # Try to create files and directories
        try:
            self.frame1_path = os.path.join(self.result_path, "camera1_frames")
            self.depth1_path = os.path.join(self.result_path, "camera1_depths")
            self.frame2_path = os.path.join(self.result_path, "camera2_frames")
            self.depth2_path = os.path.join(self.result_path, "camera2_depths")
            self.traj_file_path = os.path.join(self.result_path, "traj.txt")
            self.gripper_file_path = os.path.join(self.result_path, "gripper.txt")
            
            os.makedirs(self.frame1_path, exist_ok=True)
            os.makedirs(self.depth1_path, exist_ok=True)
            os.makedirs(self.frame2_path, exist_ok=True)
            os.makedirs(self.depth2_path, exist_ok=True)
            os.makedirs(self.result_path, exist_ok=True)
            
            self.traj_file = open(self.traj_file_path, "a")
            self.gripper_file = open(self.gripper_file_path, "a")
        except Exception as e:
            rospy.logerr(f"Initialization failed: {e}")
            # Cleanup operations in case of exception
            self.close_files()

        # Define other states and subscribers
        self.define_subscribers_and_other_states()

    def __del__(self):
        self.close_files()

    def close_files(self):
        if self.traj_file:
            self.traj_file.close()
        if self.gripper_file:
            self.gripper_file.close()

    def define_subscribers_and_other_states(self):
        # Subscribers for camera images and depth
        self.camera1_image_sub = rospy.Subscriber(self.camera1_image_topic, Image, self.camera1_image_callback)
        self.camera1_depth_sub = rospy.Subscriber(self.camera1_depth_topic, Image, self.camera1_depth_callback)
        self.camera2_image_sub = rospy.Subscriber(self.camera2_image_topic, Image, self.camera2_image_callback)
        self.camera2_depth_sub = rospy.Subscriber(self.camera2_depth_topic, Image, self.camera2_depth_callback)
        # Subscribers for joint states
        self.joint_sub_1 = rospy.Subscriber(self.joint_topic_1, JointState, self.joint_callback_1)
        self.joint_sub_2 = rospy.Subscriber(self.joint_topic_2, JointState, self.joint_callback_2)
        # Subscribers for gripper states
        self.gripper_sub_1 = rospy.Subscriber(self.gripper_topic_1, Float64MultiArray, self.gripper_callback_1)
        self.gripper_sub_2 = rospy.Subscriber(self.gripper_topic_2, Float64MultiArray, self.gripper_callback_2)
        
        # CV Bridge and other states
        self.bridge = CvBridge()
        self.joint_msg_1 = None
        self.joint_msg_2 = None
        self.camera1_color_msg = None
        self.camera1_depth_msg = None
        self.camera2_color_msg = None
        self.camera2_depth_msg = None
        self.gripper_msg_1 = None
        self.gripper_msg_2 = None
        self.frame_number = 0

        # Timer to periodically process data
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def joint_callback_1(self, msg):
        self.joint_msg_1 = msg
    
    def joint_callback_2(self, msg):
        self.joint_msg_2 = msg
    
    def gripper_callback_1(self, msg):
        self.gripper_msg_1 = msg

    def gripper_callback_2(self, msg):
        self.gripper_msg_2 = msg

    def camera1_image_callback(self, msg):
        self.camera1_color_msg = msg

    def camera1_depth_callback(self, msg):
        self.camera1_depth_msg = msg

    def camera2_image_callback(self, msg):
        self.camera2_color_msg = msg

    def camera2_depth_callback(self, msg):
        self.camera2_depth_msg = msg

    def save_transform(self, filename, data):
        """Save the transformation matrix data to a file."""
        with open(filename, 'ab') as f:
            np.savetxt(f, [data], delimiter=',', fmt='%.18e')

    def save_gripper_state(self, filename, data1, data2):
        """Save the gripper state data to a file."""
        with open(filename, 'ab') as f:
            formatted_data = np.column_stack((data1, data2))
            np.savetxt(f, formatted_data, delimiter=',', fmt='%.18e')

    def timer_callback(self, event):
        if (self.joint_msg_1 and self.joint_msg_2 and self.gripper_msg_1 and self.gripper_msg_2 and
            self.camera1_color_msg and self.camera1_depth_msg and self.camera2_color_msg and self.camera2_depth_msg):
            try:
                # For Robot 1
                joint_angles_1 = [self.joint_msg_1.position[self.joint_msg_1.name.index(joint)]
                                  for joint in ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]]
                base_to_wrist_transform_1 = forward_kinematics(joint_angles_1)
                wrist_to_gripper_transform_1 = static_transform_wrist_to_gripper()
                base_to_gripper_transform_1 = np.dot(base_to_wrist_transform_1, wrist_to_gripper_transform_1)
                flat_transform_1 = base_to_gripper_transform_1.flatten()

                # For Robot 2
                joint_angles_2 = [self.joint_msg_2.position[self.joint_msg_2.name.index(joint)]
                                  for joint in ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]]
                base_to_wrist_transform_2 = forward_kinematics(joint_angles_2)
                wrist_to_gripper_transform_2 = static_transform_wrist_to_gripper()
                base_to_gripper_transform_2 = np.dot(base_to_wrist_transform_2, wrist_to_gripper_transform_2)
                flat_transform_2 = base_to_gripper_transform_2.flatten()

                # Save transformations and gripper states
                self.save_transform(self.traj_file_path, np.concatenate((flat_transform_1, flat_transform_2)))
                self.save_gripper_state(self.gripper_file_path, self.gripper_msg_1.data, self.gripper_msg_2.data)

                # Process and save camera1 images and depth data
                cv_color_image1 = self.bridge.imgmsg_to_cv2(self.camera1_color_msg, desired_encoding='bgr8')
                frame_filename1 = os.path.join(self.frame1_path, f'frame{self.frame_number:06d}.jpg')
                cv2.imwrite(frame_filename1, cv_color_image1)

                cv_depth_image1 = self.bridge.imgmsg_to_cv2(self.camera1_depth_msg, desired_encoding="passthrough")
                depth_filename1 = os.path.join(self.depth1_path, f'depth{self.frame_number:06d}.png')
                cv2.imwrite(depth_filename1, cv_depth_image1)

                # Process and save camera2 images and depth data
                cv_color_image2 = self.bridge.imgmsg_to_cv2(self.camera2_color_msg, desired_encoding='bgr8')
                frame_filename2 = os.path.join(self.frame2_path, f'frame{self.frame_number:06d}.jpg')
                cv2.imwrite(frame_filename2, cv_color_image2)

                cv_depth_image2 = self.bridge.imgmsg_to_cv2(self.camera2_depth_msg, desired_encoding="passthrough")
                depth_filename2 = os.path.join(self.depth2_path, f'depth{self.frame_number:06d}.png')
                cv2.imwrite(depth_filename2, cv_depth_image2)

                self.frame_number += 1  # Increment frame counter only after successful operations
            except Exception as e:
                rospy.logerr(f"Error processing or saving data: {str(e)}")
        else:
            rospy.loginfo("Waiting for all messages to be received.")

def main():
    parser = argparse.ArgumentParser(description='Save images and joint states for the UR5e robot.')
    parser.add_argument('--result_path', type=str, default='Mani_data', help='Path to save the results')
    args = parser.parse_args()
    
    try:
        image_saver = ImageSaver(args.result_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

