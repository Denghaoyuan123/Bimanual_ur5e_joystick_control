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
            [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
            [math.sin(theta),  math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1],
        ]
    )

def forward_kinematics(joint_angles):
    """Compute the forward kinematics for the robot to get the transformation matrix from base to wrist."""
    dh_params = np.array(
        [
            [joint_angles[0], 0.1625, 0, math.pi / 2],
            [joint_angles[1], 0, -0.425, 0],
            [joint_angles[2], 0, -0.3922, 0],
            [joint_angles[3], 0.1333, 0, math.pi / 2],
            [joint_angles[4], 0.0997, 0, -math.pi / 2],
            [joint_angles[5], 0.0996, 0, 0],
        ]
    )
    T = np.eye(4)
    for params in dh_params:
        T = np.dot(T, dh_matrix(*params))
    return T

def static_transform_wrist_to_gripper():
    """Provide the static transformation matrix from the wrist link to the camera frame."""
    return np.array(
        [[1.0, 0.0, 0.0, 0.0],
         [0.0, 1.0, 0.0, 0.0],
         [0.0, 0.0, 1.0, 0.16],
         [0.0, 0.0, 0.0, 1.0]]
    )

class ImageSaver:
    def __init__(self, result_path):
        rospy.init_node('image_saver', anonymous=True)
        
        # Parameters
        self.camera1_image_topic = rospy.get_param('~camera1_image_topic', '/camera1/color/image_raw')
        self.camera1_depth_topic = rospy.get_param('~camera1_depth_topic', '/camera1/aligned_depth_to_color/image_raw')
        self.camera2_image_topic = rospy.get_param('~camera2_image_topic', '/camera2/color/image_raw')
        self.camera2_depth_topic = rospy.get_param('~camera2_depth_topic', '/camera2/aligned_depth_to_color/image_raw')
        self.joint_topic = rospy.get_param('~joint_topic', '/joint_states')
        self.gripper_topic = rospy.get_param('~gripper_state', '/gripper_state')
        # self.result_path = rospy.get_param('~result_path', os.path.expanduser('txt'))
        self.result_path = rospy.get_param('~result_path', os.path.expanduser(result_path))

        # Paths
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
        
        # File
        self.traj_file = open(self.traj_file_path, "a")
        self.gripper_file = open(self.gripper_file_path, "a")
        self.test = True
        
        # Subscribers
        self.camera1_image_sub = rospy.Subscriber(self.camera1_image_topic, Image, self.camera1_image_callback)
        self.camera1_depth_sub = rospy.Subscriber(self.camera1_depth_topic, Image, self.camera1_depth_callback)
        self.camera2_image_sub = rospy.Subscriber(self.camera2_image_topic, Image, self.camera2_image_callback)
        self.camera2_depth_sub = rospy.Subscriber(self.camera2_depth_topic, Image, self.camera2_depth_callback)
        self.joint_sub = rospy.Subscriber(self.joint_topic, JointState, self.joint_callback)
        self.gripper_sub = rospy.Subscriber(self.gripper_topic, Float64MultiArray, self.gripper_callback)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # State
        self.joint_msg = None
        self.camera1_color_msg = None
        self.camera1_depth_msg = None
        self.camera2_color_msg = None
        self.camera2_depth_msg = None
        self.gripper_msg = None
        self.frame_number = 0

        # Timer
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def __del__(self):
        self.traj_file.close()

    def joint_callback(self, msg):
        self.joint_msg = msg
    
    def gripper_callback(self, msg):
        self.gripper_msg = msg

    def camera1_image_callback(self, msg):
        self.camera1_color_msg = msg

    def camera1_depth_callback(self, msg):
        self.camera1_depth_msg = msg

    def camera2_image_callback(self, msg):
        self.camera2_color_msg = msg

    def camera2_depth_callback(self, msg):
        self.camera2_depth_msg = msg

    
    def timer_callback(self, event):
        if self.joint_msg and self.gripper_msg and self.camera1_color_msg and self.camera1_depth_msg and self.camera2_color_msg and self.camera2_depth_msg:
            try:
                # Compute the transformation matrices
                joint_angles = [self.joint_msg.position[self.joint_msg.name.index(joint)]
                                for joint in ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]]
                base_to_wrist_transform = forward_kinematics(joint_angles)
                wrist_to_gripper_transform = static_transform_wrist_to_gripper()
                base_to_gripper_transform = np.dot(base_to_wrist_transform, wrist_to_gripper_transform)
                flat_transform = base_to_gripper_transform.flatten()

                # Save transformation matrix
                np.savetxt(self.traj_file, [flat_transform], fmt='%.18e')
                self.traj_file.flush()  # Ensure data is written to disk

                # Save gripper state
                np.savetxt(self.gripper_file, [self.gripper_msg.data], fmt='%.18e')
                self.gripper_file.flush()  # Ensure data is written to disk

                # Process and save images for camera1
                cv_color_image1 = self.bridge.imgmsg_to_cv2(self.camera1_color_msg, desired_encoding='bgr8')
                frame_filename1 = os.path.join(self.frame1_path, f'frame{self.frame_number:06d}.jpg')
                cv2.imwrite(frame_filename1, cv_color_image1)

                cv_depth_image1 = self.bridge.imgmsg_to_cv2(self.camera1_depth_msg, desired_encoding="passthrough")
                depth_filename1 = os.path.join(self.depth1_path, f'depth{self.frame_number:06d}.png')
                cv2.imwrite(depth_filename1, cv_depth_image1)

                # Process and save images for camera2
                cv_color_image2 = self.bridge.imgmsg_to_cv2(self.camera2_color_msg, desired_encoding='bgr8')
                frame_filename2 = os.path.join(self.frame2_path, f'frame{self.frame_number:06d}.jpg')
                cv2.imwrite(frame_filename2, cv_color_image2)

                cv_depth_image2 = self.bridge.imgmsg_to_cv2(self.camera2_depth_msg, desired_encoding="passthrough")
                depth_filename2 = os.path.join(self.depth2_path, f'depth{self.frame_number:06d}.png')
                cv2.imwrite(depth_filename2, cv_depth_image2)
                while self.test:
                    print("depth review")
                    cv2.imshow("depth", cv_depth_image2)
                    depth = cv2.imread(depth_filename2,cv2.IMREAD_UNCHANGED)
                    # 排除所有零值
                    nonzero_depth = depth[depth > 0]
        
                    if nonzero_depth.size == 0:
                        print("No non-zero depth values found.")
                        continue
                    # 找出最小值
                    min_depth = np.min(nonzero_depth)
                    print(f"Minimum depth value (excluding zeros): {min_depth}mm")

                    # 定义区间
                    bins = [1, 200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800, 2000, np.inf]
                    # 计算各区间的数量
                    hist, bin_edges = np.histogram(nonzero_depth, bins=bins)
                    print("Depth distribution:")
                    for i in range(len(bins)-1):
                        print(f"{bins[i]}mm to {bins[i+1]}mm: {hist[i]}")

                    self.test = False

                self.frame_number += 1  # Increment frame counter only after successful operations
            except Exception as e:
                rospy.logerr(f"Error processing or saving data: {str(e)}")

def main():
    parser = argparse.ArgumentParser(description='Save images and joint states for the UR5e robot.')
    parser.add_argument('--result_path', type=str, required="Mani_data", help='Path to save the results')
    args = parser.parse_args()
    # args.result_path = "/home/pine/Mani_data"
    try:
        image_saver = ImageSaver(args.result_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()