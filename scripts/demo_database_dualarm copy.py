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
        
        # 参数初始化
        self.camera2_image_topic = rospy.get_param('~camera2_image_topic', '/camera2/color/image_raw')
        self.camera2_depth_topic = rospy.get_param('~camera2_depth_topic', '/camera2/aligned_depth_to_color/image_raw')
        self.joint_topic_1 = rospy.get_param('~joint_topic1', '/robot1/joint_states')
        self.joint_topic_2 = rospy.get_param('~joint_topic2', '/robot2/joint_states')
        self.gripper_topic_1 = rospy.get_param('~gripper_state1', '/gripper_state1')
        self.gripper_topic_2 = rospy.get_param('~gripper_state2', '/gripper_state2')
        self.result_path = rospy.get_param('~result_path', os.path.expanduser(result_path))
        self.traj_file = None
        self.gripper_file = None

        # 尝试创建文件和目录
        try:
            self.frame2_path = os.path.join(self.result_path, "camera2_frames")
            self.depth2_path = os.path.join(self.result_path, "camera2_depths")
            self.traj_file_path = os.path.join(self.result_path, "traj.txt")
            self.gripper_file_path = os.path.join(self.result_path, "gripper.txt")
            
            os.makedirs(self.frame2_path, exist_ok=True)
            os.makedirs(self.depth2_path, exist_ok=True)
            os.makedirs(self.result_path, exist_ok=True)
            
            self.traj_file = open(self.traj_file_path, "a")
            self.gripper_file = open(self.gripper_file_path, "a")
        except Exception as e:
            rospy.logerr(f"Initialization failed: {e}")
            # 发生异常时的清理操作
            self.close_files()

        # 定义其他的状态和订阅者
        self.define_subscribers_and_other_states()

    def __del__(self):
        self.close_files()

    def close_files(self):
        if self.traj_file:
            self.traj_file.close()
        if self.gripper_file:
            self.gripper_file.close()

    def define_subscribers_and_other_states(self):
        self.camera2_image_sub = rospy.Subscriber(self.camera2_image_topic, Image, self.camera2_image_callback)
        self.camera2_depth_sub = rospy.Subscriber(self.camera2_depth_topic, Image, self.camera2_depth_callback)
        self.joint_sub_1 = rospy.Subscriber(self.joint_topic_1, JointState, self.joint_callback_1)
        self.joint_sub_2 = rospy.Subscriber(self.joint_topic_2, JointState, self.joint_callback_2)
        self.gripper_sub_1 = rospy.Subscriber(self.gripper_topic_1, Float64MultiArray, self.gripper_callback_1)
        self.gripper_sub_2 = rospy.Subscriber(self.gripper_topic_2, Float64MultiArray, self.gripper_callback_2)
        
        # CV Bridge 和其他状态
        self.bridge = CvBridge()
        self.joint_msg_1 = None
        self.joint_msg_2 = None
        self.camera2_color_msg = None
        self.camera2_depth_msg = None
        self.gripper_msg_1 = None
        self.gripper_msg_2 = None
        self.frame_number = 0

        # 定时器
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def joint_callback_1(self, msg):
        self.joint_msg_1 = msg
    
    def joint_callback_2(self, msg):
        self.joint_msg_2 = msg
    
    def gripper_callback_1(self, msg):
        self.gripper_msg_1 = msg

    def gripper_callback_2(self, msg):
        self.gripper_msg_2 = msg

    # def camera1_image_callback(self, msg):
        # self.camera1_color_msg = msg

    # def camera1_depth_callback(self, msg):
        # self.camera1_depth_msg = msg

    def camera2_image_callback(self, msg):
        self.camera2_color_msg = msg

    def camera2_depth_callback(self, msg):
        self.camera2_depth_msg = msg

    def save_transform(self, filename, data):
        # 打开文件，'ab' 代表以二进制附加模式打开文件，保证数据在新的一行写入
        with open(filename, 'ab') as f:
            # 将数据转换为逗号分隔的格式，并添加新行字符
            np.savetxt(f, [data], delimiter=',', fmt='%.18e')

    def save_gripper_state(self, filename, data1, data2):
        # 'ab'模式同上，保证数据写入新行
        with open(filename, 'ab') as f:
            # 数据写入，其中 data1 和 data2 是可能来自不同夹爪的浮点数据数组
            formatted_data = np.column_stack((data1, data2))  # 堆叠数据
            np.savetxt(f, formatted_data, delimiter=',', fmt='%.18e')

    def timer_callback(self, event):
        if self.joint_msg_1 and self.joint_msg_2 and self.gripper_msg_1 and self.gripper_msg_2 and self.camera2_color_msg and self.camera2_depth_msg:
            try:
                # 对于机器人1
                joint_angles_1 = [self.joint_msg_1.position[self.joint_msg_1.name.index(joint)]
                                for joint in ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]]
                base_to_wrist_transform_1 = forward_kinematics(joint_angles_1)
                wrist_to_gripper_transform_1 = static_transform_wrist_to_gripper()
                base_to_gripper_transform_1 = np.dot(base_to_wrist_transform_1, wrist_to_gripper_transform_1)
                flat_transform_1 = base_to_gripper_transform_1.flatten()

                # 对于机器人2
                joint_angles_2 = [self.joint_msg_2.position[self.joint_msg_2.name.index(joint)]
                                for joint in ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]]
                base_to_wrist_transform_2 = forward_kinematics(joint_angles_2)
                wrist_to_gripper_transform_2 = static_transform_wrist_to_gripper()
                base_to_gripper_transform_2 = np.dot(base_to_wrist_transform_2, wrist_to_gripper_transform_2)
                flat_transform_2 = base_to_gripper_transform_2.flatten()

                self.save_transform(self.traj_file_path, np.concatenate((flat_transform_1, flat_transform_2)))
                self.save_gripper_state(self.gripper_file_path, self.gripper_msg_1.data, self.gripper_msg_2.data)

                # # 保存变换矩阵
                # np.savetxt(self.traj_file, [flat_transform_1, flat_transform_2], fmt='%.18e')
                # self.traj_file.flush()  # 确保数据被写入磁盘

                # # 保存夹爪状态
                # np.savetxt(self.gripper_file, [self.gripper_msg_1.data, self.gripper_msg_2.data], fmt='%.18e')
                # self.gripper_file.flush()  # 确保数据被写入磁盘

                # 处理并保存camera2的图像和深度数据
                cv_color_image2 = self.bridge.imgmsg_to_cv2(self.camera2_color_msg, desired_encoding='bgr8')
                frame_filename2 = os.path.join(self.frame2_path, f'frame{self.frame_number:06d}.jpg')
                cv2.imwrite(frame_filename2, cv_color_image2)

                cv_depth_image2 = self.bridge.imgmsg_to_cv2(self.camera2_depth_msg, desired_encoding="passthrough")
                depth_filename2 = os.path.join(self.depth2_path, f'depth{self.frame_number:06d}.png')
                cv2.imwrite(depth_filename2, cv_depth_image2)

                self.frame_number += 1  # 仅在成功操作后增加帧计数器
            except Exception as e:
                rospy.logerr(f"Error processing or saving data: {str(e)}")
        else:
            rospy.loginfo("Waiting for all messages to be received.")


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