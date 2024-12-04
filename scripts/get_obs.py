#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from datetime import datetime  # 导入datetime模块
import shutil

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver1', anonymous=True)

        # Subscribers
        self.camera1_image_sub = rospy.Subscriber('/camera1/color/image_raw', Image, self.camera1_image_callback)
        self.camera1_depth_sub = rospy.Subscriber('/camera1/aligned_depth_to_color/image_raw', Image, self.camera1_depth_callback)
        self.gripper_sub = rospy.Subscriber('/gripper_state', Float64MultiArray, self.gripper_callback)

        # CV Bridge
        self.bridge = CvBridge()

        # Result paths
        result_path = "/home/pine/ManiGaussian_real/data/current_obs"
        self.color_path = os.path.join(result_path, "front_rgb")
        self.depth_path = os.path.join(result_path, "front_depth")
        self.gripper_file_path = os.path.join(result_path, "gripper.txt")

        # Create directories
        os.makedirs(self.color_path, exist_ok=True)
        os.makedirs(self.depth_path, exist_ok=True)
        os.makedirs(result_path, exist_ok=True)

        # File
        self.gripper_file = open(self.gripper_file_path, "w")

        # Latest data storage
        self.latest_color_image = None
        self.latest_depth_image = None
        self.latest_gripper_data = None

        # Timer to update data at 10Hz
        rospy.Timer(rospy.Duration(0.4), self.timer_callback)

    def __del__(self):
        self.gripper_file.close()

    def camera1_image_callback(self, msg):
        self.latest_color_image = msg

    def camera1_depth_callback(self, msg):
        self.latest_depth_image = msg

    def gripper_callback(self, msg):
        self.latest_gripper_data = msg.data

    def timer_callback(self, event):
        # 获取当前时间戳并格式化为月日时分秒的字符串
        now = datetime.now()
        timestamp = now.strftime("%m%d%H%M%S") + f"{now.microsecond // 1000:03d}"  # 获取当前时间戳到毫秒

        # timestamp = datetime.now().strftime("%m%d%H%M%S")

        if self.latest_color_image:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_color_image, desired_encoding='bgr8')
            color_filename = os.path.join(self.color_path, f"{timestamp}_rgb.jpg")
            cv2.imwrite(color_filename, cv_image)

        if self.latest_depth_image:
            cv_depth = self.bridge.imgmsg_to_cv2(self.latest_depth_image, desired_encoding="passthrough")
            depth_filename = os.path.join(self.depth_path, f"{timestamp}_depth.png")
            cv2.imwrite(depth_filename, cv_depth)

        with open(self.gripper_file_path, 'w') as f:
            # Check if there is new gripper data, if not, default to 0
            current_gripper_data = self.latest_gripper_data if self.latest_gripper_data is not None else [0]
            np.savetxt(f, current_gripper_data, fmt='%.18e')

def main():
    try:
        image_saver = ImageSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
