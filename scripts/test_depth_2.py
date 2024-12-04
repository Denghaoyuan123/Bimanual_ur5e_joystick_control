#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class Camera2DepthSaver:
    def __init__(self, result_path):
        rospy.init_node('camera2_depth_saver', anonymous=True)
        
        # Parameters
        self.camera2_depth_topic = rospy.get_param('~camera2_depth_topic', '/camera2/aligned_depth_to_color/image_raw')
        self.result_path = rospy.get_param('~result_path', os.path.expanduser(result_path))

        # Paths
        self.depth2_path = os.path.join(self.result_path, "camera2_depths")
        os.makedirs(self.depth2_path, exist_ok=True)

        # CV Bridge
        self.bridge = CvBridge()
        
        # State
        self.camera2_depth_msg = None
        self.frame_number = 0
        self.test = True
        
        # Subscribers
        self.camera2_depth_sub = rospy.Subscriber(self.camera2_depth_topic, Image, self.camera2_depth_callback)

        # Timer
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def camera2_depth_callback(self, msg):
        self.camera2_depth_msg = msg

    def timer_callback(self, event):
        if self.camera2_depth_msg:
            try:
                # Process and save depth images for camera2
                cv_depth_image2 = self.bridge.imgmsg_to_cv2(self.camera2_depth_msg, desired_encoding="passthrough")
                depth_filename2 = os.path.join(self.depth2_path, f'depth{self.frame_number:06d}.png')
                cv2.imwrite(depth_filename2, cv_depth_image2)

                # Display and analyze the depth image
                while self.test:
                    print("Depth review for camera2")
                    cv2.imshow("Depth Camera2", cv_depth_image2)
                    depth = cv2.imread(depth_filename2, cv2.IMREAD_UNCHANGED)

                    # Exclude all zero values
                    nonzero_depth = depth[depth > 0]
                    if nonzero_depth.size == 0:
                        print("No non-zero depth values found.")
                        continue

                    # Find the minimum non-zero depth value
                    min_depth = np.min(nonzero_depth)
                    print(f"Minimum depth value (excluding zeros): {min_depth}mm")

                    # Define bins for depth distribution
                    bins = [1, 200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800, 2000, np.inf]
                    # Compute the histogram
                    hist, bin_edges = np.histogram(nonzero_depth, bins=bins)
                    print("Depth distribution for camera2:")
                    for i in range(len(bins) - 1):
                        print(f"{bins[i]}mm to {bins[i+1]}mm: {hist[i]}")

                    self.test = False

                self.frame_number += 1  # Increment frame counter after successful operations
            except Exception as e:
                rospy.logerr(f"Error processing or saving camera2 depth data: {str(e)}")

def main():
    result_path = "/home/pine/Mani_data"  # Modify as needed
    try:
        depth_saver = Camera2DepthSaver(result_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
