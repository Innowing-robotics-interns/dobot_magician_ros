#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import yaml

class ImageUndistorter:
    def __init__(self):
        rospy.init_node('image_undistorter', anonymous=True)
        self.bridge = CvBridge()

        # Load camera calibration parameters from YAML file
        self.load_camera_calibration()

        self.image_subscriber = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.undistorted_image_publisher = rospy.Publisher('/camera/image_undistorted', Image, queue_size=10)

    def load_camera_calibration(self):
        with open('/home/oem/Documents/Code/robotic arm/ROS Framework/catkin_ws/src/dobot_ros/src/camera_calibration.yaml', 'r') as f:
            calib_data = yaml.safe_load(f)

        self.camera_matrix = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
        self.distortion_coefficients = np.array(calib_data['distortion_coefficients']['data'])

    def image_callback(self, ros_image):
        # Convert ROS image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')

        # Undistort the image
        undistorted_image = cv2.undistort(cv_image, self.camera_matrix, self.distortion_coefficients)

        # Convert the undistorted image back to ROS image message
        undistorted_ros_image = self.bridge.cv2_to_imgmsg(undistorted_image, encoding='bgr8')

        # Publish the undistorted image
        self.undistorted_image_publisher.publish(undistorted_ros_image)

if __name__ == '__main__':
    undistorter = ImageUndistorter()
    rospy.spin()