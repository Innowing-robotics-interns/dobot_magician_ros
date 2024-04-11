#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import yaml

class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher', anonymous=True)
        self.bridge = CvBridge()

        # Load camera calibration parameters from YAML file
        self.load_camera_calibration()

        # Set up publishers for the raw image and camera info
        self.raw_image_publisher = rospy.Publisher('/camera_raw', Image, queue_size=10)
        self.camera_info_publisher = rospy.Publisher('/camera_info', CameraInfo, queue_size=10)

        # Video capture setup
        self.camera_model = 4  # Replace with the appropriate camera model number
        self.video_capture = cv2.VideoCapture(self.camera_model)

        if not self.video_capture.isOpened():
            rospy.logerr('Failed to open the camera')
            rospy.signal_shutdown('Failed to open the camera')
        
        self.rate = rospy.Rate(30)  # Set to the desired rate

    def load_camera_calibration(self):
        calibration_file_path = '/home/dethcube/Documents/Code/dobot/catkin_ws/dobot_ros/src/camera_calibrate/src/camera_calibration.yaml'
        with open(calibration_file_path, 'r') as f:
            calib_data = yaml.safe_load(f)

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "map"  # Set frame_id to "map"
        self.camera_info_msg.width = int(calib_data['image_width'])
        self.camera_info_msg.height = int(calib_data['image_height'])
        self.camera_info_msg.K = calib_data['camera_matrix']['data']
        self.camera_info_msg.D = calib_data['distortion_coefficients']['data']
        self.camera_info_msg.R = calib_data['rectification_matrix']['data'] if 'rectification_matrix' in calib_data else [0]*9
        self.camera_info_msg.P = calib_data['projection_matrix']['data'] if 'projection_matrix' in calib_data else self.camera_info_msg.K + [0]*12

    def publish_video_stream(self):
        while not rospy.is_shutdown():
            ret, frame = self.video_capture.read()
            if ret:
                # Publish the raw image and camera info
                ros_image_raw = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                ros_image_raw.header.stamp = rospy.Time.now()
                ros_image_raw.header.frame_id = "map"  # Set frame_id to "map"
                self.camera_info_msg.header.stamp = ros_image_raw.header.stamp
                self.raw_image_publisher.publish(ros_image_raw)
                self.camera_info_publisher.publish(self.camera_info_msg)
            else:
                rospy.logerr('Failed to read frame from the camera')

            self.rate.sleep()

        # Release the video capture and shutdown the node
        self.video_capture.release()

if __name__ == '__main__':
    camera_publisher = CameraPublisher()
    try:
        camera_publisher.publish_video_stream()
    except rospy.ROSInterruptException:
        pass