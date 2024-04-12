#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from pupil_apriltags import Detector
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class AprilTagDetector:
    def __init__(self):
        rospy.init_node('apriltag_detector', anonymous=True)
        
        self.bridge = CvBridge()
        self.detector = None  # Initialize after receiving camera info
        self.tag_size = 0.1  # Tag size in meters
        
        # Subscribe to the camera info and image topics
        self.camera_info_sub = rospy.Subscriber('/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera_raw', Image, self.callback)
        
        self.camera_params = None

    def camera_info_callback(self, msg):
        if self.camera_params is None:
            # Initialize detector with camera parameters once camera info is received
            fx = msg.K[0]
            fy = msg.K[4]
            cx = msg.K[2]
            cy = msg.K[5]
            self.camera_params = [fx, fy, cx, cy]

            self.detector = Detector(
                families='tag25h9',
                nthreads=8,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
                debug=0,
            )
            rospy.loginfo('AprilTag detector initialized with camera parameters')

    def callback(self, data):
        if self.detector is None or self.camera_params is None:
            rospy.logwarn('Detector not initialized or camera parameters not set')
            return

        try:
            # Get the original color image
            original_color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Convert to grayscale for detection
            gray_image = cv2.cvtColor(original_color_image, cv2.COLOR_BGR2GRAY)
            # Scale down the image for faster processing
            gray_image = cv2.resize(gray_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
            original_color_image = cv2.resize(original_color_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Perform the detection
        detections = self.detector.detect(gray_image, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=self.tag_size)

        # Overlay axes on the original color image
        for detection in detections:
            # Extract the rotation matrix and translation vector from the detection
            rotation_matrix = np.array(detection.pose_R)
            translation_vector = np.array(detection.pose_t)

            # Define the 3D points of the axes
            axis_length = self.tag_size / 2
            axis_points = np.float32([
                [0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]
            ]).reshape(-1, 3)

            # Project the 3D axis points to the image plane
            camera_matrix = np.array([[self.camera_params[0], 0, self.camera_params[2]],
                                      [0, self.camera_params[1], self.camera_params[3]],
                                      [0, 0, 1]])
            dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
            axis_image_points, _ = cv2.projectPoints(
                axis_points,
                detection.pose_R,
                detection.pose_t,
                camera_matrix,
                dist_coeffs
            )

            # Draw the axes on the original color image
            origin = tuple(axis_image_points[0].ravel().astype(int))
            cv2.line(original_color_image, origin, tuple(axis_image_points[1].ravel().astype(int)), (0, 0, 255), 3)  # X-axis in red
            cv2.line(original_color_image, origin, tuple(axis_image_points[2].ravel().astype(int)), (0, 255, 0), 3)  # Y-axis in green
            cv2.line(original_color_image, origin, tuple(axis_image_points[3].ravel().astype(int)), (255, 0, 0), 3)  # Z-axis in blue

        # Display the image with the drawn axes
        cv2.imshow('AprilTag Axes', original_color_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = AprilTagDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass