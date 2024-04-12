#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from pupil_apriltags import Detector
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix

class AprilTagDetector:
    def __init__(self):
        rospy.init_node('apriltag_detector', anonymous=True)
        
        self.bridge = CvBridge()
        self.detector = None
        self.tag_size = 0.1  # Tag size in meters
        self.pose_publisher = rospy.Publisher('/tag_pose', PoseStamped, queue_size=10)
        
        self.camera_info_sub = rospy.Subscriber('/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera_raw', Image, self.callback)
        
        self.camera_params = None

    def camera_info_callback(self, msg):
        if self.camera_params is None:
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
            original_color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray_image = cv2.cvtColor(original_color_image, cv2.COLOR_BGR2GRAY)
            gray_image = cv2.resize(gray_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
            original_color_image = cv2.resize(original_color_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        detections = self.detector.detect(gray_image, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=self.tag_size)

        for detection in detections:
            rotation_matrix = np.array(detection.pose_R)
            translation_vector = np.array(detection.pose_t)

            # Construct the homogeneous transformation matrix
            homogeneous_matrix = np.eye(4)
            homogeneous_matrix[:3, :3] = rotation_matrix
            homogeneous_matrix[:3, 3] = translation_vector.ravel()
            print("Homogeneous Transformation Matrix:\n", homogeneous_matrix)

            # Overlay axes on the image
            axis_length = self.tag_size / 2
            axis_points = np.float32([[0, 0, 0], [axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]]).reshape(-1, 3)
            camera_matrix = np.array([[self.camera_params[0], 0, self.camera_params[2]], [0, self.camera_params[1], self.camera_params[3]], [0, 0, 1]])
            dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion
            axis_image_points, _ = cv2.projectPoints(axis_points, rotation_matrix, translation_vector, camera_matrix, dist_coeffs)

            origin = tuple(axis_image_points[0].ravel().astype(int))
            cv2.line(original_color_image, origin, tuple(axis_image_points[1].ravel().astype(int)), (0, 0, 255), 3)  # X-axis in red
            cv2.line(original_color_image, origin, tuple(axis_image_points[2].ravel().astype(int)), (0, 255, 0), 3)  # Y-axis in green
            cv2.line(original_color_image, origin, tuple(axis_image_points[3].ravel().astype(int)), (255, 0, 0), 3)  # Z-axis in blue
            
            # Publish the pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = translation_vector[0]
            pose_msg.pose.position.y = translation_vector[1]
            pose_msg.pose.position.z = translation_vector[2]
            quaternion = quaternion_from_matrix(homogeneous_matrix)
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            self.pose_publisher.publish(pose_msg)

        cv2.imshow('AprilTag Axes', original_color_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = AprilTagDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass