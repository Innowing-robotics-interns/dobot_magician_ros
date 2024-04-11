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
        self.detector = None  # We'll initialize this after receiving camera info
        self.tag_size = 0.1  # Tag size in meters
        
        # Subscribe to the camera info and image topics
        self.camera_info_sub = rospy.Subscriber('/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera_raw', Image, self.callback)
        
        self.camera_params = None

    def camera_info_callback(self, msg):
        # Initialize the detector with camera parameters once camera info is received
        if self.camera_params is None:
            fx = msg.K[0]
            fy = msg.K[4]
            cx = msg.K[2]
            cy = msg.K[5]
            self.camera_params = [fx, fy, cx, cy]

            self.dist_coeffs = np.array(msg.D)
            self.detector = Detector(
                families='tag25h9',
                nthreads=1,
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
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        detections = self.detector.detect(cv_image)
        color_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        for detection in detections:
            rospy.loginfo('Detected tag ID: {}, Center: {}'.format(detection.tag_id, detection.center))

            # Define 3D object points of the AprilTag in its local coordinate system.
            object_points = np.array([
                [-self.tag_size / 2, -self.tag_size / 2, 0],
                [self.tag_size / 2, -self.tag_size / 2, 0],
                [self.tag_size / 2, self.tag_size / 2, 0],
                [-self.tag_size / 2, self.tag_size / 2, 0]
            ])

            # Extract the corner points from the detection message.
            image_points = np.array(detection.corners, dtype='double')

            # Convert the camera parameters and the image points to the correct shape.
            camera_matrix = np.array([[self.camera_params[0], 0, self.camera_params[2]],
                                    [0, self.camera_params[1], self.camera_params[3]],
                                    [0, 0, 1]])
            dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

            # Solve for the pose of the AprilTag.
            success, rotation_vector, translation_vector = cv2.solvePnP(
                object_points, image_points, camera_matrix, dist_coeffs
            )

            if success:
                # Convert rotation vector to a rotation matrix.
                rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
                T = np.eye(4)
                T[:3, :3] = rotation_matrix
                T[:3, 3] = translation_vector.flatten()
                rospy.loginfo('Homogeneous Transformation Matrix from tag to camera:\n{}'.format(T))

                # Define the axis length (could be the same as the tag size or shorter).
                axis_length = self.tag_size / 2

                # Define the 3D points of the axes.
                axis_points = np.float32([
                    [0, 0, 0],  # Origin point
                    [axis_length, 0, 0],  # X-axis point
                    [0, axis_length, 0],  # Y-axis point
                    [0, 0, -axis_length]  # Z-axis point (negative to point out of the tag)
                ]).reshape(-1, 3)

                # Project the 3D axis points to the image plane.
                axis_image_points, _ = cv2.projectPoints(
                    axis_points,
                    rotation_vector,
                    translation_vector,
                    camera_matrix,
                    dist_coeffs
                )

                # Draw the axes on the image.
                origin = tuple(axis_image_points[0].ravel().astype(int))
                cv2.line(color_image, origin, tuple(axis_image_points[1].ravel().astype(int)), (0, 0, 255), 3)  # X-axis in red
                cv2.line(color_image, origin, tuple(axis_image_points[2].ravel().astype(int)), (0, 255, 0), 3)  # Y-axis in green
                cv2.line(color_image, origin, tuple(axis_image_points[3].ravel().astype(int)), (255, 0, 0), 3)  # Z-axis in blue

                rospy.loginfo('Axes drawn on the tag ID: {}'.format(detection.tag_id))

        # Display the image with the drawn axes.
        cv2.imshow('AprilTag Axes', color_image)
        cv2.waitKey(1)  # Wait for a key press or use a small delay for video stream.

def main():
    try:
        tag_detector = AprilTagDetector()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down AprilTag detector node.')
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()