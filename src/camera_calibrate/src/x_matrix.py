#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix, translation_matrix

# Initialize global variables to store poses
A_list = []  # List of transformations from the robot base to the end effector
B_list = []  # List of transformations from the world to the camera

def end_effector_callback(msg):
    global A_list
    # Convert PoseStamped to transformation matrix
    A = pose_to_matrix(msg.pose)
    A_list.append(A)
    rospy.loginfo("Received end effector pose")
    if len(A_list) >= 50:
        process_samples()

def camera_callback(msg):
    global B_list
    # Convert PoseStamped to transformation matrix
    B = pose_to_matrix(msg.pose)
    B_list.append(B)
    rospy.loginfo("Received camera pose")
    if len(B_list) >= 50:
        process_samples()

def pose_to_matrix(pose):
    # Convert a Pose message to a 4x4 transformation matrix
    translation = translation_matrix((pose.position.x, pose.position.y, pose.position.z))
    rotation = quaternion_matrix((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    return np.dot(translation, rotation[:3, :3])  # Only rotation and translation part

def process_samples():
    global A_list, B_list
    if len(A_list) >= 50 and len(B_list) >= 50:
        # Take the first 50 samples
        A_samples = A_list[:50]
        B_samples = B_list[:50]
        # Clear the lists for future samples
        A_list = A_list[50:]
        B_list = B_list[50:]
        # Perform hand-eye calibration
        calibrate_hand_eye(A_samples, B_samples)

def calibrate_hand_eye(A, B):
    # Convert transformation matrices to rotation vectors and translation vectors
    R_gripper2base = [cv2.Rodrigues(matrix[:3,:3])[0] for matrix in A]
    t_gripper2base = [matrix[:3,3] for matrix in A]
    R_target2cam = [cv2.Rodrigues(matrix[:3,:3])[0] for matrix in B]
    t_target2cam = [matrix[:3,3] for matrix in B]

    # Perform hand-eye calibration
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI
    )
    
    # Convert rotation vector to rotation matrix
    R_cam2gripper_matrix, _ = cv2.Rodrigues(R_cam2gripper)

    # Print out the result
    rospy.loginfo("Real-time R_cam2gripper:\n{}".format(R_cam2gripper_matrix))
    rospy.loginfo("Real-time t_cam2gripper:\n{}".format(t_cam2gripper))

if __name__ == '__main__':
    rospy.init_node('hand_eye_calibration')

    # Initialize the subscribers
    rospy.Subscriber('/end_effector_pose', PoseStamped, end_effector_callback)
    rospy.Subscriber('/camera/pose', PoseStamped, camera_callback)

    rospy.spin()