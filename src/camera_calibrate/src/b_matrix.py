#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_from_matrix
import cv2
import numpy as np
import yaml
import threading

global image_pub, pose_pub, frame_lock, latest_frame

frame_lock = threading.Lock()
latest_frame = None

def load_camera_calibration(calibration_file):
    with open(calibration_file, 'r') as file:
        calib_data = yaml.safe_load(file)
    camera_matrix = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
    dist_coeffs = np.array(calib_data['distortion_coefficients']['data']).reshape(1, 5)
    return camera_matrix, dist_coeffs

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img

def image_callback(msg):
    global latest_frame
    try:
        with frame_lock:
            latest_frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def process_latest_frame():
    global latest_frame
    while not rospy.is_shutdown():
        frame=None
        with frame_lock:
            if latest_frame is not None:
                frame = latest_frame.copy()
                latest_frame = None
        if frame is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
            if ret:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, camera_matrix, dist_coeffs)
                if ret:
                    imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, camera_matrix, dist_coeffs)
                    frame = draw(frame, corners2, imgpts)
            try:
                image_message = CvBridge().cv2_to_imgmsg(frame, "bgr8")
                image_pub.publish(image_message)
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
        rospy.sleep(0.01)

def main():
    global image_pub, pose_pub
    rospy.init_node('camera_pose_estimation')
    image_pub = rospy.Publisher('/camera/image_with_axes', Image, queue_size=1)
    pose_pub = rospy.Publisher('/camera/pose', Pose, queue_size=1)
    rospy.Subscriber('/camera/image_raw', Image, image_callback)
    processing_thread = threading.Thread(target=process_latest_frame)
    processing_thread.start()
    rospy.spin()
    processing_thread.join()

if __name__ == '__main__':
    camera_calibration_file = '/home/dethcube/Documents/Code/dobot/catkin_ws/dobot_ros/src/camera_calibrate/src/camera_calibration.yaml'
    camera_matrix, dist_coeffs = load_camera_calibration(camera_calibration_file)
    # checkerboard_size = (9, 6)
    checkerboard_size = (3, 3)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((checkerboard_size[0]*checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1, 3)
    try:
        main()
    except rospy.ROSInterruptException:
        pass