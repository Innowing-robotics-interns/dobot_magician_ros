#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_video_stream(camera_model, topic_name):
    rospy.init_node('video_publisher', anonymous=True)
    video_capture = cv2.VideoCapture(camera_model)
    bridge = CvBridge()
    image_publisher = rospy.Publisher(topic_name, Image, queue_size=10)

    # Check if video capture is successful
    if not video_capture.isOpened():
        rospy.logerr('Failed to open the camera')
        return

    rate = rospy.Rate(60)  # Publish images at 30Hz

    while not rospy.is_shutdown():
        ret, frame = video_capture.read()
        if ret:
            # Convert the frame to ROS image message
            ros_image = bridge.cv2_to_imgmsg(frame, encoding='bgr8')

            # Publish the ROS image message
            image_publisher.publish(ros_image)
        else:
            rospy.logerr('Failed to read frame from the camera')

        rate.sleep()

    # Release the video capture and shutdown the node
    video_capture.release()
    rospy.signal_shutdown('Video stream publishing finished')

if __name__ == '__main__':
    camera_model = 2  # Replace with the appropriate camera model number
    topic_name = '/camera/image_raw'  # Replace with the desired image topic name
    publish_video_stream(camera_model, topic_name)