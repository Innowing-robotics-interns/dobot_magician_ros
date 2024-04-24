#!/usr/bin/env python

################################################################################
# Click the keyboard shortcut key 'c' to publish end effector coordinates.
################################################################################

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

global x,y,z
twist = Twist()

def click_callback(msg): 
    global twist

    coord_pub = rospy.Publisher('/end_effector_coord', Twist, queue_size=1)

    twist.linear.x = -msg.point.x * 1000
    twist.linear.y = -msg.point.y * 1000

    coord_pub.publish(twist)
    print(f"(Clicked!) Published end effector pose: {twist}")

    rospy.loginfo("Published end effector pose: \n%s", twist)

def callback(msg):
    global twist
    twist = msg
    print(f"Current end effector pose: {twist}")

if __name__ == '__main__':
    rospy.init_node('goal_publisher', anonymous=True)
    rospy.point_pub = rospy.Subscriber('/clicked_point', PointStamped, click_callback)
    # subscribe to the topic '/end_effector_coord' to get the current end effector pose
    rospy.Subscriber('/end_effector_coord', Twist, callback, queue_size=1)

    rospy.spin()