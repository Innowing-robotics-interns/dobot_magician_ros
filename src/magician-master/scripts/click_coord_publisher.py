#!/usr/bin/env python

################################################################################
# Click the keyboard shortcut key 'c' to publish end effector coordinates.
################################################################################

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

global x,y,z
def callback(msg): 

    coord_pub = rospy.Publisher('/end_effector_coord', Twist, queue_size=1)

    twist = Twist()
    twist.linear.x = -msg.point.x * 1000
    twist.linear.y = -msg.point.y * 1000
    twist.linear.z = msg.point.z * 1000

    coord_pub.publish(twist)

    rospy.loginfo("Published end effector pose: \n%s", twist)

if __name__ == '__main__':
    rospy.init_node('goal_publisher', anonymous=True)
    rospy.point_pub = rospy.Subscriber('/clicked_point', PointStamped, callback)

    rospy.spin()