#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

###############################################################################################
# This node is for publishing a plane marker to the rviz simulation for the Click&Go feature.
###############################################################################################

def create_plane_marker():
    plane_marker = Marker()
    plane_marker.header.frame_id = "base_link"  # Set the frame ID
    plane_marker.ns = "plane"
    plane_marker.id = 0
    plane_marker.type = Marker.CYLINDER
    plane_marker.action = Marker.ADD
    plane_marker.pose.position.x = 0
    plane_marker.pose.position.y = 0
    plane_marker.pose.position.z = 0    # z-coordinate of the plane
    plane_marker.pose.orientation.w = 1
    plane_marker.scale.x = 0.6          # size of the plane (x-dimension)
    plane_marker.scale.y = 0.6          # size of the plane (y-dimension)
    plane_marker.scale.z = 0.001        # thickness of the plane
    plane_marker.color.r = 0.0
    plane_marker.color.g = 1.0
    plane_marker.color.b = 0.0
    plane_marker.color.a = 0.5          # transparency of the plane

    return plane_marker

if __name__ == '__main__':
    rospy.init_node('plane_marker_node')

    plane_marker = create_plane_marker()

    marker_pub = rospy.Publisher('plane_marker', Marker, queue_size=1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        plane_marker.header.stamp = rospy.Time.now()
        marker_pub.publish(plane_marker)
        rate.sleep()
