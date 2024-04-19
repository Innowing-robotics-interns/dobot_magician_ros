#!/usr/bin/env python

############################################################################
# This node is for controlling the robot arm's joint states (sim) using
# a mapping from the /end_effector_coord Twist message to joint positions.
############################################################################

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

pose = [0, 0, 0, 0, 0]

def publish_joint_states():
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=5)
    rate = rospy.Rate(10)  # Set the publishing rate to 10 Hz
    prev_joint_positions = None

    def has_joint_changed(current_pose):
        nonlocal prev_joint_positions
        if prev_joint_positions is None:
            return True
        # Compare current joint positions with previous ones
        for i in range(0, 5):  # Assuming joints 0 to 4 (adjust as needed)
            if abs(current_pose[i] - prev_joint_positions[i]) > 0.01:  # Tolerance for change
                return True
        return False

    while not rospy.is_shutdown():
        # Check if joint positions have changed
        if prev_joint_positions is None or has_joint_changed(pose):
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['joint_1', 'joint_2', 'joint_5', 'joint_6', 'joint_7']  # Replace with your joint names

            # Map '/end_effector_coord' Twist message to joint positions
            joint_state_msg.position = [
                pose[0],  # Map Twist linear x to joint_1 position
                pose[1],  # Map Twist linear y to joint_2 position
                pose[2],  # Map Twist linear z to joint_5 position
                pose[3],  # Map Twist angular z to joint_6 position
                pose[4]   # Map Twist linear x to joint_7 position
            ]

            joint_state_msg.velocity = []  # Replace with actual joint velocities
            joint_state_msg.effort = []  # Replace with actual joint efforts

            joint_pub.publish(joint_state_msg)

            # Print the pose in 4 decimal places
            rospy.loginfo(f"Joint target (deg): {[round((x * 180 / 3.14159265359)%360, 4) for x in joint_state_msg.position]}")

            prev_joint_positions = pose
            

        rate.sleep()

def move_robot_callback(msg):
    global pose
    # pose = [msg.angular.z, msg.linear.z, msg.linear.x, msg.linear.y, 0]
    pose = [msg.angular.z, msg.linear.z, msg.linear.x, 0, 0]
    # pose[3] = 1.570796325 - (1.570796325 - (pose[2] - pose[1])) - 1.570796325/2 + 0.185
    pose[3] = (pose[2] - pose[1]) - 1.570796325/2 + 0.185   # 1.85 is just a constant offset (can tune)

if __name__ == '__main__':
    rospy.init_node('joystick_sim_control_node', anonymous=True)

    # Note: The /end_effector_coord topic is interpreted as joint angles in this sim!
    rospy.Subscriber('/end_effector_coord', Twist, move_robot_callback)
    publish_joint_states()