#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import math
from pydobot import Dobot
from serial.tools import list_ports

class JointStateFeedbackNode:
    def __init__(self):
        rospy.init_node('joint_state_feedback_node', anonymous=True)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(10)  # Set the publishing rate (adjust as needed)

        self.prev_joint_positions = None

    def publish_joint_states(self):
        while not rospy.is_shutdown():
            pose = arm.get_pose().joints

            # Check if joint positions have changed
            if self.prev_joint_positions is None or self._has_joint_changed(pose):
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = rospy.Time.now()
                joint_state_msg.name = ['joint_1', 'joint_2', 'joint_5', 'joint_6', 'joint_7']  # Replace with your joint names

                joint_state_msg.position = [math.radians(pose.j1), math.radians(pose.j2), math.radians(90 - pose.j3), math.radians(pose.j4), 0]  # Replace with actual joint positions
                joint_state_msg.velocity = []  # Replace with actual joint velocities
                joint_state_msg.effort = []  # Replace with actual joint efforts

                self.joint_pub.publish(joint_state_msg)
                self.prev_joint_positions = pose

            self.rate.sleep()

    def _has_joint_changed(self, current_pose):
        if self.prev_joint_positions is None:
            return True

        # Compare current joint positions with previous ones
        for i in range(1, 4):  # Assuming joints 1 to 4 (adjust as needed)
            if abs(current_pose[i] - self.prev_joint_positions[i]) > 0.01:  # Tolerance for change
                return True

        return False

if __name__ == '__main__':
    # Find available ports
    available_ports = list_ports.comports()
    print(f'Available ports: {[x.device for x in available_ports]}')

    # Select the first available port
    port = None
    for port in available_ports:
        if 'ttyACM' in port.device:
            print(f"Selected port: {port.device}")
            port = port.device
            break

    global arm
    arm = Dobot(port=port)

    arm.home()

    try:
        node = JointStateFeedbackNode()
        node.publish_joint_states()
    except rospy.ROSInterruptException:
        pass
