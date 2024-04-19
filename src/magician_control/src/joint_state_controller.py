#!/usr/bin/env python

#######################################################################################################################
# This node is for controlling the robot arm's joint states (real).
# It needs to be run along with the slider bar control using joint_state_publisher_gui, enable it in display.launch.
#######################################################################################################################

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from pydobot import Dobot
from serial.tools import list_ports
import numpy as np

PI = np.pi

initial_positions = [0, 0, 30, 0]
previous_positions = [0, 0, 30, 0]

# def print_pose(pose):
#     # device.get_pose().joints
#     # only print when pose changes
#     alarms = arm.get_alarms()
#     new_pose = arm.get_pose().joints
#     if pose != new_pose:
#         print(pose)
#         if alarms != set():
#             print(alarms)
#         pose = new_pose

def joint_states_callback(msg):
    global previous_positions
    global pose_publisher

    positions = msg.position

    # positions are in radians, convert to degrees, and round to 3 decimal places
    positions = [round(x * 180 / PI, 3) for x in positions]

    # print("Joint Positions:", positions)

    if positions == previous_positions:
        return  # Skip printing and moving to the same position

    print("Joint target (in degrees):", positions[:4])
    dobot_pose = arm.get_pose()
    print(f"Current end effector pose: {dobot_pose}")

    # Publish the end effector pose
    pose_msg = Pose()
    pose_msg.position.x = dobot_pose.position.x
    pose_msg.position.y = dobot_pose.position.y
    pose_msg.position.z = dobot_pose.position.z
    # Default orientation, assuming no rotation information is available
    pose_msg.orientation.x = 0
    pose_msg.orientation.y = 0
    pose_msg.orientation.z = 0
    pose_msg.orientation.w = 1
    pose_publisher.publish(pose_msg)

    arm.rotate_joint(positions[0], positions[1], 90 - (positions[2]-positions[1]), positions[3])

    rospy.sleep(1.5)  # Wait for the arm to move

    alarms = arm.get_alarms()
    # set a timeout for the alarms
    timeout = 10
    start_time = rospy.get_time()

    # If have alarm and not timeout, reset to previous positions 
    while alarms and rospy.get_time() - start_time < timeout:
        print(f"The new pose of joint {positions} has alarms: {alarms}")
        print(f"Resetting the previous positions to {previous_positions}")
        arm.rotate_joint(previous_positions[0], previous_positions[1], 90 - previous_positions[2], previous_positions[3])
        rospy.sleep(0.5)  # Wait for the arm to move
        alarms = arm.get_alarms()
    
    # If still have alarms after timeout, restore initial positions
    if alarms:
        print(f"Restore initial positions: {initial_positions}")
        arm.rotate_joint(initial_positions[0], initial_positions[1], 90 - initial_positions[2], initial_positions[3])

    print(f"The new pose of joint {positions} has no alarms")
    previous_positions = positions

    print(f"Actual joint positions: {arm.get_pose().joints[:4]}")

    print()

if __name__ == '__main__':
    rospy.init_node('joint_state_controller')

    # Find available ports
    available_ports = list_ports.comports()
    print(f'Available ports: {[x.device for x in available_ports]}')

    # Select the first available port
    port = None
    for port_info in available_ports:
        if 'ttyACM' in port_info.device:
            print(f"Selected port: {port_info.device}")
            port = port_info.device
            break
    
    if port is None:
        raise IOError("No Dobot port found.")
    
    arm = Dobot(port=port)
    arm.home()
 
    # original_pose = arm.get_pose().joints
    # print_pose(original_pose)

    # Publisher for the end effector pose
    pose_publisher = rospy.Publisher('/end_effector_pose', Pose, queue_size=10)

    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    rospy.spin()