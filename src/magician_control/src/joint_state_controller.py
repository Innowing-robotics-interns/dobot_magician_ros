#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from pydobot import Dobot
from serial.tools import list_ports
import numpy as np

PI = np.pi

initial_positions = [0, 0, 30, 0]
previous_positions = [0, 0, 30, 0]

def joint_states_callback(msg):
    global previous_positions

    positions = msg.position

    # positions are in radians, convert to degrees, and round to 3 decimal places
    positions = [round(x * 180 / PI, 3) for x in positions]

    # print("Joint Positions:", positions)

    if positions == previous_positions:
        return  # Skip printing and moving to the same position

    print("Joint target (in degrees):", positions[:4])

    arm.rotate_joint(positions[0], positions[1], 90 - positions[2], positions[3])

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
    print()


if __name__ == '__main__':
    rospy.init_node('joint_state_controller')

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

    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    rospy.spin()

