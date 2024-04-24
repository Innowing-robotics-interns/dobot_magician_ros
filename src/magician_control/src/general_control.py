#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
from pydobot import Dobot
from serial.tools import list_ports

####################################################################################
# This node is for controlling the robot arm's end effector coordinate (real), 
# and providing the feedback of joint states to the rviz simulation.
####################################################################################

initial_coord = [200, 0, 0, 0]
coord_command = initial_coord
vacuumPumpOn = False

def publish_joint_states():
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
    # coord_pub = rospy.Publisher('/end_effector_coord', Twist, queue_size=1)
    rate = rospy.Rate(100)  # Set the publishing rate to 100 Hz
    prev_joint_positions = None
    global coord_command
    prev_command = coord_command

    def has_joint_changed(current_pose):
        nonlocal prev_joint_positions
        if prev_joint_positions is None:
            return True
        # Compare current joint positions with previous ones
        for i in range(0, 4):  # Assuming joints 0 to 4 (adjust as needed)
            if abs(current_pose[i] - prev_joint_positions[i]) > 0.01:  # Tolerance for change
                return True
        return False
    def has_coordinate_changed(current_pose, prev_command):
        if prev_command is None:
            return True

        # Compare current joint positions with previous ones
        for i in range(0, 4):  # Assuming joints 0 to 4 (adjust as needed)
            if abs(current_pose[i] - prev_command[i]) > 0.01:  # Tolerance for change
                return True
        return False

    while not rospy.is_shutdown():
        coord_command = [round(x, 4) for x in coord_command]
        if has_coordinate_changed(coord_command, prev_command):
            # print the pose in 2 decimal places
            rospy.loginfo(f"End effector target: {[round(x, 2) for x in coord_command]}")
            arm.move_to(coord_command[0], coord_command[1], coord_command[2], coord_command[3])
            
            alarms = arm.get_alarms()
            # set a timeout for the alarms
            timeout = 5
            start_time = rospy.get_time()

            # If have alarm and not timeout, reset to previous positions 
            while len(alarms) != 0 and rospy.get_time() - start_time < timeout:
                # arm.clear_alarms()
                print(f"The new pose of coord {coord_command} has alarms: {alarms}")
                print(f"Resetting the previous positions to {prev_command}")
                arm.move_to(prev_command[0], prev_command[1], prev_command[2], prev_command[3])
                # rospy.sleep(0.2)  # Wait for the arm to move
                # arm.clear_alarms()
                alarms = arm.get_alarms()
            # If still have alarms after timeout, restore initial positions
            if len(alarms) != 0:
                arm.clear_alarms()
                print(f"Restore initial coord: {initial_coord}")
                arm.move_to(initial_coord[0], initial_coord[1], initial_coord[2], initial_coord[3])
                alarms = arm.get_alarms()
                coord_command = initial_coord

            prev_command = coord_command
        # else:
            # rospy.loginfo(f"new coord vs prev coord: {coord_command} vs {prev_command}")
        
        # Update the vacuum pump state
        # arm.suck(bool(vacuumPumpOn))      # for the suction cup
        arm.grip(bool(vacuumPumpOn))      # for the gripper
        
        # Publish the coord command, because it maybe updated by other nodes
        # twist = Twist()
        # twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z = coord_command
        # twist.angular.x = vacuumPumpOn
        # coord_pub.publish(twist)

        pose = arm.get_pose().joints
        # Check if joint positions have changed
        if prev_joint_positions is None or has_joint_changed(pose):
            # sim follows the real robot's joint states
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['joint_1', 'joint_2', 'joint_5', 'joint_6', 'joint_7']  # Replace with your joint names
            joint_state_msg.position = [math.radians(pose.j1),
                                         math.radians(pose.j2),
                                         math.radians(90 - pose.j3 + pose.j2),
                                        #  math.radians(pose.j4),
                                         0,
                                         0]  # Replace with actual joint positions
            joint_state_msg.position[3] = (joint_state_msg.position[2] - joint_state_msg.position[1]) - 1.570796325/2 + 0.185   # 1.85 is just a constant offset (can tune)
            joint_state_msg.velocity = []  # Replace with actual joint velocities
            joint_state_msg.effort = []  # Replace with actual joint efforts

            joint_pub.publish(joint_state_msg)
            prev_joint_positions = pose
    

        rate.sleep()

def move_robot_callback(msg):
    # global prev_pose
    global coord_command, vacuumPumpOn
    coord_command = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]

    vacuumPumpOn = msg.angular.x

    # go to initial coord if coord_command is 0
    if coord_command[0] == 0 and coord_command[1] == 0 and coord_command[2] == 0 and coord_command[3] == 0:
        coord_command = [200, 0, 0, 0]


if __name__ == '__main__':
    rospy.init_node('general_control_node')

    # Find available ports
    available_ports = list_ports.comports()
    print(f'Available ports: {[x.device for x in available_ports]}')

    # `sudo chown -R $USER:$USER /dev/*`
    port = None
    for port in available_ports:
        if 'ttyACM' in port.device:     # select the first available port
            print(f"Selected port: {port.device}")
            port = port.device
            break
    if port is None:
        raise IOError("No Dobot port found.")
    
    global arm
    arm = Dobot(port=port)

    arm.suck(False)
    arm.grip(False)
    # arm.move_to(180.0, 0, 50, 0)

    arm.home()

    global prev_pose
    prev_pose = None
    rospy.Subscriber('/end_effector_coord', Twist, move_robot_callback, queue_size=1)
    publish_joint_states()
