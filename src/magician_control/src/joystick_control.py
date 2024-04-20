#!/usr/bin/env python

###################################################################################
# This node is for publishing to the /end_effector_coord topic using a joystick.
###################################################################################

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

initialCoordinates = [202, 0, 0, 0]
prev_x, prev_y, prev_z, prev_r = initialCoordinates

vacuumPumpOn = 0
settingsUpdated = False
timeout = 0.5

prevTwist = Twist()

def joystick_callback(data):
    global prev_x, prev_y, prev_z, prev_r, prevTwist, vacuumPumpOn, settingsUpdated
    global start_time, timeout

    x = data.axes[1]  # Horizontal movement (x-coordinate)
    y = data.axes[0]  # Vertical movement (y-coordinate)
    z = data.axes[3]  # Altitude (z-coordinate)
    r = data.axes[2]  # Rotation of end effector

    # Reset if data.buttons[5] is pressed
    if data.buttons[5] == 1:
        prev_x, prev_y, prev_z, prev_r = initialCoordinates
        settingsUpdated = True

    # print("time passed: ", rospy.get_time() - start_time)
    if data.buttons[7] == 1 and rospy.get_time() - start_time > timeout:
        # can only click button <7> every <timeout> second
        start_time = rospy.get_time()
        print("clicked button 7")
        # flip the vacuum pump state
        vacuumPumpOn = not vacuumPumpOn

        print(f"Vacuum Pump: {vacuumPumpOn}")
        settingsUpdated = True
        

    # Apply sensitivity and map to coordinate changes
    DEADZONE = 0.05
    x_change = 0 if abs(x) < DEADZONE else sensitivity * x
    y_change = 0 if abs(y) < DEADZONE else sensitivity * y
    z_change = 0 if abs(z) < DEADZONE else sensitivity * z
    r_change = 0 if abs(r) < DEADZONE else sensitivity * r

    # Update previous values
    prev_x += x_change
    prev_y += y_change
    prev_z += z_change
    prev_r += r_change

    # only publish if there is a change in the coordinates
    if settingsUpdated or x_change != 0 or y_change != 0 or z_change != 0 or r_change != 0:
      # Create a Twist message
      twist = Twist()
      twist.linear.x = prev_x
      twist.linear.y = prev_y
      twist.linear.z = prev_z
      twist.angular.z = prev_r

      twist.angular.x = vacuumPumpOn
      
      # Publish the Twist message
      pub.publish(twist)
      prevTwist = twist
    # else:
      # pub.publish(prevTwist)
    
    settingsUpdated = False

# Update the current actual coordinates, enable 
# further incrementation of coordinate using joystick
def coord_callback(data):
    global prev_x, prev_y, prev_z, prev_r
    prev_x = data.linear.x
    prev_y = data.linear.y
    prev_z = data.linear.z
    prev_r = data.angular.z

if __name__ == '__main__':
    global pub
    global sensitivity
    global start_time
    rospy.init_node('joystick_control_node')
    sensitivity = rospy.get_param('~sensitivity', 0.5)

    pub = rospy.Publisher('/end_effector_coord', Twist, queue_size=1)
    rospy.Subscriber('/end_effector_coord', Twist, coord_callback)

    start_time = rospy.get_time()
    rospy.Subscriber('/joy', Joy, joystick_callback) #rosrun joy joy_node _dev:=/dev/input/js0 _autorepeat_rate:=15

    # block until the node is shutdown
    rospy.spin()

