#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Initial coordinates
initialCoordinates = [100, 0, 0, 0]

# Initialize previous coordinate values
prev_x, prev_y, prev_z, prev_r = initialCoordinates

# Sensitivity factor (adjust as needed)
sensitivity = 0.1

# previous values
prevTwist = Twist()

def joystick_callback(data):
    global prev_x, prev_y, prev_z, prev_r, prevTwist

    # Extract joystick axes values
    x = -data.axes[0]  # Horizontal movement (x-coordinate)
    y = data.axes[1]  # Vertical movement (y-coordinate)
    z = data.axes[3]  # Altitude (z-coordinate)
    r = -data.axes[2]  # Rotation (r-coordinate)

    # Reset if data.buttons[5] is pressed
    if data.buttons[5] == 1:
        prev_x, prev_y, prev_z, prev_r = initialCoordinates

    # Apply sensitivity and map to coordinate changes
    x_change = sensitivity * x
    y_change = sensitivity * y
    z_change = sensitivity * z
    r_change = sensitivity * r

    # Update previous values
    prev_x += x_change
    prev_y += y_change
    prev_z += z_change
    prev_r += r_change

    # only publish if there is a change in the coordinates
    if data.buttons[5] == 1 or x_change != 0 or y_change != 0 or z_change != 0 or r_change != 0:
      # Create a Twist message
      twist = Twist()
      twist.linear.x = prev_x
      twist.linear.y = prev_y
      twist.linear.z = prev_z
      twist.angular.z = prev_r
      
      # print 3 sig fig of x y z r
      # print(f"x: {prev_x:.3f}, y: {prev_y:.3f}, z: {prev_z:.3f}, r: {prev_r:.3f}")
      # print(twist)

      # Publish the Twist message
      pub.publish(twist)
      prevTwist = twist
    else:
      pub.publish(prevTwist)

if __name__ == '__main__':
    global pub
    rospy.init_node('joystick_control_node')
    pub = rospy.Publisher('/end_effector_coord', Twist, queue_size=10)
    rospy.Subscriber('/joy', Joy, joystick_callback) #rosrun joy joy_node _dev:=/dev/input/js0 _autorepeat_rate:=15

    # block until the node is shutdown
    rospy.spin()

