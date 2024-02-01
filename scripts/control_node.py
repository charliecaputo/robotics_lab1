#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from robotics_lab1.msg import Turtlecontrol

def pose_callback(data):
    global current_position
    current_position = data.x

def control_params_callback(msg):
    global desired_position, control_gain
    desired_position = msg.xd
    control_gain = msg.kp

def proportional_control():
    global current_position, desired_position, control_gain
    error = desired_position - current_position


    # Ensure the control velocity is within a reasonable range
    control_velo = control_gain*(desired_position - current_position)

    # Publish velocity command
    twist_msg = Twist()
    twist_msg.linear.x = control_velo
    velocity_publisher.publish(twist_msg)
    
    print (current_position, " ---- ", desired_position, " ---- ", control_gain)

if __name__ == '__main__':
    # initialize the node
    rospy.init_node('control_node', anonymous=True)

    global current_position, desired_position, control_gain
    current_position = 0.0
    desired_position = 0.0
    control_gain = 0.0

    # Subscribers
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rospy.Subscriber('/turtle1/control_params', Turtlecontrol, control_params_callback)

    # Publisher
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    

    # Control Loop
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        proportional_control()
        rate.sleep()
