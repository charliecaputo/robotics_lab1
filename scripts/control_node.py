#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from robotics_lab1.msg import Turtlecontrol

class ControlNode:
    def __init__(self):
        rospy.init_node('control_node', anonymous=True)

        self.current_position = 0.0
        self.desired_position = 0.0
        self.control_gain = 0.0

        # Subscribers
        rospy.Subscriber('/turtle1/pose', Twist, self.pose_callback)
        rospy.Subscriber('/turtle1/control_params', Turtlecontrol, self.control_params_callback)

        # Publisher
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Control Loop
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.proportional_control()
            rate.sleep()

    def pose_callback(self, msg):
        self.current_position = msg.linear.x

    def control_params_callback(self, msg):
        self.desired_position = msg.xd
        self.control_gain = msg.kp

    def proportional_control(self):
        error = self.desired_position - self.current_position
        control_velocity = self.control_gain * error

        # Publish velocity command
        twist_msg = Twist()
        twist_msg.linear.x = control_velocity
        self.velocity_publisher.publish(twist_msg)
