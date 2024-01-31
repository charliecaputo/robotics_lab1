#!/usr/bin/env python

import rospy
from Turtlecontrol.msg
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleControlNode:
    def __init__(self):
        rospy.init_node('turtle_control_node', anonymous=True)

        # Subscriber to receive position information from turtlesim_node
        rospy.Subscriber('/turtle1/pose', Pose, self.turtle_pose_callback)

        # Subscriber to receive desired position and control gain
        rospy.Subscriber('/turtle1/control_params', Turtlecontrol, self.control_params_callback)

        # Publisher to send control commands to turtlesim_node
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize variables to store position information and control parameters
        self.current_pose = Pose()
        self.desired_position = 0.0
        self.control_gain = 0.0

    def turtle_pose_callback(self, msg):
        # Update current position information
        self.current_pose = msg

    def control_params_callback(self, msg):
        # Update desired position and control gain
        self.desired_position = msg.xd
        self.control_gain = msg.kp

    def proportional_control(self):
        # Proportional controller logic
        error = self.desired_position - self.current_pose.x

        # Calculate control command using proportional control
        control_command = self.control_gain * error

        # Create Twist message to publish control command
        twist_msg = Twist()
        twist_msg.linear.x = control_command

        # Publish control command
        self.cmd_vel_pub.publish(twist_msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Perform proportional control based on updated parameters
            self.proportional_control()

            rate.sleep()
