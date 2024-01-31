#!/usr/bin/env python3

import rospy
# we are going to read turtlesim/Pose messages thigit s time
from Turtlecontrol.msg import Pose
# for radians to degrees conversions
import math
#import new message
from ros_intro_lectures.msg import Shortpose


ROTATION_SCALE = 180.0/math.pi

#defining a global variable
pos_msg = Turtlecontrol()

def pose_callback(data):
	global pos_msg
	# convert angular position to degrees
	##pos_msg.theta = data.theta * ROTATION_SCALE
	# convert x and y to cm
	pos_msg.xd = data.x * 100
	pos_msg.kp = data.y * 100
	# show the results
	#rospy.loginfo("x is %0.2f cm, y is %0.2f cm, theta is %0.2f degrees", x_in_cm, y_in_cm, rot_in_degree)
	

if __name__ == '__main__':
	# initialize the node
	rospy.init_node('pos_publisher', anonymous = True)
	# add a subscriber to it to read the position information
	rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
	pos_pub = rospy.Publisher('/turtle1/short_pose', Shortpose, queue_size = 10)
	
	#frequency loop rate, set to 10 hertz
	loop_rate = rospy.Rate(10)
	# spin() simply keeps python from exiting until this node is stopped
	# run this control loop regularly
	while not rospy.is_shutdown():

		pos_pub.publish(pos_msg)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()

	rospy.spin()
	
