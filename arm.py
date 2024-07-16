#!/usr/bin/env python2
import time
import numpy as np
import rospy
from std_msgs.msg import Float64


def main():

	pub = rospy.Publisher('/arm_shoulder_pan_joint/command', Float64, queue_size=10)
	pub2 = rospy.Publisher('/arm_shoulder_lift_joint/command', Float64, queue_size=10)
	pub3 = rospy.Publisher('/arm_elbow_flex_joint/command', Float64, queue_size=10)
	pub4 = rospy.Publisher('/arm_wrist_flex_joint/command', Float64, queue_size=10)
	pub5 = rospy.Publisher('/gripper_joint/command', Float64, queue_size=10)


	time.sleep(0.1)

	a = 0
	#while not rospy.is_shutdown():
	pub.publish(0)
	pub2.publish(1.57)
	pub3.publish(0)
	pub4.publish(0)
	pub5.publish(0.5) #0.5 to 2


	time.sleep(0.1)
	exit()


def listener():
	rospy.init_node('AAAAAAAAAAAAAAAAAAAAAA')
	main()
	rospy.spin()

listener()














