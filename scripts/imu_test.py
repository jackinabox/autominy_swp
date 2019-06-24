#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from autominy_msgs.msg import NormalizedSpeedCommand


def drive_at_speed(speed, acceleration_time):
	pub_speed.publish(value=speed)
	rospy.sleep(acceleration_time)
	pub_speed.publish(value=0)


rospy.init_node("brake_test", anonymous=True)

# create subscribers and publishers
pub_speed = rospy.Publisher("/control/command/normalized_wanted_speed", NormalizedSpeedCommand, queue_size=2)

iter_per_speed = 3
speed_list = [.3, .4, .5, .6, .7]
acc_list = [4, 3, 2, 1.5, 1]

assert len(speed_list) == len(acc_list), "#speeds != #accelerations"
for i in range(0, len(speed_list)):
	for j in range(iter_per_speed):
		drive_at_speed(speed_list[i], acc_list[i])
		dist = float(input("brake distance in m: "))  # measured distance in meters
		rospy.loginfo("brake distance at speed of %f: %f" % (speed_list[i], dist))
		rospy.loginfo(30*"-")
