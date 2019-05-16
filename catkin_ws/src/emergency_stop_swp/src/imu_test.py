#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from autominy_msgs.msg import NormalizedSpeedCommand


def drive_at_speed(speed):
	pub_speed.publish(value=speed)
	rospy.sleep(acceleration_time)
	pub_speed.publish(value=0)


rospy.init_node("brake_test", anonymous=True)

# create subscribers and publishers
pub_speed = rospy.Publisher("/control/command/normalized_wanted_speed", NormalizedSpeedCommand, queue_size=2)

acceleration_time = 4  # in seconds
speed = 0.3
while True:
	drive_at_speed(speed)
	dist = float(input("brake distance in m: "))  # measured distance in meters
	rospy.loginfo("brake distance at speed of %f: %f" % (speed, dist))
	rospy.loginfo(30*"-")
