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


iter_per_speed = 3
acceleration_time = 2  # in seconds
speed_list = [.1, .2, .3, .4, .5, .6, .7, .8, .9, 1.]
res = {}
for speed in speed_list:
	for i in range(iter_per_speed):
		rospy.loginfo("braking from speed of %f - run %d" % (speed, i))
		drive_at_speed(speed)
		dist = float(input("brake distance in m: "))  # measured distance in meters
		if speed in res:
			res[speed] = res[speed] + [dist]
		else:
			res[speed] = [dist]
		rospy.loginfo("brake distance at speed of %f: %f" % (speed, dist))
		#input("put car into position and press any key")
		rospy.loginfo(30*"-")

print("\nresults for the respective speeds:")
for key in res:
	print(key, ": ", res[key])

print("\nmean results for the respective speeds:")
for key in res:
	print(key, ": ", np.mean(res[key]))

input("brake distance test done\npress any key to exit...")
