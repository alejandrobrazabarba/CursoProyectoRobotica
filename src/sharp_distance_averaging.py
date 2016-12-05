#!/usr/bin/python

import maestro
import rospy
from std_msgs.msg import Int32

messages_received = 0
measure_cumulative = 0
measure_average = 0

def callback(msg):
	global messages_received
	global measure_cumulative
	global measure_average
	messages_received += 1
	measure_cumulative += msg.data
	measure_average = measure_cumulative / messages_received
	print "Measured average: ",measure_average

rospy.init_node('sharp_distance_averaging', anonymous=False)
rospy.Subscriber('sharp_distance', Int32, callback)
rospy.spin()
