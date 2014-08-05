#! /usr/bin/python

import rospy
from rocket.msg import *

class Data(): pass
D = Data()

init():
	rospy.init_node("deployment")
	
	# publishers
	D.pub = rospy.Publisher("eject",Bool)
	
	# subscribers
	rospy.Subscriber("gps_data",RosGPS,gps_callback)
	# rospy.Subscriber("usr_input",


def gps_callback(data):
	"""
	Stores latest GPS data
	"""


def run():
	""'
	Checks whether or not it is time to deploy the parachute.
	If so, publish flag and set GPIO pin to high.
	"""
	



if __name__ == "__main__":
	init()
	rospy.loginfo("Initialized deployment node")
	run()
