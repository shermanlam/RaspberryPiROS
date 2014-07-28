#! usr/bin/python

##################################
#
# This is a ros wrapper for gpsd
#
##################################


import rospy
import time
from rocket.msg import RosGPS
from subprocess import call


def init():
	"""
	For initializing the node and the globals
	"""
	# init the node
	rospy.init("gps_node")
	
	# init the publishers

	# gps data



def initGPSD():
	"""
	sets up gpsd.

	"""


def publish():
	pass


def run():
	pass




if __name__ == "__main__":
	init()
	rospy.loginfo("GPS node initialized")
	while not rospy.is_shutdown():
		run()
