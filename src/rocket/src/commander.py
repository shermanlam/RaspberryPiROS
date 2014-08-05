#! /usr/bin/python

########################################################################
#
# This node is for sending command signals to all the other nodes
#
########################################################################i

import rospy
from std_msgs.msg import String

def init():
	rospy.init_node("commander")
	rospy.Subscriber("cmd",String,parse)

def parse(data):
	"""
	Interprets command
	"""
	cmd = str(data.data)
	try:
		if (cmd == "shutdown") or (cmd == "quit"):
			rospy.signal_shutdown("User requests rospy shutdown")
	except:
		rospy.loginfo("Command %s not recognized"%cmd)

if __name__ == "__main__":
	init()
	rospy.loginfo("Initialized commander node")
	rospy.spin()
