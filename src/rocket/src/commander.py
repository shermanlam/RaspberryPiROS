#! /usr/bin/python

########################################################################
#
# This node is for position estimation.When it detects apogee, it sends
# a "fire" command to the deployment node.
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
		if (cmd == "shutdown") or (cmd == "quit") or (cmd == "stop"):
			rospy.loginfo("Shutting down")
			rospy.signal_shutdown("Shutdown requested")
		else:
			rospy.loginfo("Command %s not recognized"%cmd)
	except:
		rospy.loginfo("Error shutting down")

if __name__ == "__main__":
	init()
	rospy.loginfo("Commander node initialized")
	rospy.spin()
