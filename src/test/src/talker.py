#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
	# setup publisher
	pub = rospy.Publisher("chatter",String)

	# publish a message
	msg = "Hello World %.4f" % rospy.get_time()
	pub.publish(msg)

if __name__ == "__main__":
	rospy.init_node("talker")
	rate = rospy.Rate(1)
	while rospy.is_shutdown() == False:
		talker()
		rate.sleep()
		
