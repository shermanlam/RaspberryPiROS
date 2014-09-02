#! /usr/bin/python

############################################################################
#
# When this node recieves the fire command, it pulls the signal pin of the 
# deployment board high. This separates the rocket.
#
############################################################################


import rospy
import time
import wiringpi2 as wp2
from std_msgs.msg import Bool

class Data(): pass
D = Data()

def init():
	rospy.init_node("deployment")
	
	# publishers
	D.firePub = rospy.Publisher("fire_success",Bool)
	
	# subscribers
	rospy.Subscriber("fire",Bool,fire)
	
	# firing information
	D.pin = 23	# BCM numbering system
	D.sec = 2	# number of seconds to hold the pin high 
	
	init_gpio()	


def init_gpio():
	"""
	Initializes gpio pin
	"""
	wp2.wiringPiSetupGpio() 	# should be setup with BCM numbering system
	wp2.pinMode(D.pin,1) 		# set to output


def fire(data):
	"""
	Turns the GPIO pin on for the specified number of seconds.
	"""
	global D
	if data.data==True:
		rospy.loginfo("Firing ejection charge")
		rospy.loginfo("Setting GPIO pin %s to HIGH"%D.pin)
		wp2.digitalWrite(D.pin,1) 	# set the pin high
		time.sleep(D.sec)		# wait
		rospy.loginfo("Setting GPIO pin %s to LOW"%D.pin)
		wp2.digitalWrite(D.pin,0)	# set the pin low
		msg = Bool(True)
		D.firePub.publish(msg)


if __name__ == "__main__":
	init()
	rospy.loginfo("Initialized deployment node")
	rospy.spin()
