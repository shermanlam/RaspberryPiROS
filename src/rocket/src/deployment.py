#! /usr/bin/python

import rospy
import time
import wiringpi2 as wp2
from std_msgs.msg import Bool
from rocket.msg import *

class Data(): pass
D = Data()

def init():
	rospy.init_node("deployment")
	
	# publishers
	D.pub = rospy.Publisher("eject",Bool)
	
	# subscribers
	rospy.Subscriber("gps_data",RosGPS,gps_callback)
	
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


def gps_callback(data):
	"""
	Stores latest GPS data
	"""
	pass


def fire():
	"""
	Turns the GPIO pin on for the specified number of seconds.
	"""
	global D
	rospy.loginfo("Setting GPIO pin %s to HIGH"%D.pin)
	wp2.digitalWrite(D.pin,1) 	# set the pin high
	time.sleep(D.sec)		# wait
	rospy.loginfo("Setting GPIO pin %s to LOW"%D.pin)
	wp2.digitalWrite(D.pin,0)	# set the pin low


def test():
	"""
	Cycles the output pin on and off
	"""
	global D
	fire()
	time.sleep(D.sec)		


def run():
	"""
	Checks whether or not it is time to deploy the parachute.
	If so, publish flag and set GPIO pin to high.
	"""
	while not rospy.is_shutdown():
		test()


if __name__ == "__main__":
	init()
	rospy.loginfo("Initialized deployment node")
	run()
