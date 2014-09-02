#! /usr/bin/python

#############################################################################
#
# This is a ros wrapper for gpsd. It publishes data from the GPS at 1Hz.
#
#############################################################################


import rospy
import time
import gps
from rocket.msg import RosGPS
from subprocess import call


class Data(): pass
D = Data()


def init():
	"""
	For initializing the node and the globals
	"""
	# init the node
	rospy.init_node("gps_node")
	
	# init the publishers
	D.gpsPub = rospy.Publisher("gps_data",RosGPS)

	# init gps connection
	init_gpsd()
	
	# gps data
	D.NSatellites = 0	# the number of satellites in view

	# time conversion info
	D.tzOffset = -8 	# offset in hours due to timezones
	D.dst = 1		# daylight savings. 1 = yes, 0 = no

	
def init_gpsd():
	"""
	sets up gpsd.

	"""
	global D
	# stop all instances of gpsd
	call(["sudo","killall","gpsd"])
	# start gpsd connection
	call(["sudo","gpsd","/dev/ttyAMA0","-F","/var/run/gpsd.sock"]) 
	# start gpsd session on port 2947 of local host (Adafruit)
	D.session = gps.gps("localhost","2947")
	D.session.stream(gps.WATCH_ENABLE|gps.WATCH_NEWSTYLE)
	

def run():
	"""
	Checks if there is a next piece of data. If so, get the elements
	we want and publish
	"""
	global D
	if True:
		report = D.session.next()
		# print report
		gpsMsg = RosGPS()
		gpsMsg.ros_time = rospy.get_time()
		if report["class"] == "TPV":
			if hasattr(report,"time"): 
				if report.time != None: gpsMsg.gps_time = getUnix(report.time)
			if hasattr(report,"lon"): 
				if report.lon != None: gpsMsg.longitude = report.lon
			if hasattr(report,"lat"): 
				if report.lat != None: gpsMsg.latitude = report.lat
			if hasattr(report,"climb"): 
				if report.climb != None: gpsMsg.climb = report.climb
			if hasattr(report,"speed"):
				if report.speed != None: gpsMsg.speed = report.speed
			if hasattr(report,"epx"):
				if report.epx != None: gpsMsg.epx = report.epx
			if hasattr(report,"epy"): 
				if report.epy != None: gpsMsg.epy = report.epy
			if hasattr(report,"ept"): 
				if report.ept != None: gpsMsg.ept = report.ept
			if hasattr(report,"eps"): 
				if report.eps != None: gpsMsg.eps = report.eps
			if hasattr(report,"epv"): 
				if report.epv != None: gpsMsg.epv = report.epv	
		# get number of satellites in view. Save to global instead of message
		# since it doesn't always get published
		elif report["class"] == "SKY":
			D.NSatellites = len(report.satellites)
		# also save satellite data in msg 	
		gpsMsg.NSatellites = D.NSatellites
		# publish the message
 		D.gpsPub.publish(gpsMsg)		

	#except:
		#rospy.logwarn("Failed to read next message")


def getUnix(gpsTime):
	"""
	Converts a time string into unix
	"""
	yr = int(gpsTime[0:4])
	mon = int(gpsTime[5:7])
	mday = int(gpsTime[8:10])
	hour = int(gpsTime[11:13])+D.tzOffset+D.dst	# account for offset due to timezones
	min = int(gpsTime[14:16])
	sec = int(gpsTime[17:19])
	wday = -1		# weekday: -1 for don't know.
	yday = -1		# day of the year: -1 for don't know
	isdst = 0 		# daylight savings. doesn't seem to have effect. Add in hour
	unix = time.mktime((yr,mon,mday,hour,min,sec,wday,yday,isdst))
	# print time.localtime(unix)
	return unix	


if __name__ == "__main__":
	init()
	rospy.loginfo("GPS node initialized")
	while not rospy.is_shutdown():
		run()
