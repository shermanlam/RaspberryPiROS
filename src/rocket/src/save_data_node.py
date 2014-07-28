#! /usr/bin/python

#######################################################
#
# This is a node for saving data to a file. To add an
# additional piece of data to save, add an additional
# subscriber, a callback for that topic, and a function
# to write the data to file
#
#######################################################


import rospy
import numpy as np
import os.path
from rocket.msg import *


class Data(): pass
D = Data()


def init():
	"""
	Inits subscribers and global variables
	"""
	
	# init ros node
	rospy.init_node("logger")

	# init gps stuff
	rospy.Subscriber("gps_data",RosGPS,gps_callback)
	D.gpsData = None
	D.gpsFileName = "gps" 		# string describing file names.
					# typical file: gps1.txt

	rospy.on_shutdown(write)


def gps_callback(data):
	"""
	This gets called when the logger recieves new message.
	Saves data in global
	"""
	global D

	# turn the data into a massive (1x13) array
	array = np.asarray(	[[data.gps_time,
				data.ros_time,
				data.latitude,
				data.longitude,
				data.altitude,
				data.speed,
				data.climb,
				data.eps,
				data.epx,
				data.epy,
				data.epv,
				data.ept	]])
	
	# first piece of data to be received
	if D.gpsData == None:
		D.gpsData = array
	
	# otherwise, append to the existing list
	else:
		D.gpsData = np.append(D.gpsData,array,0)
	
	
def gps_write():
	"""
	Writes the gps data to file
	"""
	global D
	
	# header for describing the data in the file
	header = "Order of data: gpstime,rostime,latitude,longitude,"
	header += "altitude,speed,climb,eps,epx,epy,epv,ept. "
	header += "All data can be read using numpy.loadtxt"

	# get the next available number
	N = get_next_fileNum(D.gpsFileName)

	# try writing the file. If successful, return True
	try:
		filename = "/home/pi/catkin_ws/src/rocket/flights/%s%d.txt"%(D.gpsFileName,N)
		np.savetxt(filename,D.gpsData)
		rospy.loginfo("Successfully wrote gps data to %s"%filename)
		return True
	except:
		rospy.logwarn("Failed to write gps data to %s"%filename)
		return False


def get_next_fileNum(fileName):
	"""
	This returns an integer that is the latest copy of a 
	file type. For example, if fileName is "D.gpsFileName",
	this means we want to look at the set of gps files. All
	gps files will be in the format <D.gpsFileName>N.txt 
	where the < and the > are omitted, D.gpsFileName is a 
	string, and N in an integer. Typically, N represents a 
	flight number. If there is already a file named
	<D.gpsFileName>1.py, this should return 2 since it is the
	next unused "flight number" in line.
	"""
	global D

	N = 1
	while os.path.isfile("/home/pi/catkin_ws/src/rocket/flights/%s%d.txt"%(fileName,N)):
		N += 1
	return N
	

def write():
	"""
	Calls all the write functions on shudown
	"""
	rospy.loginfo("Starting to write data to file")
	success = False 
	success = success or gps_write()	

	if success:
		rospy.loginfo("Successfully wrote all data to file")
	else:
		msg = "Failed to write some data to file. "
		msg += "Check .ros for error details"
		rospy.logwarn(msg)
	


if __name__ == "__main__":
	init()
	rospy.loginfo("Data logging node initialized")
	rospy.spin() 
