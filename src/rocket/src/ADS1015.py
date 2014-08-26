#!/usr/bin/python

###########################################################
#
# This is a ROS wrapper for the ADS1015 analog-to-digital
# converter.
#
# Note: all variables that start with a "_" (_128SPS)
# are options. These are meant to abstract away to binary
# codes. If I want to set the sampling rate to 128 samples
# per second, I would use "D.sps = D._128SPS"
#
###########################################################

# import rospy
import time
from Adafruit_GPIO import I2C
import RPi.GPIO as GPIO

class Data: pass
D = Data()


def init():
	global D

	# init ROS stuff	
	# init_ros()
	
	# init I2C stuff
	init_i2c()


def init_ros():
	"""
	inits globals needed for ROS operation
	"""
	global D

	# node
	# rospy.init_node("ADS1015")
	
	# subscribers

	# publishers
	

def init_i2c():
	"""
	Options for i2c communications with the ADS1015
	
	"""
	global D

	# address of the ADC
	D.addr 		= 0x48

	# bus number of the ADC. On RPi Model B, default is 1
	D.bus 		= 1

	# if ordering system is little endian or big endian
	D.littleEndian 	= False

	init_ADC_options()
	init_default_options()
 
	# use the adafruit I2C library for I2C access
	D.dut = I2C.Device(D.addr,D.bus)

	# activate ready pin on ADC
	activate_ready()

	# init a pin for listening to the "conversion-ready" pin
	D.conversionReady = 17
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(D.conversionReady, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.add_event_detect(D.conversionReady, GPIO.RISING, callback=ready_callback)


def activate_ready():
	"""
	swaps the high and low threshold registers so that the conversion
	ready pin will be activated on the ADC
	"""
	high = I2C.reverseByteOrder(0x8000)
	low = I2C.reverseByteOrder(0x7fff)
	D.dut.write16(D.highThreshReg, high)
	D.dut.write16(D.lowThreshReg,low)


def ready_callback(channel):
	"""
	Gets called when a sample from the ADC is ready
	"""
	print "Conversion Ready"


def init_ADC_options():
	"""
	These are configuration options we can set the configuration
	parameters to
	"""
	global D

	# registers that we can access
	D.conversionReg 	= 0b00
	D.configReg		= 0b01
	D.lowThreshReg 		= 0b10
	D.highThreshReg		= 0b11

	# data sampling rates
	D._128SPS	= 0b000
	D._250SPS 	= 0b001
	D._490SPS 	= 0b010
	D._920SPS 	= 0b011
	D._1600SPS 	= 0b100	 	# default
	D._2400SPS 	= 0b101
	D._3300SPS	= 0b110

	# mux configurations for sampling 1 channel at a time
	D._MUXA0 	= 0b100 	# default
	D._MUXA1 	= 0b101
	D._MUXA2 	= 0b110
	D._MUXA3 	= 0b111

	# progammable gain configurations
	D._PGA2_3	= 0b000 
	D._PGA1	 	= 0b001
	D._PGA2 	= 0b010 	# default
	D._PGA4		= 0b011
	D._PGA8 	= 0b100
 	D._PGA16	= 0b101


def init_default_options():
	"""
	Sets the default configuration options
	"""
	global D
	D.dr = D._1600SPS 	# data rate
	D.mux = D._MUXA0 	# mux configuration
	D.pga = D._PGA1		# programmable gain
	

def write_config():
	"""
	Write the desired options to the ADC configuration register.
	Get data from global options
	"""
	global D
	
	# bit shift everything into its right place in the message. Check datasheet.
	# Some options were not broken out for changing. The "magic bits" are 
	# those options. Combine all the options through bit-wise OR
	newConfig = (	0 << 15		|	# OS
			D.mux << 12	|	# MUX
			D.pga << 9	|	# PGA
			0 << 8		|	# MODE
			D.dr << 5	|	# DR
			0 << 4		|	# COMP_MODE
			0 << 3 		|	# COMP_POL
			0 << 2		|	# COMP_LAT
			0 << 1 	)		# COMP_QUE
	
	print "mux: ", bin(D.mux)
	print "pga: ", bin(D.pga)
	print "dr: ", bin(D.dr)

	# due to something with byte-writing order, we need to flip
	# the bytes before writing. I don't know the full details
	print "config: %s"%bin(newConfig)
	print "length: %d"%len(bin(newConfig))
	rev = I2C.reverseByteOrder(newConfig)
	print "Reversed byte order: ", bin(rev)

	# write it!
	D.dut.write16(D.configReg,rev)	
	print "Written config: ", bin(D.dut.readU16(D.configReg,D.littleEndian))


def read_single_in(channel):
	"""
	Reads a single analog input into the ADC
	Input: 0-3
	"""
	global D
	
	print "Trying to read %d"%channel

	# change the configuration to account for the correct channel
	if channel == 0:
		D.mux = D._MUXA0
	elif channel == 1:
		D.mux = D._MUXA1
	elif channel == 2:
		D.mux = D._MUXA2
	elif channel == 3:
		D.mux = D._MUXA3
	else:
		# TODO signal some error
		pass
	
	# write the new configuration
	write_config()

	# wait for the configuration to be loaded
	# time.sleep(0.1)	
	
	# read new conversion
	print "Conversion: ", D.dut.readU16(0)
	print 
	

def run():
	read_single_in(0)
	time.sleep(1)	


if __name__ == "__main__":
	init()
	cleanedUp = False
	try:
		while True:
			run()
	except KeyboardInterrupt:
		GPIO.cleanup() 		# clean up on <CTRL-C>
		cleanedUp = True
	if cleanedUp == False:
		GPIO.cleanup()		# clean up on normal exit
