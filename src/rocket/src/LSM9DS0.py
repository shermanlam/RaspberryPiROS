#!/usr/bin/python

##################################################
#
# This node is for obtaining accelerometer, gyroscope,
# and magnetometer data from the LSM9DS0 IC.
#
##################################################

import rospy
from Adafruit_GPIO import I2C


class Data: pass
D = Data()


def init():
	init_ros()
	init_i2c()


def init_ros():
	"""
	Initializes ROS stuff
	"""
	rospy.init_node("LSM9DS0")

	# rate
	D.rate = 1	# [Hz]

def init_i2c():
	"""
	Initializes i2c connection with the IC
	"""
	# addresses
	addrG = 0x6b		# gyro
	addrXM = 0x1d		# accel
	
	# i2c bus
	bus = 1

	# initialize the registers
	init_registers_g()
	init_registers_xm()	

	# initialize options for the accelerometer and gyro
	init_ctrl_options_G()
	init_ctrl_options_XM()

	# use the Adafruit I2C library to setup a connection
	D.gyro = I2C.Device(addrG,bus)
	D.accel = I2C.Device(addrXM,bus)

	# check that the devices are connected properly
	check_id()

	# send the configurations
	write_ctrl_options()	


def init_registers_g():
	"""
	Init the gyro registers
	"""
	global D
	
	# IC identity
	D.WHO_AM_I_G		= 0x0f		
	
	# control registers
	D.CTRL_REG1_G		= 0x20
	D.CTRL_REG2_G		= 0x21
	D.CTRL_REG3_G		= 0x22
	D.CTRL_REG4_G		= 0x23
	D.CTRL_REG5_G		= 0x24
	
	# status register
	D.STATUS_REG_G		= 0x27
	
	# data output registers
	D.OUT_X_L_G		= 0x28
	D.OUT_X_H_G		= 0x29	
	D.OUT_Y_L_G		= 0x2a	
	D.OUT_Y_H_G		= 0x2b
	D.OUT_Z_L_G		= 0x2c
	D.OUT_Z_H_G		= 0x2d

	# FIFO
	D.FIFO_CTRL_REG_G	= 0x2e
	
	
def init_registers_xm():
	"""
	Init the accel and magnetometer registers
	"""
	global D
	
	# IC identity
	D.WHO_AM_I_XM		= 0x0f		
	
	# control registers
	D.CTRL_REG1_XM		= 0x20
	D.CTRL_REG2_XM		= 0x21
	D.CTRL_REG3_XM		= 0x22
	D.CTRL_REG4_XM		= 0x23
	D.CTRL_REG5_XM		= 0x24	
	D.CTRL_REG6_XM		= 0x25
	D.CTRL_REG7_XM		= 0x26

	# status register
	D.STATUS_REG_A		= 0x27
	
	# data output registers
	D.OUT_X_L_A		= 0x28
	D.OUT_X_H_A		= 0x29	
	D.OUT_Y_L_A		= 0x2a	
	D.OUT_Y_H_A		= 0x2b
	D.OUT_Z_L_A		= 0x2c
	D.OUT_Z_H_A		= 0x2d

	# FIFO
	D.FIFO_CTRL_REG_XM	= 0x2e	# in datasheet, listed without XM tag


def init_ctrl_options_G():
	"""
	Sets the control options for the gyroscope. I am only including
	the non-default options and some of the default options.
	Check the datasheet for options
	"""
	global D
	
	# gyro identity
	D.identityG	= 0b11010100

	# control register 1
	DR		= 0b11		# 760 [Hz]
	BW		= 0b11		# 100 [Hz]
	PD		= 1		# normal
	Zen		= 1		# enabled	
	Yen		= 1		# enabled
	Xen 		= 1		# enabled
	D.ctrl1_G	= (DR<<6)|(BW<<4)|(PD<<3)|(Zen<<2)|(Yen<<1)|Xen


def init_ctrl_options_XM():
	"""
	Sets the control options for the accelerometer. I am only including
	the non-default options and some of the default options.
	"""
	global D
	# accel identity
	D.identityXM	= 0b01001001

	# control register 1
	AODR		= 0b1010	# 1600 [Hz]
	BDU		= 0b0		# continuous update
	AZen		= 0b1		# enabled
	AYen		= 0b1		# enabled
	AXen		= 0b1		# enabled
	D.ctrl1_XM	= (AODR<<4)|(BDU<<3)|(AZen<<2)|(AYen<<1)|AXen

	# control register 2
	ABW		= 0b00		# 773 [Hz]
	AFS		= 0b100		# +/- 16 [g]
	AST		= 0b00		# normal
	D.ctrl2_XM	= (ABW<<6)|(AFS<<3)|(AST<<1)


def check_id():
	"""
	Checks that the IDs of the accel and gyro match the expected IDs
	"""
	global D
	
	error = False

	# gyro
	idG = D.gyro.readU8(D.WHO_AM_I_G)
	if idG != D.identityG:
		error = True
		rospy.logerror("gyro connected to wrong I2C device")
	# accel
	idXM = D.accel.readU8(D.WHO_AM_I_XM) 
	if idXM != D.identityXM:
		error = True
		rospy.logerror("accel connected to wrong I2C device")
	if error:
		rospy.signal_shutdown("wrong I2C connections")


def write_ctrl_options():
	"""
	Sends the control options for the gyro and accel
	"""
	global D
	# gyro controls
	D.gyro.write8(D.CTRL_REG1_G,D.ctrl1_G)

	# accel controls
	D.accel.write8(D.CTRL_REG1_XM,D.ctrl1_XM)
	D.accel.write8(D.CTRL_REG2_XM,D.ctrl2_XM)


def read_gyro():
	"""
	reads the data from the gyroscope
	"""
	global D
	# merge the low and high bits
	x = (D.gyro.readU8(D.OUT_X_H_G)<<8)|D.gyro.readU8(D.OUT_X_L_G)
	y = (D.gyro.readU8(D.OUT_Y_H_G)<<8)|D.gyro.readU8(D.OUT_Y_L_G)
	x = (D.gyro.readU8(D.OUT_Z_H_G)<<8)|D.gyro.readU8(D.OUT_Z_L_G)
	data = [x,y,z]
	# convert to two's complement
	converted = map(twos_complement,data)
	return converted 


def read_accel():
	"""
	reads the data from the accel
	"""
	global D
	x = (D.accel.readU8(D.OUT_X_H_A)<<8)|D.accel.readU8(D.OUT_X_L_A)
	y = (D.accel.readU8(D.OUT_Y_H_A)<<8)|D.accel.readU8(D.OUT_Y_L_A)
	x = (D.accel.readU8(D.OUT_Z_H_A)<<8)|D.accel.readU8(D.OUT_Z_L_A)
	data = [x,y,z]
	# convert to two's complement
	converted = map(twos_complement,data)
	return converted 

	
def twos_complement(u16):
	"""
	converts an unsigned 16 bit value into two's compliment
	"""
	bits = 16
	if (u16 & (1<<(bits-1))) != 0:	# get MSB. check if u16 is negative
		u16 = u16 - (1<<bits)
	return u16


def read_data():
	"""
	waits for the gyro and accel data to be ready before trying to read.
	Alternates between checking each one
	"""
	# whether or not the data has been read
	readG = 0
	readA = 0
	# data
	gyroData = [0,0,0]
	accelData = [0,0,0]
	# loop while both have not been read
	while not(readG and readA):
		# check if gyro is ready
		if not readG:
			print "checking G"
			readyG = D.STATUS_REG_G & (1<<3) # 4th LSB
			if readyG:
				# read
				dataG = read_gyro()
				readG = 1
		if not readA:
			print "checking A"
			readyA = D.STATUS_REG_A & (1<<3) # 4th LSB
			if readyA:
				# read
				dataA = read_accel()
				readA = 1
	return dataG,dataA


def run():
	"""
	samples the gyro and accel at the desired control loop rate
	"""
	global D
	rate = rospy.Rate(D.rate)	
	while not rospy.is_shutdown():
		# dataG,dataA = read_data()
		# TODO: publish
		print "gyro ctrl reg", bin(D.gyro.readU8(D.CTRL_REG1_G))
		print "accel ctrl reg 1", bin(D.accel.readU8(D.CTRL_REG1_XM))
		print "accel ctrl reg 2", bin(D.accel.readU8(D.CTRL_REG2_XM))
		rate.sleep()
	

if __name__ == "__main__":
	init()
	rospy.loginfo("LSM9DS0 node initialized")
	try:	
		run()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down LSM9DS0")
		time.sleep(1)	
		rospy.signal_shutdown("Keyboard interrupt")
