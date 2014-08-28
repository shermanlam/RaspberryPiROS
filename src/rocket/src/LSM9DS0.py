#!/usr/bin/python

##################################################
#
# This node is for obtaining accelerometer, gyroscope,
# and magnetometer data from the LSM9DS0 IC.
#
##################################################

# import rospy
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

def init_i2c():
	"""
	Initializes i2c connection with the IC
	"""
	# addresses
	addrG = 0x6a		# gyro
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
	D.gyro = I2C.Device(D.addrG,bus)
	D.accel = I2C.Device(D.addrXM,bus)


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
	AZen		= 0b1		# enabled
	D.ctrl1_XM	= (AODR<<4)|(BDU<<3)|(AZen<<2)|(AYen<<1)|AXen

	# control register 2
	ABW		= 0b00		# 773 [Hz]
	AFS		= 0b100		# +/- 16 [g]
	AST		= 0b00		# normal
	D.ctrl2_XM	= (ABW<<6)|(AFS<<4)|(AST<<1)
