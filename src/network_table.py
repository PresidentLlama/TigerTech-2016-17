#!/usr/bin/env python3
#
# This is a NetworkTables client (eg, the DriverStation/coprocessor side).
# You need to tell it the IP address of the NetworkTables server (the
# robot or simulator).
#
# When running, this will continue incrementing the value 'dsTime', and the
# value should be visible to other networktables clients and the robot.
#

import sys
import time
from networktables import NetworkTables

# To see messages from networktables, you must setup logging
import logging
class Network():
	def __init__(self):
		logging.basicConfig(level=logging.DEBUG)
		#replace with pi's ip while connected to roboradio
		#smartdashboard
		self.sd = NetworkTables.getTable("SmartDashboard")
		self.sd.putBoolean("   Is Gear Mech Out?", False)
		i = 0
		self.sd.putNumber('RobotTime', i)
		try:
			print('piTime', self.sd.getNumber('piTime'))
		except KeyError:
			print('piTime','N/A')
		i = i+1
		time.sleep(1)
	def get_d(self):
		try:
			return self.sd.getNumber('Pdistance')
		except:
			return 1
	def get_see(self):
		return self.sd.getBoolean('PseePeg',False)
	def get_Rect(self):
		return self.sd.getString('Rectcoords','None')
	def get_width(self):
		return self.sd.getNumber('tape_width',1)
	def get_x1(self):
		x =640
		y = [0,1]
		try :
			self.RectArray = self.sd.getValue('RectArray')
		except KeyError:
			self.RectArray = y
		for c in self.RectArray:
			if c < x:
				x =c 
		return x 
	def get_x2(self):
		y = [0,1]
		try : 
			self.RectArray = self.sd.getValue('RectArray')
		except KeyError:
			self.RectArray = y
		x = 0
		
		for c in self.RectArray:
			if c >x:
				x =c 
		return x
			