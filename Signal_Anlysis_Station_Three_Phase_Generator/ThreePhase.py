#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  ThreePhase.py
#  
#  Copyright 2017  <pi@raspberrypi>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  

import sys
import os
import subprocess
from PyQt4 import QtGui, QtCore, uic

#I2C Connection
#	Raspi		|		Target
# 	pin3				pin10
#	pin5				pin9
#	pin6				GND

import smbus
import struct


class MyWindow(QtGui.QMainWindow):
	def __init__(self):
		#Create Window
		super(MyWindow, self).__init__()
		uic.loadUi('ThreePhase.ui',self)
		self.show()
		qr = self.frameGeometry()
		cp = QtGui.QDesktopWidget().availableGeometry().center()
		qr.moveCenter(cp)
		self.move(qr.topLeft())
		
		#Create Vars to hold output values
		self.freq = 100 #hz 
		self.freqStep = 100
		self.gain = 0.8003
		self.offset = 0
		self.op_code = 1 #idle
		self.channel_1_shift_value = 0
		self.channel_2_shift_value = 0
		self.channel_3_shift_value = 0
		self.channel_1_mult_value = 0.0
		self.channel_2_mult_value = 0.0
		self.channel_3_mult_value = 0.0
		
		#Create necessary local vars
		self.update_freq = 1000 #time in msec for timer experation
		self.msg_data = []
		self.dataHasChanged = False
				
		#Connect GUI features to functionality
			#connect frequency input
		self.Increase_Freq_Btn.clicked.connect(self.increaseFreq)
		self.Decrease_Freq_Btn.clicked.connect(self.decreaseFreq)
		self.connect(self.FreqDial, QtCore.SIGNAL('valueChanged(int)'), self.updateFreq)
			#connect gain input
		self.OffsetInput.valueChanged.connect(self.updateOffset)
			#connect offset input
		self.GainInput.valueChanged.connect(self.updateGain)
			#connect start button
		self.Startbtn.clicked.connect(self.startBtnPress)
			#connect stop button
		self.Stopbtn.clicked.connect(self.stopBtnPress)
		self.Stopbtn.setEnabled(False)
			#connect channel offset 
		self.Channel1_offset.valueChanged.connect(self.channelShiftUpdate)
		self.Channel2_offset.valueChanged.connect(self.channelShiftUpdate)
		self.Channel3_offset.valueChanged.connect(self.channelShiftUpdate)
			#connect channel multipliers
		self.Channel1_mult.valueChanged.connect(self.channelMultiplierUpdate)
		self.Channel2_mult.valueChanged.connect(self.channelMultiplierUpdate)
		self.Channel3_mult.valueChanged.connect(self.channelMultiplierUpdate)
			#connect reset button
		self.ResetBtn.clicked.connect(self.resetGenData)
		
		#Create I2C comm
		self.DEVICE_BUS = 1
		self.DEVICE_ADDR = 0x08
		try:
			self.bus = smbus.SMBus(self.DEVICE_BUS)
			self.updateSysStatus("Connected")
		except:
			self.updateSysStatus("I2C Connection Error Please Check Connections and Restart")
			
		#Create update timer
		self.timer = QtCore.QTimer()
		self.timer.setInterval(self.update_freq)
		self.timer.timeout.connect(self.timerExperation)
		self.timer.start()
	
	def increaseFreq(self):
		#grab current dial value 
		self.freq = self.FreqDial.sliderPosition() + self.freqStep
		self.FreqDial.setValue(self.freq)
		#update LCD display to this value
		self.FreqLCD.display((self.freq/2)-0.5)
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def decreaseFreq(self):
		#grab current dial value 
		self.freq = self.FreqDial.sliderPosition() - self.freqStep
		self.FreqDial.setValue(self.freq)
		#update LCD display to this value
		self.FreqLCD.display((self.freq/2)-0.5)
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def resetGenData(self):
		#reset 3 phase generator settings
		self.FreqDial.setValue(100)
		self.FreqLCD.display((self.freq/2)-0.5)
		self.OffsetInput.setValue(0.0)
		self.GainInput.setValue(0.8)
		self.Startbtn.setEnabled(True)
		self.Stopbtn.setEnabled(False)
		self.Channel1_offset.setValue(0)
		self.Channel2_offset.setValue(0)
		self.Channel3_offset.setValue(0)
		self.Channel1_mult.setValue(0.0)
		self.Channel2_mult.setValue(0.0)
		self.Channel3_mult.setValue(0.0)
		self.op_code = 3
		#set global update value
		self.dataHasChanged = True
		
	def channelShiftUpdate(self):
		#update all channel shift values
		self.channel_1_shift_value = self.Channel1_offset.value()
		self.channel_2_shift_value = self.Channel2_offset.value()
		self.channel_3_shift_value = self.Channel3_offset.value()
		#set global update value
		self.dataHasChanged = True
	
	def channelMultiplierUpdate(self):
		#update all channel shift values
		self.channel_1_mult_value = self.Channel1_mult.value()
		self.channel_2_mult_value = self.Channel2_mult.value()
		self.channel_3_mult_value = self.Channel3_mult.value()
		#set global update value
		self.dataHasChanged = True
		
	def updateSysStatus(self, msg="Normal"):
		self.ErrorLbl.setText("Status: "+msg)
		
	def startBtnPress(self):
		#udpate op_code 
		self.op_code = 2 #start
		#set global update value
		self.dataHasChanged = True
		#disable startBtn and enable stopBtn
		self.Stopbtn.setEnabled(True)
		self.Startbtn.setEnabled(False)
		
	def stopBtnPress(self):
		#update op_code
		self.op_code = 3 #stop, cleanup, and then idle
		#set global update value
		self.dataHasChanged = True
		#disable stopBtn and enable startBtn
		self.Startbtn.setEnabled(True)
		self.Stopbtn.setEnabled(False)
		
	def updateFreq(self):
		#grab current dial value 
		self.freq = self.FreqDial.sliderPosition()
		#update LCD display to this value
		self.FreqLCD.display((self.freq/2)-0.5)
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def updateOffset(self):
		#grab current spin box value
		self.offset=self.OffsetInput.value()		
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def updateGain(self):
		#grab current spin box value
		self.gain=self.GainInput.value()		
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def timerExperation(self):
		#every experation of the timer (as defined by update_freq)
		#this function will run. If udpates have been made to any of the 
		#four local parameters (freq, gain, offset, op_code)
		#a new message will be sent to the launchpad via the I2C 
		#interface to make these updates.
		if(self.dataHasChanged):
			#reset flag
			self.dataHasChanged = False
			
			#send new settings to the launchpad
			self.writeSettingsToGenerator()
			
	def writeSettingsToGenerator(self,debug=False):
		self.command = 0
		
		self.msg_data = self.createMsg(freq=self.freq, gain=self.gain, offset=self.offset, op_code=self.op_code)
		self.sendMsg(msg=self.msg_data)
	
	def sendMsg(self, msg, debug=False):
		if debug:
			print("Write Test Data: ",self.msg_data)
		try:
			self.bus.write_i2c_block_data(self.DEVICE_ADDR, self.command, self.msg_data)
			self.updateSysStatus()
		except:
			self.updateSysStatus("I2C Write Error. Please Check Connections")
			
	def dataConversionForTransfer(self,value):
		r = "{0:#0{1}x}".format(value,6)	
		topBits = int(r[:-2], 16)
		lowerBits = int("0x"+r[4:], 16)
		return topBits, lowerBits
		
	def channelShiftToCode(self, channel_val):
		'''Channel Shift Codes to Pass Negatives
			Shift Value 	|		Transmit Code
				-1						0
				 0						1
				 1						2
		'''
		if(channel_val==-1):
			return 0
		elif(channel_val==0):
			return 1
		elif(channel_val==1):
			return 2
			
	def generateSignData(self):
			#Establish ThreePhaseSign (8-bit word used to pass data sign information)
			#If flag == 1 then the corresponding data is negative
			'''ThreePhaseSign
			BIT		:		Data
			1		:	General Offset 
			2		:	Channel 1 Offset
			3		:	Channel 2 Offset
			4		:	Channel	3 Offset
			5		:	Channel	1 Gain
			6		:	Channel	2 Gain
			7		:	Channel 3 Gain
			8		:		RSV			'''
			ThreePhaseSign = 0x00
			
			if(self.offset < 0):
				ThreePhaseSign = ThreePhaseSign | 0x01
			if(self.channel_1_shift_value < 0):
				ThreePhaseSign = ThreePhaseSign | 0x02
			if(self.channel_2_shift_value < 0):
				ThreePhaseSign = ThreePhaseSign | 0x04
			if(self.channel_3_shift_value < 0):
				ThreePhaseSign = ThreePhaseSign | 0x08
			if(self.channel_1_mult_value < 0):
				ThreePhaseSign = ThreePhaseSign | 0x10
			if(self.channel_2_mult_value < 0):
				ThreePhaseSign = ThreePhaseSign | 0x20
			if(self.channel_3_mult_value < 0):
				ThreePhaseSign = ThreePhaseSign | 0x40
			print("Sign Word: ",bin(ThreePhaseSign))
			return ThreePhaseSign
	
	def createMsg(self,
					freq			= 3, 
					gain			= 0.8003, 
					offset			= 0, 
					op_code			= 3,
					channel1_shift 	= 0,
					channel2_shift 	= 0,
					channel3_shift 	= 0,
					channel1_mult 	= 0.0,
					channel2_mult 	= 0.0,
					channel3_mult 	= 0.0,
					debug			= False):
		#limited to 8bits transfered in each index(i.e: [0xFF,0x43 ....])			
		'''MSG structure:
			data = [
			0		: 	Command
			1		:	Buffer
			2		:	Freq MSB
			3		:	Freq LSB
			4		:	Buffer
			5		:	Gain
			6		:	ThreePhaseSign
			7		:	Offset
			8		:	Op_Code
			9		:	Channel 1 Offset
			10		:	Channel 2 Offset
			11		:	Channel 3 Offset
			12		:	Channel 1 Multiplier
			13		:	Channel 2 Multiplier
			14		:	Channel 3 Multiplier
			15		: 	Buffer	
			]
			'''
		bufferVal = 0
		
		#Add Buffer after Command
		msg = [bufferVal]
		if debug:
			print(msg)
			
		#Add Freq and Buffer
		MSB,LSB = self.dataConversionForTransfer(freq)
		msg.append(MSB)
		msg.append(LSB)	
		msg.append(bufferVal)
		if debug:
			print(msg)
			
		#Add Gain and Buffer
		MSB,LSB = self.dataConversionForTransfer(int(gain*100)) #multiplied by 100 to avoid decimals
		#msg.append(MSB) nothing should be in this bit gain is 0-1 (0-10 after *10)
		msg.append(LSB)	
		if debug:
			print(msg)
			
		#Add All Sign Data, general offset Value, and Buffer
		ThreePhaseSignData = self.generateSignData()
		msg.append(ThreePhaseSignData)
		MSB,LSB = self.dataConversionForTransfer(int(offset*100)) #multiplied by 100 to avoid decimals
		#msg.append(MSB) #nothing should be in this value offset is -1 <-> 1
		msg.append(LSB)	
		if debug:
			print(msg)
			
		#Add Op_code and Buffer
		MSB,LSB = self.dataConversionForTransfer(op_code)
		#msg.append(MSB) Nothing in this bit
		msg.append(LSB)	
		if debug:
			print(msg)
		
		#Add channel 1-3 Offset
		MSB,LSB = self.dataConversionForTransfer(int(self.channel_1_shift_value * 100))
		msg.append(LSB)
		MSB,LSB = self.dataConversionForTransfer(int(self.channel_2_shift_value * 100))
		msg.append(LSB)
		MSB,LSB = self.dataConversionForTransfer(int(self.channel_3_shift_value * 100))
		msg.append(LSB)
		'''
		msg.append(int(self.channel_1_shift_value * 100))
		msg.append(int(self.channel_2_shift_value * 100))
		msg.append(int(self.channel_3_shift_value * 100))'''
		if debug:
			print(msg)
			
		#Add channel 1-3 Multipliers and Buffer
		MSB,LSB = self.dataConversionForTransfer(int(self.channel_1_mult_value * 100))
		msg.append(LSB)
		MSB,LSB = self.dataConversionForTransfer(int(self.channel_2_mult_value * 100))
		msg.append(LSB)
		MSB,LSB = self.dataConversionForTransfer(int(self.channel_3_mult_value * 100))
		msg.append(LSB)
		'''msg.append(int(self.channel_1_mult_value * 100))
		msg.append(int(self.channel_2_mult_value * 100))
		msg.append(int(self.channel_3_mult_value * 100))'''
		msg.append(bufferVal)
		if debug:
			print(msg)
			print(len(msg))
		
		self.generateSignData()
		return msg
		
		
	def closeEvent(self,event):
		#send default msg to reset all values and stop the generator if it is running
		self.msg_data = self.createMsg()
		self.sendMsg(msg=self.msg_data)
		#close the I2C bus
		self.bus.close()
		

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())
