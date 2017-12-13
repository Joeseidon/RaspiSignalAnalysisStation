#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  ThreePhase.py
#  
#  Copyright 2017  <pi@raspberrypi>
#
#  Author: Joseph Cutino
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
		self.freq = 2001 #(1000hz) 
		self.freqStep = 20 #1hz
		self.gain = 0.8003
		self.offset = 0
		self.op_code = 1 #idle
		self.channel_1_offset_value = 0
		self.channel_2_offset_value = 0
		self.channel_3_offset_value = 0
		self.channel_1_gain_value = 0.0
		self.channel_2_gain_value = 0.0
		self.channel_3_gain_value = 0.0
		self.channel_1_phase_value = 0
		self.channel_2_phase_value = 0
		self.channel_3_phase_value = 0
		#Holds encoded phase change
		'''phaseData
			BIT		:		Data
			0		:		buf 
			1		:	Channel 1 UP
			2		:	Channel 1 DOWN
			3		:	Channel	2 UP
			4		:	Channel	2 DOWN
			5		:	Channel	3 UP
			6		:	Channel 3 DOWN
			7		:		buf			'''
		self.phaseData = 0x00
		
		#Create necessary local vars
		self.update_freq = 1000 #time in msec for timer experation
		self.phaseUpdateFreq = 500
		self.msg_data = []
		self.dataHasChanged = False
				
		#Connect GUI features to functionality
			#connect frequency input
		self.Increase_Freq_Btn.clicked.connect(self.increaseFreq)
		self.Decrease_Freq_Btn.clicked.connect(self.decreaseFreq)
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
		self.Channel1_offset.valueChanged.connect(self.channelOffsetUpdate)
		self.Channel2_offset.valueChanged.connect(self.channelOffsetUpdate)
		self.Channel3_offset.valueChanged.connect(self.channelOffsetUpdate)
			#connect channel multipliers
		self.Channel1_gain.valueChanged.connect(self.channelGainUpdate)
		self.Channel2_gain.valueChanged.connect(self.channelGainUpdate)
		self.Channel3_gain.valueChanged.connect(self.channelGainUpdate)
			#connect reset button
		self.ResetBtn.clicked.connect(self.resetGenData)
			#Update LCD to default Freq
		self.FreqLCD.display(int((self.freq/2)-0.5))
			#Connect Multipliers
		self.FreqMult5.toggled.connect(lambda:self.setFreqStep(10))
		self.FreqMult10.toggled.connect(lambda:self.setFreqStep(20))
		self.FreqMult100.toggled.connect(lambda:self.setFreqStep(200))
		self.FreqMult1000.toggled.connect(lambda:self.setFreqStep(2000))
			#Connect Channel Phase Shift
		self.Channel1_phase.valueChanged.connect(self.channelPhaseUpdate)
		self.Channel2_phase.valueChanged.connect(self.channelPhaseUpdate)
		self.Channel3_phase.valueChanged.connect(self.channelPhaseUpdate)
			#This values can only be changed while the system is running. Disable until start
		self.Channel1_phase.setEnabled(False)
		self.Channel2_phase.setEnabled(False)
		self.Channel3_phase.setEnabled(False)
		
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
		
		self.phaseResetTimer = QtCore.QTimer()
		self.phaseResetTimer.setInterval(self.phaseUpdateFreq)
		self.phaseResetTimer.timeout.connect(self.phaseCorrect)
		self.phaseResetTimer.setSingleShot(True)
		
	def channelPhaseUpdate(self):
		'''phaseData
			BIT		:		Data
			0		:		buf 
			1		:	Channel 1 UP
			2		:	Channel 1 DOWN
			3		:	Channel	2 UP
			4		:	Channel	2 DOWN
			5		:	Channel	3 UP
			6		:	Channel 3 DOWN
			7		:		buf			'''
		#Disable until after data sent. Prevents GUI and DPS from getting out of synch
		self.Channel1_phase.setEnabled(False)
		self.Channel2_phase.setEnabled(False)
		self.Channel3_phase.setEnabled(False)
		#print("Old Data: ", self.channel_1_phase_value, " ", self.channel_2_phase_value, " ", self.channel_3_phase_value)
		#print("New Data: ", self.Channel1_phase.value(), " ",self.Channel2_phase.value(), " ", self.Channel3_phase.value())
		#reset phase data
		self.phaseData = 0x00

		#compare new and old data fro each channel 
		if(self.channel_1_phase_value < self.Channel1_phase.value()):
			self.phaseData = self.phaseData | 0x04
		if(self.channel_1_phase_value > self.Channel1_phase.value()):
			self.phaseData = self.phaseData | 0x02
		if(self.channel_2_phase_value < self.Channel2_phase.value()):
			self.phaseData = self.phaseData | 0x10
		if(self.channel_2_phase_value > self.Channel2_phase.value()):
			self.phaseData = self.phaseData | 0x08
		if(self.channel_3_phase_value < self.Channel3_phase.value()):
			self.phaseData = self.phaseData | 0x40
		if(self.channel_3_phase_value > self.Channel3_phase.value()):
			self.phaseData = self.phaseData | 0x20
			
			
		#update data
		self.channel_1_phase_value = self.Channel1_phase.value()
		self.channel_2_phase_value = self.Channel2_phase.value()
		self.channel_3_phase_value = self.Channel3_phase.value()
		
		#print(hex(self.phaseData))
		
		#set global update value
		self.dataHasChanged = True
		
	def setFreqStep(self, mult):
		self.freqStep = mult
		
	def increaseFreq(self):
		self.freq = self.freq + self.freqStep
		if(self.freq<0):
			self.freq=0
		if(self.freq>20001):
			self.freq=20001
		#update LCD display to this value
		self.FreqLCD.display(int((self.freq/2)-0.5))
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def decreaseFreq(self):
		self.freq = self.freq - self.freqStep
		if(self.freq<0):
			self.freq=0
		if(self.freq>20001):
			self.freq=20001
		#update LCD display to this value
		self.FreqLCD.display(int((self.freq/2)-0.5))
		#set global update value so that on timer experation new data is sent
		self.dataHasChanged = True
		
	def resetGenData(self):
		#reset 3 phase generator settings
		self.freq=2001
		self.FreqLCD.display(int((self.freq/2)-0.5))
		self.OffsetInput.setValue(0.0)
		self.GainInput.setValue(0.8)
		self.Startbtn.setEnabled(True)
		self.Stopbtn.setEnabled(False)
		self.Channel1_offset.setValue(0)
		self.Channel2_offset.setValue(0)
		self.Channel3_offset.setValue(0)
		self.Channel1_gain.setValue(0.0)
		self.Channel2_gain.setValue(0.0)
		self.Channel3_gain.setValue(0.0)
		self.Channel1_phase.setValue(0)
		self.Channel2_phase.setValue(0)
		self.Channel3_phase.setValue(0)
		self.phaseData = 0x7E
		self.op_code = 3
		#set global update value
		self.dataHasChanged = True
		
	def channelOffsetUpdate(self):
		#update all channel offset values
		self.channel_1_offset_value = self.Channel1_offset.value()
		self.channel_2_offset_value = self.Channel2_offset.value()
		self.channel_3_offset_value = self.Channel3_offset.value()
		#set global update value
		self.dataHasChanged = True
	
	def channelGainUpdate(self):
		#update all channel gain values
		self.channel_1_gain_value = self.Channel1_gain.value()
		self.channel_2_gain_value = self.Channel2_gain.value()
		self.channel_3_gain_value = self.Channel3_gain.value()
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
		#Enable Phase Adjustment
		self.Channel1_phase.setEnabled(True)
		self.Channel2_phase.setEnabled(True)
		self.Channel3_phase.setEnabled(True)
		
	def stopBtnPress(self):
		#update op_code
		self.op_code = 3 #stop, cleanup, and then idle
		#set global update value
		self.dataHasChanged = True
		#disable stopBtn and enable startBtn
		self.Startbtn.setEnabled(True)
		self.Stopbtn.setEnabled(False)
		#Disable Phase Adjustment
		self.Channel1_phase.setEnabled(False)
		self.Channel2_phase.setEnabled(False)
		self.Channel3_phase.setEnabled(False)
	
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
	
	def phaseCorrect(self):
		#Reset phase data to clear phase changes on second send 
		self.phaseData = 0x00
		#Called from timerExperation after every data update 
		#Will resend data to correct phase after 0.5sec

		#send new settings to the launchpad
		self.writeSettingsToGenerator()
		
		#Reenable phase adjustment
		if(self.op_code == 2):
			self.Channel1_phase.setEnabled(True)
			self.Channel2_phase.setEnabled(True)
			self.Channel3_phase.setEnabled(True)
		
		
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
			
			#start phase timer for phase reset
			self.phaseResetTimer.start()
			
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
			if(self.channel_1_offset_value < 0):
				ThreePhaseSign = ThreePhaseSign | 0x02
			if(self.channel_2_offset_value < 0):
				ThreePhaseSign = ThreePhaseSign | 0x04
			if(self.channel_3_offset_value < 0):
				ThreePhaseSign = ThreePhaseSign | 0x08
			if(self.channel_1_gain_value < 0):
				ThreePhaseSign = ThreePhaseSign | 0x10
			if(self.channel_2_gain_value < 0):
				ThreePhaseSign = ThreePhaseSign | 0x20
			if(self.channel_3_gain_value < 0):
				ThreePhaseSign = ThreePhaseSign | 0x40

			return ThreePhaseSign
	
	def createMsg(self,
					freq			= 2000, 
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
			4		:	Phase Data
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
		if debug:
			print(msg)
		
		#Add Phase Data
		msg.append(self.phaseData)
		if debug:
			print(msg)
			
		#Add Gain and Buffer
		MSB,LSB = self.dataConversionForTransfer(int(gain*100)) #multiplied by 100 to avoid decimals
		msg.append(LSB)	
		if debug:
			print(msg)
			
		#Add All Sign Data, general offset Value, and Buffer
		ThreePhaseSignData = self.generateSignData()
		msg.append(ThreePhaseSignData)
		MSB,LSB = self.dataConversionForTransfer(int(offset*100)) #multiplied by 100 to avoid decimals
		msg.append(LSB)	
		if debug:
			print(msg)
			
		#Add Op_code and Buffer
		MSB,LSB = self.dataConversionForTransfer(op_code)
		msg.append(LSB)	
		if debug:
			print(msg)
		
		#Add channel 1-3 Offset
		MSB,LSB = self.dataConversionForTransfer(int(self.channel_1_offset_value * 100))
		msg.append(LSB)
		MSB,LSB = self.dataConversionForTransfer(int(self.channel_2_offset_value * 100))
		msg.append(LSB)
		MSB,LSB = self.dataConversionForTransfer(int(self.channel_3_offset_value * 100))
		msg.append(LSB)
		if debug:
			print(msg)
			
		#Add channel 1-3 Multipliers and Buffer
		MSB,LSB = self.dataConversionForTransfer(int(self.channel_1_gain_value * 100))
		msg.append(LSB)
		MSB,LSB = self.dataConversionForTransfer(int(self.channel_2_gain_value * 100))
		msg.append(LSB)
		MSB,LSB = self.dataConversionForTransfer(int(self.channel_3_gain_value * 100))
		msg.append(LSB)
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
    app.setStyle("GTK+")
    window = MyWindow()
    sys.exit(app.exec_())
