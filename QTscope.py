import sys
import os
import subprocess
import time

from functools import partial

try:
    from PyQt4 import QtGui, QtCore, uic
except ImportError:
    os.system("lxterminal -e 'python QTinstall.py'")
    sys.exit() 

try:
    import piplates.DAQC2plate as DAQC2
except ImportError:
    os.system("lxterminal -e 'python py23install.py'")
    sys.exit()

class MyWindow(QtGui.QMainWindow):  
    def __init__(self):
        super(MyWindow, self).__init__()
        uic.loadUi('Oscilloscope.ui', self)
        self.show()
        qr = self.frameGeometry()
        cp = QtGui.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

        self.clip_rect=QtCore.QRect(0,0,999,639)
        
        self.addr=0
        self.initADDR2()
        
        self.tChannel1=oChannel(1,QtCore.Qt.yellow,self.Chan1Vertical,self.Position1,self.Enable1,self.Chan1Sensitivity)
        self.tChannel2=oChannel(2,QtCore.Qt.cyan,self.Chan2Vertical,self.Position2,self.Enable2,self.Chan2Sensitivity)
        
        self.sRate=9
        self.trigSource=1
        self.trigLevel=0.00
        self.trigType=0     #0 for auto and 1 for Edge
        self.trigEdge=0     #0 for Rising and 1 for Falling

        self.autoCounter=0
        
        self.rateStrings=['1 ','500 m','200 m','100 m','50 m','20 m','10 m','5 m','2 m','1 m','500 u','200 u','100 u','50 u','20 u','10 u']
        self.upDateSweepRate()
        self.upDateTrigger()
        self.tChannel1.upDate()
        self.tChannel2.upDate()        
        self.looptimer = QtCore.QTimer()
        self.intmontimer = QtCore.QTimer()
        self.intmontimer.timeout.connect(self.monInt)
        self.looptimer.timeout.connect(self.Sweep)
        self.looptimer.start(50)

        self.connect(self.SweepRate, QtCore.SIGNAL('valueChanged(int)'), self.upDateSweepRate)
        self.connect(self.TriggerLevel, QtCore.SIGNAL('valueChanged(int)'), self.upDateTrigger)
        self.connect(self.Chan1Vertical, QtCore.SIGNAL('valueChanged(int)'), self.tChannel1.upDate)  
        self.connect(self.Chan2Vertical, QtCore.SIGNAL('valueChanged(int)'), self.tChannel2.upDate)
        self.Enable1.clicked.connect(self.tChannel1.upDate)
        self.Enable2.clicked.connect(self.tChannel2.upDate)

        self.HelpButton.clicked.connect(self.getHelp) 

    def Sweep(self):
        sr_index=self.SweepRate.sliderPosition()
        if (sr_index>12):
            sr_index=12
        if (self.tChannel1.getStatus() or self.tChannel2.getStatus()):
            DAQC2.startOSC(self.addr)
            DAQC2.setOSCchannel(self.addr, (1 if self.tChannel1.getStatus() else 0),(1 if self.tChannel2.getStatus() else 0))
            self.upDateTrigger()
            DAQC2.setOSCsweep(self.addr,sr_index)    
            DAQC2.runOSC(self.addr)
            self.looptimer.stop()
            self.autoCounter=50
            self.intmontimer.start(10)
            
    def monInt(self):
        if(DAQC2.GPIO.input(22)==0):
            DAQC2.getINTflags(self.addr)
            DAQC2.getOSCtraces(self.addr)
            self.intmontimer.stop()
            self.update()             
            DAQC2.stopOSC(self.addr)
            self.looptimer.start(20)
        else:
            if (self.tTypeEdge.isChecked()==False):
                self.autoCounter-=1
                if (self.autoCounter==0):
                    DAQC2.trigOSCnow(self.addr)
        
    def upDateSweepRate(self):
        self.rRate=self.SweepRate.sliderPosition()
        self.sweepRate.setText('Sweep Rate: '+self.rateStrings[self.rRate]+'sec/div')

    def upDateTrigger(self):
        self.trigLevel=self.TriggerLevel.sliderPosition()/100.0
        self.trigLevelLabel.setText('Trigger Level: '+str(self.trigLevel))
        tLevel=int(self.TriggerLevel.sliderPosition()/1200.0*2048+2048)
        if (tLevel>4095):
            tLevel=4095
        if (tLevel<0):
            tLevel=0       
        if (self.tSource1.isChecked()):
            self.trigSource=1
        else:
            self.trigSource=2
        if (self.tTypeEdge.isChecked()):
            tType='normal'
        else:
            tType='auto'
        if (self.tEdgeRising.isChecked()):
            tEdge='rising'
        else:
            tEdge='falling'
        DAQC2.setOSCtrigger(self.addr,self.trigSource,tType,tEdge,tLevel)
            
    def paintEvent(self, e):	
        paint = QtGui.QPainter()
        paint.begin(self)
        paint.setBrush(QtCore.Qt.black)
        paint.drawRect(0,0,999,639)
        paint.setClipRect(self.clip_rect)
        self.drawGrid(paint)
        self.drawTraces(paint)
        self.drawTrigger(paint)
        paint.end()
        
    def drawTraces(self,paint):
        self.OneChanCount=[1000,500,200,100]
        self.OneChanPitch=[1,2,5,10]
        self.TwoChanCount=[500,250,100,50]
        self.TwoChanPitch=[2,4,10,20]    
        self.pCount=1000
        self.pitch=1
        if(self.rRate>11):
            if(self.tChannel1.getStatus() and self.tChannel2.getStatus()):
                self.pCount=self.TwoChanCount[self.rRate-12]
                self.pitch=self.TwoChanPitch[self.rRate-12]        
            else:
                self.pCount=self.OneChanCount[self.rRate-12]
                self.pitch=self.OneChanPitch[self.rRate-12]
        if(self.tChannel1.getStatus()):
           self.tChannel1.drawTrace(paint,DAQC2.trace1,self.pitch,self.pCount)
        if(self.tChannel2.getStatus()):
           self.tChannel2.drawTrace(paint,DAQC2.trace2,self.pitch,self.pCount)           

    def drawTrigger(self, paint):
        if ((self.trigSource==1) and self.tChannel1.getStatus()):
            self.tChannel1.drawTrigger(paint,self.TriggerLevel.sliderPosition())
        if ((self.trigSource==2) and self.tChannel2.getStatus()):
            self.tChannel2.drawTrigger(paint,self.TriggerLevel.sliderPosition())        

    def drawGrid(self,qp):
        qp.setPen(QtCore.Qt.gray)
        tSize=3
        qp.drawLine(0,320,999,320)
        qp.drawLine(500,0,500,639)
        for i in range(50):
            qp.drawLine(20*i,320-tSize,20*i,320+tSize+1)
            qp.drawLine(20*i,0,20*i,tSize+1)   
            qp.drawLine(20*i,639-tSize-1,20*i,639)
        for i in range(40): 
            qp.drawLine(500-tSize,16*i,500+tSize+1,16*i)  
            qp.drawLine(0,16*i,tSize+1,16*i)
        qp.pen=QtGui.QPen(QtCore.Qt.gray)
        patList=[1.0,7.0]
        qp.pen.setDashPattern(patList)
        qp.pen.setStyle(QtCore.Qt.CustomDashLine)
        qp.setPen(qp.pen)
        for i in range(4):
            qp.drawLine(100*(i+1),0,100*(i+1),639)
            qp.drawLine(100*(i+6),0,100*(i+6),639)
        patList=[1.0,9.0]            
        qp.pen.setDashPattern(patList)
        qp.pen.setStyle(QtCore.Qt.CustomDashLine)
        qp.setPen(qp.pen)        
        for i in range(3):
            qp.drawLine(0,80*(i+1),999,80*(i+1))
            qp.drawLine(0,80*(i+5),999,80*(i+5))  

    def initADDR2(self):
        addrSet=False
        addresses = [False,False,False,False,False,False,False,False]
        for i in range(8):
            tempADDR=DAQC2.getADDR(i)
            if(tempADDR==i):
                addresses[i]=True
                if(addrSet==False):
                    self.setAddr(i)
                    addrSet=True
                    self.ADDRlabel.setText('Running on Address '+str(self.addr))
        
    def setAddr(self, value):
            self.addr=value         

    def getHelp(self):
        cmd_line = "xpdf DAQC2plateOscilloscopeManual.pdf"
        p = subprocess.Popen(cmd_line, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        out = p.communicate()[0]

    def closeEvent(self,event):
        self.intmontimer.stop()
        self.looptimer.stop()
        ok=False
        while(ok == False):
            DAQC2.stopOSC(self.addr)
            if (DAQC2.DataGood==True):
                ok=True
        DAQC2.RESET(self.addr)

class oChannel:
    def  __init__(self,number,color,vKnob,pKnob,enButton,sLabel):
        self.color=color
        self.number=number
        self.sensitivity=0
        self.position=320
        self.verticalKnob=vKnob
        self.positionKnob=pKnob
        self.enableButton=enButton
        self.senseText=sLabel
        self.enabled=False
        self.sensitivityStrings=['10 m','20 m','50 m','100 m','200 m','500 m','1 ','2 ','5 ']
        self.sensitivityValues=[0.01,0.02,0.05,0.1,0.2,0.5,1,2,5]
        self.upDate()

    def upDate(self):
        index=8-self.verticalKnob.sliderPosition()
        self.sensitivity=self.sensitivityValues[index]
        self.senseText.setText('Channel '+str(self.number)+': '+self.sensitivityStrings[index]+'V/div')
        if (self.enableButton.isChecked()):
            self.enabled=True
        else:
            self.enabled=False    

    def getStatus(self):
        return self.enabled
        
    def drawTrace(self,paint,data,pitch,pcount):
        yoff=639-self.positionKnob.sliderPosition()
        patList=[2.0,8.0]
        paint.pen.setDashPattern(patList)
        paint.pen.setStyle(QtCore.Qt.CustomDashLine)
        paint.pen.setColor(self.color)
        paint.pen.setWidth(1)
        paint.setPen(paint.pen)
        paint.drawLine(0,yoff,999,yoff)             
        pt=list(range(pcount))            
        for i in range(pcount):
            x=i*pitch
            # In the following statement, we scale and invert the data in y since the point 0,0 
            # is in the upper lefthand corner
            y=yoff-(data[i]-2048)*12.0/(4*self.sensitivity)*320/2048                
            pt[i]=QtCore.QPoint(x,y)
        needle=QtGui.QPolygon(pt)
        marker=self.drawMarker(yoff)
        paint.pen.setStyle(QtCore.Qt.SolidLine)
        paint.setPen(self.color)
        paint.setBrush(self.color)
        paint.pen.setWidth(2)
        paint.setPen(paint.pen)
        paint.drawPolyline(needle)
        paint.drawPolygon(marker)
        paint.setPen(QtCore.Qt.black)
        paint.drawText(QtCore.QPoint(996-5,yoff+7),str(self.number))  

    def drawTrigger(self,paint,tKnob):
        toff=-int(tKnob/100.0/self.sensitivity*80)
        yoff=639-self.positionKnob.sliderPosition()
        patList=[1.0,9.0]
        paint.pen.setDashPattern(patList)
        paint.pen.setStyle(QtCore.Qt.CustomDashLine)
        paint.setBrush(self.color)
        paint.pen.setColor(self.color)
        paint.pen.setWidth(1)
        paint.setPen(paint.pen)
        paint.drawLine(0,yoff+toff,999,yoff+toff)
        marker=self.drawMarker(yoff+toff)
        paint.pen.setStyle(QtCore.Qt.SolidLine)
        paint.drawPolygon(marker)                
        paint.setPen(QtCore.Qt.black)
        paint.drawText(QtCore.QPoint(996-5,yoff+toff+7),"T")       
    
    def drawMarker(self,yoff):
        pH=10
        pW=15
        marker = QtGui.QPolygon()
        marker.append(QtCore.QPoint(999,pH+yoff))
        marker.append(QtCore.QPoint(999-pW,yoff))
        marker.append(QtCore.QPoint(999,yoff-pH))
        return marker
    
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())        
