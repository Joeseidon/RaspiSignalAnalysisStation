import sys
import os
import subprocess

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
        uic.loadUi('DAQC2Motor.ui', self)
        self.show()
        qr = self.frameGeometry()
        cp = QtGui.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        self.addr=0
        self.initADDR2()

        self.mOFF=0
        self.mON=1
        self.mSTOP=2
        self.mMOVE=3
        self.mJOG=4

        self.mState1=self.mOFF
        self.mState2=self.mOFF
        #init step rate
        self.SRval1=1
        self.SRval2=1
        #init step counts
        self.SCval1=1
        self.SCval2=1
        #init directions
        self.SDval1='cw'
        self.SDval2='cw'
        #init step size
        self.SSval1='w'
        self.SSval2='w'

        self.connect(self.SR1, QtCore.SIGNAL('valueChanged(int)'), self.upDateSR1)
        self.connect(self.SR2, QtCore.SIGNAL('valueChanged(int)'), self.upDateSR2)
        self.connect(self.SC1, QtCore.SIGNAL('valueChanged(int)'), self.upDateSC1)
        self.connect(self.SC2, QtCore.SIGNAL('valueChanged(int)'), self.upDateSC2)

        self.WHOLE1.clicked.connect(partial(self.setSize1,value='w'))
        self.WHOLE2.clicked.connect(partial(self.setSize2,value='w'))
        self.HALF1.clicked.connect(partial(self.setSize1,value='h'))
        self.HALF2.clicked.connect(partial(self.setSize2,value='h'))

        self.CW1.clicked.connect(partial(self.setDir1,value='cw'))
        self.CW2.clicked.connect(partial(self.setDir2,value='cw'))
        self.CCW1.clicked.connect(partial(self.setDir1,value='ccw'))
        self.CCW2.clicked.connect(partial(self.setDir2,value='ccw'))                                

        self.MOVE1.clicked.connect(self.handleMove1)
        self.MOVE2.clicked.connect(self.handleMove2)
        self.JOG1.clicked.connect(self.handleJog1)
        self.JOG2.clicked.connect(self.handleJog2)
        self.STOP1.clicked.connect(self.handleStop1)
        self.STOP2.clicked.connect(self.handleStop2)        
        self.OFF1.clicked.connect(self.handleOff1)
        self.OFF2.clicked.connect(self.handleOff2) 

        self.HelpButton.clicked.connect(self.getHelp)         
        
    def setDir1(self,value):
        #self.chkMotor1()
        self.SDval1=value
        DAQC2.motorDIR(self.addr,1,self.SDval1)
        
    def setDir2(self,value):
        #self.chkMotor2()        
        self.SDval2=value
        DAQC2.motorDIR(self.addr,2,self.SDval2)
        
    def setSize1(self,value):
        #self.chkMotor1()        
        self.SSval1=value
        DAQC2.motorRATE(self.addr,1,self.SRval1,self.SSval1)
        
    def setSize2(self,value):
        #self.chkMotor2()        
        self.SSval2=value        
        DAQC2.motorRATE(self.addr,2,self.SRval2,self.SSval2)
        
    def handleMove1(self):
        if (self.mState1==self.mOFF):
            self.mState1=self.mON
        DAQC2.motorMOVE(self.addr,1,self.SCval1)

    def handleMove2(self):
        if (self.mState2==self.mOFF):
            self.mState2=self.mON
        DAQC2.motorMOVE(self.addr,2,self.SCval2)        

    def handleJog1(self):
        DAQC2.motorJOG(self.addr,1)

    def handleJog2(self):
        DAQC2.motorJOG(self.addr,2)

    def handleStop1(self):
        DAQC2.motorSTOP(self.addr,1)

    def handleStop2(self):
        DAQC2.motorSTOP(self.addr,2)
        
    def handleOff1(self):
        DAQC2.motorOFF(self.addr,1)

    def handleOff2(self):
        DAQC2.motorOFF(self.addr,2)
        
    def upDateSR1(self):
        #self.chkMotor1()        
        self.SRval1=self.SR1.sliderPosition()
        self.SRD1.display(self.SRval1)
        DAQC2.motorRATE(self.addr,1,self.SRval1,self.SSval1)

    def upDateSR2(self):
        #self.chkMotor2()        
        self.SRval2=self.SR2.sliderPosition()
        self.SRD2.display(self.SRval2)
        DAQC2.motorRATE(self.addr,2,self.SRval2,self.SSval2)
        
    def upDateSC1(self):
        #self.chkMotor1()        
        self.SCval1=self.SC1.sliderPosition()
        self.SCD1.display(self.SCval1)   

    def upDateSC2(self):
        #self.chkMotor2()        
        self.SCval2=self.SC2.sliderPosition()
        self.SCD2.display(self.SCval2) 

    def chkMotor1(self):
        if (self.mState1==self.mOFF):
            self.mState1=self.mON
            DAQC2.motorON(self.addr,1)
        
    def chkMotor2(self):
        if (self.mState2==self.mOFF):
            self.mState2=self.mON
            DAQC2.motorON(self.addr,2)

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
        DAQC2.motorDISABLE(self.addr)                
        DAQC2.motorENABLE(self.addr)
            
    def getHelp(self):
        cmd_line = "xpdf DAQC2plateMotorControllerManual.pdf"
        p = subprocess.Popen(cmd_line, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        out = p.communicate()[0]

    def closeEvent(self,event):
            DAQC2.motorDISABLE(self.addr)
        

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())

    
