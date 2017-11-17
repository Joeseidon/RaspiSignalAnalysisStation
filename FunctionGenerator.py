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
        uic.loadUi('FunctionGenerator.ui', self)
        self.show()
        qr = self.frameGeometry()
        cp = QtGui.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        self.addr=0
        self.initADDR2()
        
        self.Ch1Mult=100
        self.Ch2Mult=100
        self.Freq1=1000
        self.Freq2=1000

        self.InitFG()
        
        self.WaveButtonSine1.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=1,type=1))
        self.WaveButtonTriangle1.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=1,type=2))
        self.WaveButtonSquare1.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=1,type=3))
        self.WaveButtonRSaw1.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=1,type=4))
        self.WaveButtonFSaw1.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=1,type=5))
        self.WaveButtonNoise1.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=1,type=6))
        self.WaveButtonSinc1.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=1,type=7))       

        self.WaveButtonSine2.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=2,type=1))
        self.WaveButtonTriangle2.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=2,type=2))
        self.WaveButtonSquare2.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=2,type=3))
        self.WaveButtonRSaw2.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=2,type=4))
        self.WaveButtonFSaw2.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=2,type=5))
        self.WaveButtonNoise2.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=2,type=6))
        self.WaveButtonSinc2.clicked.connect(partial(DAQC2.fgTYPE,addr=self.addr,chan=2,type=7))

        self.connect(self.Frequency1, QtCore.SIGNAL('valueChanged(int)'), self.upDateFreq1)
        self.connect(self.Frequency2, QtCore.SIGNAL('valueChanged(int)'), self.upDateFreq2)
        self.radioButtonX10Ch1.clicked.connect(partial(self.multiplierUpdate,channel=1,value=10))
        self.radioButtonX100Ch1.clicked.connect(partial(self.multiplierUpdate,channel=1,value=100))
        self.radioButtonX10Ch2.clicked.connect(partial(self.multiplierUpdate,channel=2,value=10))
        self.radioButtonX100Ch2.clicked.connect(partial(self.multiplierUpdate,channel=2,value=100))

        self.sigDivBy1_1.clicked.connect(partial(DAQC2.fgLEVEL,addr=self.addr,chan=1,level=4))
        self.sigDivBy2_1.clicked.connect(partial(DAQC2.fgLEVEL,addr=self.addr,chan=1,level=3))
        self.sigDivBy4_1.clicked.connect(partial(DAQC2.fgLEVEL,addr=self.addr,chan=1,level=2))
        self.sigDivBy8_1.clicked.connect(partial(DAQC2.fgLEVEL,addr=self.addr,chan=1,level=1))
        self.sigDivBy1_2.clicked.connect(partial(DAQC2.fgLEVEL,addr=self.addr,chan=2,level=4))
        self.sigDivBy2_2.clicked.connect(partial(DAQC2.fgLEVEL,addr=self.addr,chan=2,level=3))
        self.sigDivBy4_2.clicked.connect(partial(DAQC2.fgLEVEL,addr=self.addr,chan=2,level=2))
        self.sigDivBy8_2.clicked.connect(partial(DAQC2.fgLEVEL,addr=self.addr,chan=2,level=1))

        self.HelpButton.clicked.connect(self.getHelp) 

    def multiplierUpdate(self,channel,value):
        if (channel==1):
            self.Ch1Mult=value
            self.upDateFreq1()
        else:
            self.Ch2Mult=value
            self.upDateFreq2()

    def upDateFreq1(self):
        self.Freq1=int(self.Ch1Mult*(10**(self.Frequency1.sliderPosition()/1000.0))+0.5)
        self.Ch1Display.display(self.Freq1)   
        DAQC2.fgFREQ(self.addr,1,self.Freq1)
        
    def upDateFreq2(self):
        self.Freq2=int(self.Ch2Mult*(10**(self.Frequency2.sliderPosition()/1000.0))+0.5)
        self.Ch2Display.display(self.Freq2)   
        DAQC2.fgFREQ(self.addr,2,self.Freq2)

    def InitFG(self):
        DAQC2.fgON(self.addr,1)
        DAQC2.fgON(self.addr,2)
        DAQC2.fgFREQ(self.addr,1,self.Freq1)
        DAQC2.fgFREQ(self.addr,2,self.Freq2)        
        DAQC2.fgTYPE(self.addr,1,1)
        DAQC2.fgTYPE(self.addr,2,1)

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
        cmd_line = "xpdf DAQC2plateFunctionGeneratorManual.pdf"
        p = subprocess.Popen(cmd_line, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        out = p.communicate()[0]

    def closeEvent(self,event):
        DAQC2.fgOFF(self.addr,1)
        DAQC2.fgOFF(self.addr,2)



if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())
