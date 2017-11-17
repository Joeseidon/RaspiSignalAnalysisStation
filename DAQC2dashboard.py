import os
import time
import sys

if (sys.version_info > (3, 0)):
    # Python 3 code in this block
    from tkinter import *
else:
    # Python 2 code in this block
    from Tkinter import *

try:
    import piplates.DAQC2plate as DAQC2
except ImportError:
    os.system("lxterminal -e 'python py23install.py'")
    sys.exit()

def callback():
    DAQC2.CLOSE()
    root.destroy()  
  
def task():
    global addr    
    val=DAQC2.getADC(addr,8)
    sval=str('{: 7.3f}'.format(val))
    VccText.set(sval)
    val=DAQC2.getFREQ(addr)
    sval=str('{: 8.2f}'.format(val))
    freqText.set(sval)    
    for i in range(0,8):        
        val=DAQC2.getADC(addr,i)
        sval=str('{: 7.3f}'.format(val))
        analogin[i].set(sval)
        j=DAQC2.getDINbit(addr,i)
        chkivar[i].set(j)        
    root.after(1000,task)

def toggleChk(par):
    global addr
    if chkovar[par].get()==1:       
        DAQC2.setDOUTbit(addr,par)
    else:
        DAQC2.clrDOUTbit(addr,par)

def setDAC(par1,par2):
    global addr
    #print par1,type(par1),par2,type(par2)
    par1=float(par1)
    DAQC2.setDAC(addr,par2,par1)

def setPWM(par1,par2):
    global addr
    par1=float(par1)
    DAQC2.setPWM(addr,par2,par1)


def setADDR():
    global addr
    addr=int(ADDRenter.get())
    ADDRin.set(ADDRenter.get())

def initADDR3():
    global addr
    addrSet=False
    addresses = [False,False,False,False,False,False,False,False]
    for i in range(8):
        tempADDR=DAQC2.getADDR(i)
        if(tempADDR==i):
            addresses[i]=True
            if(addrSet==False):
                addr=i
                addrSet=True
                
addr=0 
initADDR3()
root = Tk()
root.config(bg="black")
root.wm_title("DAQC2plate Dashboard")


##wu = root.winfo_screenwidth()
##hu = root.winfo_screenheight()
##WinH=425
##WinW=500
##x = wu/2 - WinW/2
##y = hu/2 - WinH/2
##root.geometry("%dx%d+%d+%d" % (WinW,WinH,x,y))
root.geometry("+200+200")


af=Frame(root,bg="#000000000",padx=0,pady=0)
af.grid(row=0,column=0, sticky=N+S+W+E)

df=Frame(root,bg="#000000000",padx=0,pady=0)
df.grid(row=0,column=1, sticky=N+S+W+E)


dof=Frame(df,bg="#000000444",padx=4,pady=4)
dof.grid(row=0,column=0, sticky=N+S+W+E)

dif=Frame(df,bg="#444000000",padx=10,pady=5)
dif.grid(row=0,column=1, sticky=N+S+W+E)

dacf=Frame(af,bg="#888888000",padx=5,pady=5)
dacf.grid(row=1,column=0, sticky=N+S+W+E)

adcf=Frame(af,bg="#000444000",padx=5,pady=5)
adcf.grid(row=0,column=0, sticky=W+E)

freqf=Frame(df,bg="#444000444",padx=5,pady=5)
freqf.grid(row=1,column=0, columnspan=2,sticky=W+E)

sysf=Frame(df,bg="black")
sysf.grid(row=2,column=0, columnspan=2)

Label(adcf,text="Analog Inputs",padx=4,pady=4
      ,bg="#000444000",fg="White").grid(row=0,column=0,columnspan=2)
Label(adcf,text="5 Volt Supply:",padx=4,pady=0
      ,bg="#000444000",fg="White").grid(row=1,column=0,sticky=E)

VccText=StringVar()
VccText.set("-----")
Label(adcf,textvariable=VccText,padx=4,pady=0,bg="#000444000",fg="White").grid(row=1,column=1, sticky=W)

analogin = list(range(8))
for i in range(0,8):
    Label(adcf,text="Channel "+str(i)+":"
          ,padx=4,pady=0,bg="#000444000"
          ,fg="White").grid(row=i+2,column=0,sticky=E)    
    analogin[i]=StringVar()
    analogin[i].set("-----")
    Label(adcf,textvariable=analogin[i],padx=4,pady=2,bg="#000444000",fg="White").grid(row=i+2,column=1, sticky=W)

Label(dof,text="Digital Outputs"
      ,padx=4,pady=4,bg="#000000444"
      ,fg="White").grid(row=0,column=0,sticky=N)

chkovar = list(range(8))
for i in range(0,8):
    DAQC2.clrDOUTbit(addr,i)
    chkovar[i]=IntVar()
    chkovar[i].set(0)
    Checkbutton(dof,text="Channel "+str(i)
                ,padx=4,pady=4, indicatoron=1,bd=0, highlightthickness=0
                ,bg="#000000444",fg="White", variable=chkovar[i], selectcolor="blue"
                ,command=lambda j=i: toggleChk(j)).grid(row=i+1,column=0)

Label(dif,text="Digital Inputs"
      ,padx=4, pady=4,bg="#444000000"
      ,fg="White").grid(row=0,column=0,sticky=N)

chkivar = list(range(8))
for i in range(0,8):
    chkivar[i]=IntVar()
    chkivar[i].set(0)
    Label(dif,text="Channel "+str(i)+":"
          ,padx=4,pady=3,bg="#444000000"
          ,fg="White").grid(row=1+i,column=0, sticky=W)
    Label(dif,textvariable=str(chkivar[i])
          ,padx=4,pady=3,bg="#444000000"
          ,fg="White").grid(row=1+i,column=1, sticky=W)

Label(dacf,text="Analog Outputs"
      ,bg="#888888000",fg="White"
      ,padx=4,pady=4).grid(row=0, column=0,columnspan=4)

for i in range(4):
    Label(dacf,text='Chan '+str(i),bg="#888888000",fg="White",padx=4,pady=15).grid(row=2+i,column=0)
    Scale(dacf,orient=HORIZONTAL,from_=0.0,to=4.095,resolution=0.001,length=200
        ,command=lambda event, j=i: setDAC(event,j),bg="#888888000",fg="White").grid(row=2+i,column=1)

Label(freqf,text="Frequency Counter:",padx=4,pady=0
      ,bg="#444000444",fg="White").grid(row=0,column=0,sticky=W)

freqText=StringVar()
freqText.set("-----")
Label(freqf,textvariable=freqText,padx=4,pady=0,bg="#444000444",fg="White").grid(row=0,column=1, sticky=E)


Label(sysf,text="PWM Outputs"
      ,bg="#000000000",fg="White"
      ,padx=4,pady=4).grid(row=1, column=0,columnspan=2)

for i in range(2):
    Label(sysf,text='Chan '+str(i),bg="#000000000",fg="White",padx=4,pady=15).grid(row=2+i,column=0)
    Scale(sysf,orient=HORIZONTAL,from_=0.0,to=100,resolution=1,length=180
        ,command=lambda event, j=i: setPWM(event,j),bg="#000000000",fg="White",).grid(row=2+i,column=1)

Label(sysf,text="Board Address: "+str(addr)
      ,bg="black",fg="White").grid(row=6,column=0,columnspan=2,sticky=E+W)

QUITbutton=Button(sysf,text="Quit",command=callback,padx=4,pady=4).grid(row=7, column=0,columnspan=2)

root.wm_protocol("WM_DELETE_WINDOW", callback)
root.after(1000,task)
root.mainloop()
DAQC2.CLOSE()

