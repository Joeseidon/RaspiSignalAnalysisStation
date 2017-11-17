
from Tkinter import *        # The Tk package
import Pmw                   # The Python MegaWidget package
import math                  # import the sin-function
from tkFileDialog import *
from tkMessageBox import *
import sys

# define options for opening an existing log file
xlogfile_opt = options = {}
options['defaultextension'] = '.log'
options['filetypes'] = [('log files', '.log')]
options['title'] = 'Open existing log file'

def OpenLogFile():
    global vFile
    #xfName=''
    #xfName=askopenfilename(**xlogfile_opt)
    #if ('.log' in xfName):
        #vFile=open(xfName,'r') 
    vFile=open(sys.argv[1],'r')
        
if (len(sys.argv) != 2):
    print "Wrong argument count!"
    quit()

if ('.log' not in sys.argv[1]):
    print "Wrong file type!"
    quit() 
        
viewroot = Tk()              # build Tk-environment
viewroot.resizable(0,0)
W=800
H=600

w = viewroot.winfo_screenwidth()
h = viewroot.winfo_screenheight()
x = w/2 - W/2
y = h/2 - H/2
viewroot.geometry("%dx%d+%d+%d" % (W,H,x, y))
viewroot.wm_title("DAQCplate Data Log Viewer")

vFile=''
OpenLogFile()
        
#f=open('TestLog.log','r')
localFile=''
end=False
lineCount=0
record=[]
while (end==False):
    line=vFile.readline()
    if (line!=''):
        record.append([])
        record[lineCount] = line.split(',')
        localFile=localFile+line
        lineCount+=1
    else:
        end=True
        vFile.close()

channelCount=len(record[0][:])-1

xvals=[]
  
sampleCount=lineCount-1  
#remove new line characters
for i in range(lineCount):
    slen=len(record[i][channelCount])
    record[i][channelCount]=record[i][channelCount][0:slen-1] 

#put time stamp in xvals    
for i in range(sampleCount):
    xvals.append(record[i+1][0])

#Extract Data Headers
headers=[]
headers=record[0][1:channelCount+1]

#Create data channel lists
channelData=[]
for i in range(channelCount):
    channelData.append([])
    for j in range(sampleCount):
        channelData[i].append(record[j+1][i+1])

#Determine data type        
cType=range(channelCount)
aCount=0
dCount=0
for i in range(channelCount):
    val=channelData[i][0]
    if ('.' in val):
        cType[i]='a'
        aCount+=1
    else:
        cType[i]='d'
        dCount+=1
        for j in range(sampleCount):
            val=float(channelData[i][j]) + dCount*1.1
            channelData[i][j]=str(val)
            
#Start Plotting        
color = ['#ff0000','#ffff00','#00ff00','#00ffff','#0000ff','#ff00ff']
        
if (aCount>0):        
    ncurves = aCount           
    npoints = sampleCount                  

    smoothing='linear'
    symbols  = 0

    # Using Pmw.Blt.Vectors is often slower, but in this case very convenient.
    vector_x = Pmw.Blt.Vector()   
    vector_y = []
    vector_x=range(sampleCount)

    for y in range(ncurves):
        vector_y.append(Pmw.Blt.Vector())      # make vector for y-axis
    
    aList=[]
    for c in range(ncurves):                   # for each curve...
        for i in range(channelCount):
            if (cType[i]=='a'):
                aList.append(i)
                vector_y[c]=channelData[c][:]  # create y-values

    if (dCount>0):
         g = Pmw.Blt.Graph(viewroot,
            width = 800,
            height = '3.1i',)                     # make a new graph area
    else:
         g = Pmw.Blt.Graph(viewroot,
            width = 800,
            height = '6.2i',)                     # make a new graph area

    g.grid_toggle()        
    g.grid(row=0,column=0)

    for c in range(ncurves):                      # for each curve...
       curvename = headers[aList[c]]              # make a curvename
       g.line_create(curvename,                   # and create the graph
                     xdata=vector_x,              # with x data,
                     ydata=vector_y[c],           # and  y data
                     color=color[c%6],            # and a color
                     dashes=0,                    # and no dashed line
                     linewidth=2,                 # and 2 pixels wide
                     symbol='')                   # ...and no disks
       
    g.configure(title='Analog Channels')          # enter a title

if (dCount>0):  
    ncurves = dCount           
    npoints = sampleCount                  

    symbols  = 0

    # Using Pmw.Blt.Vectors is often slower, but in this case very convenient.
    dvector_x = Pmw.Blt.Vector()   
    dvector_y = []
    dvector_x=range(sampleCount)

    for y in range(ncurves):
        dvector_y.append(Pmw.Blt.Vector())          # make vector for y-axis
    
    dList=[]
    
    c=0
    for i in range(channelCount):
        if (cType[i]=='d'):    
            dList.append(i)
            dvector_y[c]=channelData[i][:]  # create y-values
            c+=1
    if (aCount>0):
        g2 = Pmw.Blt.Graph(viewroot,
                width = 800,
                height = '3.1i',)
    else:
        g2 = Pmw.Blt.Graph(viewroot,
                width = 800,
                height = '6.2i',)
        
    # make a new graph area
    g2.grid_toggle() 
    g2.configure(title='Digital Channels')          # enter a title       
    g2.grid(row=1,column=0)
    smoothing='step'    
    for c in range(ncurves):                       # for each curve...
       curvename2 = headers[dList[c]]              # make a curvename
       g2.line_create(curvename2,                  # and create the graph
                     xdata=dvector_x,              # with x data,
                     ydata=dvector_y[c],           # and  y data
                     color=color[c%6],             # and a color
                     dashes=0,                     # and no dashed line
                     linewidth=2,                  # and 2 pixels wide
                     symbol='',
                     smooth='step',)               # ...and no disks

viewroot.mainloop()                                


