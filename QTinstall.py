import os
import time

print ("To run this program you need to install the QT4 GUI routines.")
print ("Do so by executing the following from the command line:")
print ("'sudo apt-get install python3-pyqt4'")
response=raw_input("Would you like to install them now? (y/n)")
if (response.lower()=='y'):
    os.system("sudo apt-get install python3-pyqt4")
    print('QT4 GUI routines installed. Please restart the program')
print('This window will close in 5 seconds...')
time.sleep(5)
