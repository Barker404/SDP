"""
Comms testing: How many commands are we dropping?
"""
import serial
import time
from cv2 import waitKey
import sys

MSG_GAP = 0.1

def main():
    
    arduino = Arduino('/dev/ttyACM0', 115200, 0.01, 1)

    n = 0
    c = True
    last_time = time.clock()
    while(c != 27):
        now = time.clock()
        time.sleep(MSG_GAP - (now-last_time))
        last_time = time.clock()
        arduino.write("MSGLOL\n\r")
        n+=1

        lines = arduino.serial.readlines()
        print "Robo has", lines, "commands"
        print "We sent", n, "commands"

        c = waitKey(2) & 0xFF

class Arduino:

    def __init__(self, port, rate, timeOut, comms):
        self.serial = None
        self.comms = comms
        self.port = port
        self.rate = rate
        self.timeout = timeOut
        self.last_command = ""
        self.setComms(comms)
        self.increment_command=0

    def setComms(self, comms):
        if comms > 0:
            self.comms = 1
            if self.serial is None:
                try:
                    self.serial = serial.Serial(self.port, self.rate, timeout=self.timeout)
                except:
                    print self.port
                    print self.rate
                    print self.timeout
                    print "No Arduino detected!"
                    sys.exit(0)
                    print "Continuing without comms."
                    self.comms = 0
        else:
            self.write('\rRUN_ENG %d %d\r' % (0, 0))
            self.comms = 0

    def write(self, string):
        if self.comms == 1:
            self.serial.write(string)

if __name__ == '__main__':
            main()