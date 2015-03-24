"""
Comms testing: How many commands are we dropping?
"""
import serial
import time
from cv2 import waitKey
import sys

def main():
	
	arduino = Arduino('/dev/ttyACM0', 115200, 1, 1)

	n = 0
	c = True
	while(c != 27):

		arduino.write("Don't lose me!")
		n+=1

		if (n % 20) == 0:
			read = arduino.serial.readline()
			print "Robo has", read, "commands"
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
            self.increment_command+=1;
            if self.last_command != string or self.increment_command > 5:
                self.increment_command=0;
                print string
                self.last_command = string
                self.serial.write(string)

if __name__ == '__main__':
			main()