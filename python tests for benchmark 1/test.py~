import serial

comm = serial.Serial("/dev/ttyACM0", 115200, timeout=1)

while True:
	comm.write(input("> ") + "\n\r")

