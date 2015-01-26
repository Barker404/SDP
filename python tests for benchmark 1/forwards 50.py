import serial

comm = serial.Serial("/dev/ttyACM0", 115200, timeout=1)

comm.write("FORWARDS_50\n\r")

