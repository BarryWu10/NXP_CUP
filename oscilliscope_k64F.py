import serial
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
ser = serial.Serial('COM5', 9600)

voltage = [0] * 128 

fig = plt.figure() 

# make plot
sub = fig.add_subplot(111)
line, = sub.plot(voltage)
sub.set_ylim([0,66000])

def getSerial():
    data = ser.readline().split(b',')
    while (len(data) != 128):
        data = ser.readline().split(b',')	
    return data

def update(bob):
    cereal = getSerial()
    line.set_ydata(cereal)	
	
ani = animation.FuncAnimation(fig, update)
plt.show()