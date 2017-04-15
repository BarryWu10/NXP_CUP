import serial
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
ser = serial.Serial('COM5', 9600)

voltage = [0] * 128 

fig = plt.figure() 

# make plot
sub1 = fig.add_subplot(311) #camera ADC
sub2 = fig.add_subplot(312) #normalized
sub3 = fig.add_subplot(313) #derivative
line, = sub1.plot(voltage)
linee, = sub2.plot(voltage)
lineee, = sub3.plot(voltage)
sub1.set_ylim([0,66000])
sub2.set_ylim([0,2])
sub3.set_ylim([-8000,8000])

def norm(lst):
    newlst = []
    for point in lst:
        
        if point >= 30000:
            newlst.append(1)
        else: 
            newlst.append(0)

    return newlst

def derive(lst):
    newlst[0]*128
    for point in range(0, 128):
        if point == 0:
            newlst[point] = 0
        else:
            newlst[point] = lst[point]-lst[point-1]
    return newlst

def getSerial():
    data = ser.readline().split(b',')
    print(data)
    while (len(data) != 128):
        data = ser.readline().split(b',')	
    #print(cereal)
    return data

def update(bob):
    print("update")
    cereal = getSerial()
    #milk = norm(cereal)
    #bowl = derive(milk)
    line.set_ydata(cereal)	
    linee.set_ydata(norm(cereal))
    #lineee.set_ydata(milk)
	
ani = animation.FuncAnimation(fig, update)
plt.show()