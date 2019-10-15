import matplotlib.pyplot as plt
import numpy as np
import math as m
import time
from GYEMSClass import *
GY = GYEMS()  # create an object
A = 60.0
driveAng = []
Tf = 100
inc = 0.0
while True:
	#driveAng.append(A*m.sin(2*m.pi*(inc)))   # For plot
	driveAng = A*m.sin(2*m.pi*(inc))
	GY.PositionControlMode2(1,driveAng,1000.0)
	inc = inc + 0.001
	#time.sleep(0.1)

'''	
plt.figure(1)
plt.plot(range(0,Tf),driveAng)
plt.show()
'''
