#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import os
import time
import select
import socket
import struct
import numpy as np
import math as m
from GYEMSClass import *
from simple_pid import PID
import matplotlib.pyplot as plt

GY = GYEMS()  # create an object

DATA_PORT = 3101
data_socks = [
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM),
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM),
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM),
]

# XDrive has one camera
#data_socks[0].bind(("127.0.0.1", DATA_PORT))
data_socks[1].bind(("127.0.0.1", DATA_PORT))
#data_socks[2].bind(("127.0.0.1", DATA_PORT + 2))

############
# These are the bounding boxes (and distances) from "collision-detector"
###
rx_bbox_data = [
    {
        'remote_id': 0,
        'nboxes': 0,
        'closestIdx': None,
        'distances': np.zeros(16, np.float),
        'bboxes': np.zeros((16, 4), np.int32),
        'ts': time.time()
    },
    {
        'remote_id': 0,
        'nboxes': 0,
        'closestIdx': None,
        'distances': np.zeros(16, np.float),
        'bboxes': np.zeros((16, 4), np.int32),
        'ts': time.time()
    },
    {
        'remote_id': 0,
        'nboxes': 0,
        'closestIdx': None,
        'distances': np.zeros(16, np.float),
        'bboxes': np.zeros((16, 4), np.int32),
        'ts': time.time()
    },
]

## find the closest camera and distance
def parse_bbox_data(camNum, data, addr):
    rx_bbox_data[camNum]['remote_id'] = data[2]
    rx_bbox_data[camNum]['nboxes'] = data[3]
    rx_bbox_data[camNum]['ts'] = time.time()
    nboxes = min(16, data[3])
    for i in range(nboxes):
        iStart = i*20 + 4
        iStop = iStart + 20
        #distances[i], bboxes[i][0], bboxes[i][1], bboxes[i][2], bboxes[i][3] = \
        a, b, c, d, e = struct.unpack('fiiii', data[iStart:iStop])
        rx_bbox_data[camNum]['distances'][i] = a
        rx_bbox_data[camNum]['bboxes'][i][0] = b
        rx_bbox_data[camNum]['bboxes'][i][1] = c
        rx_bbox_data[camNum]['bboxes'][i][2] = d
        rx_bbox_data[camNum]['bboxes'][i][3] = e
    if nboxes:
        closestIdx = np.argmin(rx_bbox_data[camNum]['distances'][:nboxes])
        rx_bbox_data[camNum]['closestIdx'] = closestIdx
    else:
        rx_bbox_data[camNum]['closestIdx'] = None

def map(val, in_min, in_max, out_min, out_max):
	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
	
def IsStop(ID,driveAng):
	tol = 1
	if driveAng < 0:
		driveAng = 360 + driveAng
	while (abs(GY.GetCurrentDeg(ID) - driveAng) > tol):
		#print("diff", abs(GY.GetCurrentDeg(ID) - driveAng))
		pass

#GY.PositionControlMode1(1,0.0)
sp = 0.0  # at the center

# For speed 120.0
Kp = 10.0
Ki = 70.0
Kd = 0.05 

deg2rad = m.pi/180.0
rad2deg = 180.0/m.pi
camConstAng = 55.0*deg2rad
period = 0.1
pre_error = 0.0

pid = PID(Kp, Ki, Kd, setpoint=sp)
pid.sample_time = 0.01
driveAngMin = -60
driveAngMax = 60
pid.output_limits = (driveAngMin, driveAngMax)
#pid.proportional_on_measurement = True
i = 0
xDist_list = []
driveAng_list = []
plotTime = []
runTime = 0
lastDriveAng = 0
preAng = GY.GetCurrentDeg(1)
lastDriveAng = preAng
trig = True
waitTime = 0.0
#for i in range(0,300):
while True:
	startTime = time.time()
	data, addr = data_socks[1].recvfrom(1024)
	try:
		parse_bbox_data(1, data, addr)
	except Exception as ee:
		print('failed to process bbox data #1:', ee)
	else:
		#print(rx_bbox_data[1]['bboxes'][0])
		#print("data",data)
		#print("rx_bbox_data[1]['nboxes']",rx_bbox_data[1]['nboxes'])
		pixel_x_min = rx_bbox_data[1]['bboxes'][0][0]
		pixel_x_max = rx_bbox_data[1]['bboxes'][0][1]
		pixel_x_center = (pixel_x_max + pixel_x_min) / 2.0
		pos = 1.0*(pixel_x_center - 424) / 424.0
		dist = rx_bbox_data[1]['distances'][0]
		frameDist = 2.0*dist/m.tan(camConstAng)
		xDist = map(pos, -1.0, 1.0, -frameDist/2.0, frameDist/2.0)

		#driveAng = m.atan(object_from_center/dist)
		#driveAng = driveAng*rad2deg
		CurrentDeg = GY.GetCurrentDeg(1)
		print('CurrentDeg',CurrentDeg)
		if rx_bbox_data[1]['nboxes'] is 0:
		
			if waitTime > 1.0:
				print("Searching for human....")
				if trig is True:
					print("drive +")
					driveAng = driveAngMax
					GY.PositionControlMode2(1,driveAng,30.0)
					IsStop(1,driveAng)
				else:
					print("drive -")
					driveAng = driveAngMin
					GY.PositionControlMode2(1,driveAng,30.0)
					IsStop(1,driveAng)	
				
				
				waitTime = 0.0	# clear waitTime
				
			else:
				print("No human...wait for searching...")
				driveAng = CurrentDeg
				#GY.PositionControlMode2(1,driveAng,90.0)
			print("waitTime",waitTime)
			waitTime = waitTime + period
				
		else:
			print("PID controlling...")
			driveAng = pid(xDist)
			if abs(xDist) > 0.01:
				GY.PositionControlMode2(1,-driveAng,120.0)
			
		trig = not trig
		lastDriveAng = driveAng
			
		#print("pixel_x_center", pixel_x_center)
		#print("frameDist", frameDist)
		print("xDist", xDist)
		#print("pos",pos)
		#print("dist", dist)
		print("driveAng [deg]",driveAng)
		
		'''
		error = setpoint - object_from_center
		outputKp = error*Kp
		outputKd = ((error - pre_error)/period)*Kd
		
		print("outputKp", outputKp)
		print("outputKd", outputKd)
		driveAng = -outputKp + outputKd
		GY.PositionControlMode2(1,driveAng,720.0)
		
		pre_error = error
		'''
		

		
		period = time.time() - startTime
		print("period", period)
		print("-------------------------------")
	
		xDist_list.append(xDist)
		driveAng_list.append(driveAng)
		plotTime.append(runTime + period)
		runTime = runTime + period
	
'''
plt.figure(1)
plt.subplot(2,1,1)
plt.plot(plotTime,xDist_list)	
plt.grid()
plt.subplot(2,1,2)
plt.plot(plotTime,driveAng_list)
plt.grid()
		
plt.show()
'''
		
