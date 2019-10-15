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
import threading

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

##############################################################################################

sem = threading.Semaphore()

def HumanDetection():
	print("HumanDetection thread start")
	deg2rad = m.pi/180.0
	rad2deg = 180.0/m.pi
	camConstAng = 55.0*deg2rad
	
	global xDist_share, nboxes_share, closestIdx_share 
	while True:
		data, addr = data_socks[1].recvfrom(1024)
		try:
			parse_bbox_data(1, data, addr)
		except Exception as ee:
			print('failed to process bbox data #1:', ee)
		else:
			#print(rx_bbox_data[1]['bboxes'][0])
			#print("data",data)
			#print("rx_bbox_data[1]['nboxes']",rx_bbox_data[1]['nboxes'])
			closestBox = rx_bbox_data[1]['closestIdx']
			#print(closestBox)
			#print(type(closestBox))
			if closestBox is not None:
				pixel_x_min = rx_bbox_data[1]['bboxes'][closestBox][0]
				pixel_x_max = rx_bbox_data[1]['bboxes'][closestBox][1]
			else:
				pixel_x_min = rx_bbox_data[1]['bboxes'][0][0]
				pixel_x_max = rx_bbox_data[1]['bboxes'][0][1]
				
			pixel_x_center = (pixel_x_max + pixel_x_min) / 2.0
			pos = 1.0*(pixel_x_center - 424) / 424.0
			dist = rx_bbox_data[1]['distances'][0]
			frameDist = 2.0*dist/m.tan(camConstAng)
			xDist = map(pos, -1.0, 1.0, -frameDist/2.0, frameDist/2.0)
			
			sem.acquire()
			xDist_share = xDist
			nboxes_share = rx_bbox_data[1]['nboxes']
			closestIdx_share = rx_bbox_data[1]['closestIdx']
			sem.release()
			
			#print("xDist", xDist)
			
			
			
def Tracking():
	print("Tracking thread start")
	global xDist_share, nboxes_share, closestIdx_share

	xDist_share=0
	nboxes_share = 0
	
	Kp = 15.0
	Ki = 70.0
	Kd = 0.05 
	
	deg2rad = m.pi/180.0
	rad2deg = 180.0/m.pi
	pid = PID(Kp, Ki, Kd, setpoint=0.0)
	pid.sample_time = 0.01
	driveAngMin = -60.0
	driveAngMax = 60.0
	pid.output_limits = (driveAngMin, driveAngMax)
	prevDeg = GY.GetCurrentDeg(1)
	lastDriveAng = prevDeg
	waitTime = 0.0
	period = 0.0
	sign = 1
	tol = 1.0
	inc = 0.0

	while True:
		startTime = time.time()
		sem.acquire()
		xDist_PID = xDist_share
		nboxes = nboxes_share
		#closestIdx = closestIdx_share
		sem.release()
		print("nboxes:    ", nboxes)
		if nboxes is not 0:
			driveAng = pid(xDist_PID)
			#print("xDist_PID", xDist_PID)
			#print("driveAng",driveAng)
			if abs(xDist_PID) > 0.01:
				print("doing PID")
				GY.PositionControlMode2(1,-driveAng,120.0)
			waitTime = 0.0
		else:
			waitTime = waitTime + period
			if waitTime > 2.0:
				print("searching on human")
				
				driveAng = startSearchAng*m.sin(2*m.pi*(inc))
				GY.PositionControlMode2(1,driveAng,300.0)

				#print("waitTime", waitTime)
			else:
				
				print("wait for the time to search")
				CurrentDeg = GY.GetCurrentDeg(1)
				if CurrentDeg > 180.0:
					CurrentDeg = CurrentDeg - 360.0
				if CurrentDeg < 0.0:
					startSearchAng = driveAngMin
				else:
					startSearchAng = driveAngMax
				
				
		inc = inc + 0.002
				
	
		period = time.time() - startTime
			

def SearchingHuman():
	global searchingTrig
	global driveAngMin, driveAngMax
	searchingTrig = False
	print("SearchingHuman thread start")
	
	while searchingTrig:
		print("searching human...")
		GY.PositionControlMode2(1,driveAngMax,30.0)
		IsStop(1,driveAngMax)
		GY.PositionControlMode2(1,driveAngMin,30.0)
		IsStop(1,driveAngMin)

###############################################################################################

try:
	t1=threading.Thread(target = HumanDetection)
	t2=threading.Thread(target = Tracking)

	t1.start()
	t2.start()

	
except:
	print("Error: unable to start thread")
	
while 1:
	pass


'''
#GY.PositionControlMode1(1,0.0)
sp = 0.0  # at the center

# For speed 120.0
Kp = 10.0
Ki = 70.0
Kd = 0.05 

deg2rad = m.pi/180.0
rad2deg = 180.0/m.pi

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
		xDist = map(pos, -0.8, 0.8, -frameDist/2.0, frameDist/2.0)

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
		
		
		period = time.time() - startTime
		print("period", period)
		print("-------------------------------")
	
		xDist_list.append(xDist)
		driveAng_list.append(driveAng)
		plotTime.append(runTime + period)
		runTime = runTime + period
	


'''
