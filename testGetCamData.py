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
	
deg2rad = m.pi/180.0
rad2deg = 180.0/m.pi
camConstAng = 55.0*deg2rad

while True:
	startTime = time.time()
	data, addr = data_socks[1].recvfrom(1024)
	try:
		parse_bbox_data(1, data, addr)
	except Exception as ee:
		print('failed to process bbox data #1:', ee)
	else:
		#print(rx_bbox_data[1]['bboxes'][0])
		box = rx_bbox_data[1]['nboxes']
		if box > 0:
			print("closestIdx: ",rx_bbox_data[1]['closestIdx'])
			
		#print("rx_bbox_data[1]['nboxes']",rx_bbox_data[1]['nboxes'])
		pixel_x_min = rx_bbox_data[1]['bboxes'][0][0]
		pixel_x_max = rx_bbox_data[1]['bboxes'][0][1]
		pixel_x_center = (pixel_x_max + pixel_x_min) / 2.0
		pos = 1.0*(pixel_x_center - 424) / 424.0
		dist = rx_bbox_data[1]['distances'][0]
		frameDist = 2.0*dist/m.tan(camConstAng)
		xDist = map(pos, -1.0, 1.0, -frameDist/2.0, frameDist/2.0)

		period = time.time() - startTime
		#print("period", period)
		#print("-------------------------------")
	


	
	
