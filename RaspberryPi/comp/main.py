#!/usr/bin/env python3

## IMPORTS
import time
import zlib
import cv2
import bitarray
import os

import FRCCAN as can
from Logger import Logger
from ConePipeline import ConePipeline
from CubePipeline import CubePipeline
from CameraServer import CameraServer
from SystemStats import Stats

## INIT
log = Logger("../logs")
bus = can.initCAN(log)
device = can.Device(bus, 19, log)
stat = Stats(log)

camera1 = cv2.VideoCapture(0)
server = CameraServer("output", 5000)

coneDetector = ConePipeline()
cubeDetector = CubePipeline()

## STATES
CAMERA_SERVER_RUNNING = False
MODE = 0 # 0 = running, 1 = pause, 2 = stop, 3 = shutdown, 4 = reboot
START = time.time()
LOOPS = 0

## CONTROL (restart, shutdown, stop, etc)
@device.receive(2)
def control(index, data):
	if index == 1: # pause
		MODE = 1
	elif index == 2: # resume
		START = time.time()
		LOOPS = 0
		MODE = 0
	elif index == 3: # reboot
		os.system("sudo reboot")
	elif index == 4: # shutdown
		os.system("sudo shutdown now")
	elif index == 5: # start camera
		server.run()
		CAMERA_SERVER_RUNNING = True
	else:
		log.warn("Unknown control message: " + index)

## CREATE PACKET (see docs)
def toBinary(value, size):
	bin = "{0:b}".format(value).zfill(size)

	if len(bin) > size:
		bin = bin[:size]

	return bin

def buildPacket(hasTarget, x, y, dist, t, ls, vi, temp, cpu, mem, fps):
	#print(f"Built packet w/ data: {hasTarget} x{x} y{y} d{dist} t{t} ls{ls} vi{vi} te{temp} cpu{cpu} mem{mem} fps{fps}")

	packet = bitarray.bitarray()

	if hasTarget:
		packet.append(x > 0)
		packet.extend(toBinary(abs(int(x)), 7))
		packet.append(y > 0)
		packet.extend(toBinary(abs(int(y)), 7))
	else:
		packet.extend([0] * 16)

	packet.extend(toBinary(int(dist), 10))
	packet.append(bool(t))
	packet.append(bool(ls))
	packet.append(bool(vi))
	packet.extend(toBinary(int(temp), 7))
	packet.extend(toBinary(int(cpu), 7))
	packet.extend(toBinary(int(mem), 7))
	packet.extend(toBinary(int(fps), 6))

	packet.extend(toBinary(zlib.crc32(packet.tobytes()), 8))

	# it is assumed, at this point, the length of packet is exactly 64 bits
	return packet

## RUN
device.startListening()

while True:
	# Check State
	if MODE != 0:
		continue

	# Start
	iterStart = time.time()

	# Get frame
	success, frame = camera1.read()

	if not success:
		log.warn("Could not fetch frame")
		continue

	# Detect objects
	cones = coneDetector.process(frame)
	cones = sorted(cones, key=lambda x: cv2.contourArea(x))

	cubes = cubeDetector.process(frame)
	cubes = sorted(cubes, key=lambda x: cv2.contourArea(x))

	# Determine primary target
	target = None
	targetType = True # True = cone, false = cube

	if len(cones) == 0 and len(cubes) == 0:
		target = None
	elif len(cones) == 0:
		targetType = True
		target = cv2.boundingRect(cubes[0])
	elif len(cubes) == 0:
		targetType = False
		target = cv2.boundingRect(cones[0])
	elif cv2.contourArea(cubes[0]) > cv2.contourArea(cones[0]):
		targetType = False
		target = cv2.boundingRect(cubes[0])
	else:
		targetType = True
		target = cv2.boundingRect(cones[0])

	# Send CAN packet
	device.send(1, 0, buildPacket(
		(target != None),
		0 if target == None else target[0] + (target[2]/2),
		0 if target == None else target[1] + (target[3]/2),
		10,
		targetType,
		False,
		stat.voltage,
		stat.temp,
		stat.cpu,
		stat.mem,
		int(LOOPS / (time.time() - START))
	))

	LOOPS+=1

	if len(cones) != 0:
		cv2.drawContours(coneDetector.cv_resize_output, cones, 0, (255, 0, 0), 3)

	if CAMERA_SERVER_RUNNING:
		server.frame = coneDetector.cv_resize_output

	# Don't run faster than once every 20 ms
	time.sleep(max(0.02 - (time.time() - iterStart), 0))
