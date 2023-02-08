## IMPORTS
import time
import cv2
import FRCCAN as can
from Logger import Logger
from CameraServer import CameraServer
from ConePipeline import ConePipeline

## INIT
log = Logger("logs")
bus = can.initCAN(log)
device = can.Device(bus, 19, log)

servers = [
	CameraServer("resize", 5000),
	CameraServer("hsl", 5001),
	CameraServer("erode", 5002),
	CameraServer("dilate", 5003)
]

ConeDetector = ConePipeline()

camera1 = cv2.VideoCapture(0)

mode = 1 # 1 = running, 0 = paused, -1 = shutdown

## RUN
servers[0].run()
servers[1].run()
servers[2].run()
servers[3].run()

while mode != -1:
    if mode == 0: # paused
        time.sleep(0.1)
        continue
    
    success, frame = camera1.read()
    #server1.frame = frame

    if not success:
        log.warn("Could not get camera frame")
        continue

    cones = ConeDetector.process(frame)

    servers[0].frame = ConeDetector.cv_resize_output
    servers[1].frame = ConeDetector.hsl_threshold_output
    servers[2].frame = ConeDetector.cv_erode_output
    servers[3].frame = ConeDetector.cv_dilate_output
    #print(cones)

