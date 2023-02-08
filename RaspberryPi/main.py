## IMPORTS
import time
import cv2
import FRCCAN as can
from Logger import Logger
from ConePipeline import ConePipeline

## INIT
log = Logger("logs")
bus = can.initCAN(log)
device = can.Device(bus, 19, log)
ConeDetector = ConePipeline()

camera1 = cv2.VideoCapture(0)

mode = 1 # 1 = running, 0 = paused, -1 = shutdown

## RUN
while mode != -1:
    if mode == 0: # paused
        time.sleep(0.1)
        continue
    
    success, frame = camera1.read()

    if not success:
        log.warn("Could not get camera frame")
        continue

    cones = ConeDetector.process(frame)
    print(cones)
