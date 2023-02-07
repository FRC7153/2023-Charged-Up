## IMPORTS
import FRCCAN as can
from Logger import Logger

## INIT
log = Logger("logs")
bus = can.initCAN(log)
device = can.Device(bus, 19, log)


