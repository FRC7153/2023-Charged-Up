## Imports
import RPi.GPIO as GPIO
import time
import threading

## Init
def initGPIO():
	GPIO.setmode(GPIO.BCM)

## Cleanup
def cleanup():
	GPIO.cleanup()

## Sensor
class UltrasonicSensor:
	## Init
	def __init__(self, triggerPort, echoPort):
		GPIO.setmode(GPIO.BCM)

		GPIO.setup(triggerPort, GPIO.OUT)
		GPIO.setup(echoPort, GPIO.IN)

		self.__ports = [triggerPort, echoPort]

		self.__dist = 0

		GPIO.output(self.__ports[0], GPIO.LOW)

		self.__terminate = False

		self.__thread = threading.Thread(target = self.__measure)
		self.__thread.daemon = True
		self.__thread.start()

	## Get Distance
	def getDist(self):
		return self.__dist

	## Cleanup
	def cleanup(self):
		self.__terminate = True

	## Measure Distance Thread
	def __measure(self):
		time.sleep(2)
		while not self.__terminate:
			# Send ping
			GPIO.output(self.__ports[0], True)
			time.sleep(0.00001)
			GPIO.output(self.__ports[0], False)

			start = time.time()
			stop = time.time()

			# Wait for response
			while GPIO.input(self.__ports[1]) == 0:
				start = time.time()

			while GPIO.input(self.__ports[1]) == 1:
				stop = time.time()

			self.__dist = (stop - start) * 171500 # speed of sound, mm/s divided by 2 (this is roundtrip time)

			# Wait before next loop
			time.sleep(0.05)

		GPIO.cleanup()

# nice