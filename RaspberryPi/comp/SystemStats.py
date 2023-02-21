## IMPORTS
import subprocess as sp
import psutil
import threading
import time

## STATS
class Stats:
	## INIT
	def __init__(self, log):
		self.voltage = True
		self.temp = 0
		self.cpu = 0
		self.mem = 0

		self.__log = log

		# Start thread
		self.__thread = threading.Thread(target=self.__refreshThread)
		self.__thread.daemon = True
		self.__thread.start()

	## REFRESH ALL STATS PERIODICALLY
	def __refreshThread(self):
		while True:
			try:
				# Voltage
				rawVoltage = sp.run(["vcgencmd", "get_throttled"], stdout=sp.PIPE)
				rawVoltage = rawVoltage.stdout.decode().split("=")[1].replace("\n", "")
				self.voltage = not (int(rawVoltage, 0) == 0x01)

				# Temp
				with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
					self.temp = int(int(f.read()) / 1000)

				# CPU
				self.cpu = int(psutil.cpu_percent())

				# Memory Allocation
				self.mem = int(psutil.virtual_memory().percent)
			except Exception as e:
				self.__log.warn(f"Could not fetch system stats: {e}")

			time.sleep(0.2)
