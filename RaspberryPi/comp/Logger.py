import os
import time

class Logger:
	def __init__(self, logDir):
		if os.path.isdir(logDir):
			self.__dir = logDir
		else:
			print(f"Invalid logging path: {logDir}")
			self.__dir = None
		self.__msgs = []

	def warn(self, msg):
		msg = "[{}] WARNING: {}".format(time.strftime("%D, %T"), msg)
		self.__msgs.append(msg)
		print(msg)

	def warningCount(self):
		return len(self.__msgs)

	def save(self):
		with open("{}/log-{}.txt".format(self.__dir, time.strftime("%h-%d-%Y-%H-%M-%S")), "w+") as logFile:
			if len(self.__msgs) != 0:
				logFile.write("\n".join(self.__msgs))
			else:
				logFile.write("[No messages from this log]")

	def clear(self):
		self.__msgs = []

	def clearLogs(self):
		for d in os.listdir(self.__dir):
			if os.path.isfile(f"{self.__dir}/{d}"):
				try:
					os.remove(f"{self.__dir}/{d}")
				except Exception as e:
					self.warn(f"Couldn't remove log file {d}: {e}")
