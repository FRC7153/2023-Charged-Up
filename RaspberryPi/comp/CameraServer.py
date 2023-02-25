from flask import Flask, Response, Markup
import threading
import cv2

class CameraServer:
	def __init__(self, name, port):
		self.name = name
		self.__app = Flask(name)
		self.__thread = None
		self.__port = port

		self.frame = None # will be overwritten

	def __serveFrame(self):
		while True:
			if type(self.frame).__name__ == "NoneType":
				return Markup("<b>No frames yet</b> (refresh the page)")

			_, buffer = cv2.imencode(".jpg", self.frame)
			buffer = buffer.tobytes()
			yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + buffer + b'\r\n')

	def __videoEndpoint(self):
		return Response(self.__serveFrame(), mimetype="multipart/x-mixed-replace; boundary=frame")

	def run(self):
		self.__app.add_url_rule("/", None, self.__videoEndpoint)

		self.__thread = threading.Thread(
			target = lambda: self.__app.run(
				host="0.0.0.0",
				port=self.__port,
				debug=False,
				use_reloader=False
			)
		)
		self.__thread.daemon = True

		self.__thread.start()
