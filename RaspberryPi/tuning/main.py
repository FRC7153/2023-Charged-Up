## IMPORTS
import cv2
from flask import Flask, render_template, Response, request

## INIT
app = Flask(__name__)
camera = cv2.VideoCapture(0)

## MAIN
@app.route("/")
def index():
    return render_template("tune.html")

## VIDEO STREAM
def generateFrames():
    while True:
        success, frame = camera.read()

        if not success:
            break

        _, frame = cv2.imencode(".jpg", frame)
        frame = frame.tobytes()
        yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + buffer + b'\r\n')

@app.route("/stream")
def stream():
    return Response(generateFrames(), mimetype = "multipart/x-mixed-replace; boundary=frame")

## SAVE
@app.route("/save")
def save():
    return "not saved"

## RUN
app.run(host="0.0.0.0", port=8080)