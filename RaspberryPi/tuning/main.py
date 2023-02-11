## IMPORTS
import cv2
from flask import Flask, render_template, Response, request

## INIT
app = Flask(__name__, template_folder=".")
camera = cv2.VideoCapture(1)

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
        yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route("/stream")
def stream():
    return Response(generateFrames(), mimetype = "multipart/x-mixed-replace; boundary=frame")

## SAVE
@app.route("/publish", methods=["POST"])
def save():
    try:
        content = {
            "venue": request.form.get("venue", "?"),
            "cone": {
                "h": [request.form.get("coneMinH", 0.0), request.form.get("coneMaxH", 180.0)],
                "s": [request.form.get("coneMinS", 0.0), request.form.get("coneMaxS", 255.0)],
                "l": [request.form.get("coneMinL", 0.0), request.form.get("coneMaxL", 255.0)]
            },
            "cube": {
                "h": [request.form.get("cubeMinH", 0.0), request.form.get("cubeMaxH", 180.0)],
                "s": [request.form.get("cubeMinS", 0.0), request.form.get("cubeMaxS", 255.0)],
                "l": [request.form.get("cubeMinL", 0.0), request.form.get("cubeMaxL", 255.0)]
            }
        }
        return str(content)
    except Exception as e:
        return f"Could not save: {e}"

## RUN
app.run(host="0.0.0.0", port=8080)