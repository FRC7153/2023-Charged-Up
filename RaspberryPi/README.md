# RaspberryPi
We used a RaspberryPi 4b on the arm of our robot to handle vision processing for the claw. It communicated with the RoboRio via CAN bus through a [RS485 CAN Hat](https://www.waveshare.com/rs485-can-hat.htm). It was not connected to the robot's radio.

## Pipeline
The pipeline was created in Grip, then exported to Python and uploaded to the Pi with some modifications made. We had two pipelines, [cone.grip](RaspberryPi/GRIP/cone.grip) and [cube.grip](RaspberryPi/GRIP/cube.grip). The HSL values of these pipelines were changed later to add support for tuning. Now, they are loaded in from a JSON file.

## Tuning


## CAN 