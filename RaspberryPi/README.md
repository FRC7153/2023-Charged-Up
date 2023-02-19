# RaspberryPi
We used a RaspberryPi 4b on the arm of our robot to handle vision processing for the claw. It communicated with the RoboRio via CAN bus through a [RS485 CAN Hat](https://www.waveshare.com/rs485-can-hat.htm). It was not connected to the robot's radio.

## CAN API
#### Refresh (Class 1, Index 0)
Standard refresh packets sent from the RaspberryPi periodically, containing all the data the robot needs.

Data (8 bytes, 64 bits):
| 0 - 7 | 8 - 15 | 16 - 25 | 26 | 27 | 28 | 29 - 35 | 36 - 42 | 43 - 49 | 50 - 55 | 56 - 63 |
|---|---|---|---|---|---|---|---|---|---|---|
| X angle\*|Y angle\*|Distance (mm)| Target\*\*\*\* | Limit switch\*\* | Voltage Input Stable | CPU temp (C) | CPU % | Memory % | Camera FPS\*\*\* | CRC32 Checksum |

\*_In half degrees, 0 is center. This is a signed value, and negative 0 represents no target_
<br>
\*\*_Reserved for if this sensor(s) is added_
<br>
\*\*\*_As a decimal, 4 decimal bits before zero point_
<br>
\*\*\*\*_1 = cone, 0 = cube. If no target, is cached._

#### Control (Class 2)
Messages sent by the RoboRio to send commands to the RaspberryPi.

**Indexes:**
- 0: Run CameraServer (http)
- 1: Pause processing
- 2: Resume processing
- 3: Reboot Pi
- 4: Shutdown Pi

## Pipeline
The pipeline was created in Grip, then exported to Python and uploaded to the Pi with some modifications made. We had two pipelines, [cone.grip](RaspberryPi/GRIP/cone.grip) and [cube.grip](RaspberryPi/GRIP/cube.grip). The HSL values of these pipelines were changed later to add support for tuning. Now, they are loaded in from a JSON file.

## Tuning


## CAN 
