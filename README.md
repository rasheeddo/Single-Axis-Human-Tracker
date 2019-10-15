# Single-Axis-Human-Tracker

This is a single axis gimbal human tracker by using IntelRealsense and ATDServo. You have to run `./alertIfTooClose 127.0.0.1` on this [repo](https://github.com/mfassler/collision-detector.git). The detected human data with bounding-box will send to humanTracker.py and use that data to do a PID control for a gimbal.

This is the result https://www.youtube.com/watch?v=rGUT_X9O1jE
