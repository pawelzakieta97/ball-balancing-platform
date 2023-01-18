# Ball balancing platform

![image](https://user-images.githubusercontent.com/28107745/213289347-ad10fcf1-6afd-453d-8df2-7952b65f7351.png)

This repository contains a source code and report of a robotics project.
The aim of the project was to design, construct and control a platform that could balance a ball, as well as make it follow a given trajectory.
Constructing a real-world unstable system allowed for a comparison between PID and model predictive control system.
Footage comapring the platform operating with different control systems is uploaded to `report` directory 

Video below showcases following a circular trajectory with a radius of 7cm using a model predictie control:

https://user-images.githubusercontent.com/28107745/213289066-59c10ac9-6908-4360-b5ae-b6560e022d09.mp4


## Feedback loop
Data fed to the control system was heavily preprocessed according to the feedback loop.
High-framerate footage from raspberry camera had to be downscaled, converted to HSV color space and analyzed to find the center of and orange ball.
Detected pixel coordinates are converted into real-world coordinates.
High frequency noise is filtered out by approximating some number of past samples with a polynomial.
It is assumed, that the control systems output the desired acceleration of the ball.
The acceleration is then converted into desired platform roll and pitch, and the correct servo angles are derived using inversed kinematics.

![image](https://user-images.githubusercontent.com/28107745/213280525-8ed93693-fb0d-4d05-9288-54d33948a5b4.png)
