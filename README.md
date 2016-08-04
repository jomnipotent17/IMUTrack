# IMUTrack
This purpose of this project is to DEEPLY integrate IMU data into feature tracking. The type of tracker we are using to test our IMU integration is the classic KLT (Kanade-Lucas-Tomasi) tracker. The specific implementation we are using and the starting point for this project is from the following paper. 

Myung Hwangbo, Jun-Sik Kim, and Takeo Kanade, "Inertial-aided KLT Feature Tracking for a Moving Camera", IEEE/RSJ Int'l Conf. on Intelligent Robots and Systems (IROS'09), Oct 2009, pp. 1909-1916.

The original source code can be found at http://www.cs.cmu.edu/~myung/IMU_KLT/, however, there are many modifications from this in the current project. Most importantly, while they have implemented a way to either not use IMU integration or to use their method of integrating the IMU into the tracker, we have added even more types of IMU integration to select between in order to evaluate each method's performance. 

## 1) Install

### Dependencies
1) OpenCV

2) Intel IPP (Integrated Performance Primitives)
	Version 7 or something, not the newest one. TODO: double check this one. 

3) NVIDIA CUDA Toolkit

4) CMake

### Build
1) Download the Source Code
`git clone https://github.com/jomnipotent17/IMUTrack`

2) Make a build folder

3) cmake 

4) make

## 2) Run
1) Run
`Code`
