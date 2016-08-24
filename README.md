# IMUTrack
This purpose of this project is to DEEPLY integrate IMU data into feature tracking. The type of tracker we are using to test our IMU integration is the classic KLT (Kanade-Lucas-Tomasi) tracker. The specific implementation we are using and the starting point for this project is from the following paper. 

Myung Hwangbo, Jun-Sik Kim, and Takeo Kanade, "Inertial-aided KLT Feature Tracking for a Moving Camera", IEEE/RSJ Int'l Conf. on Intelligent Robots and Systems (IROS'09), Oct 2009, pp. 1909-1916.

The original source code can be found at http://www.cs.cmu.edu/~myung/IMU_KLT/, however, there are many modifications from this in the current project. Most importantly, while they have implemented a way to either not use IMU integration or to use their method of integrating the IMU into the tracker, we have added even more types of IMU integration to select between in order to evaluate each method's performance. 

## 1) Install

### Dependencies
1) OpenCV
	http://opencv.org/

2) Intel IPP (Integrated Performance Primitives) Version 7.1
	https://registrationcenter.intel.com/en/forms/?licensetype=2&productid=2380

3) NVIDIA CUDA Toolkit
	https://developer.nvidia.com/cuda-toolkit

4) CMake

### Build

```
git clone https://github.com/jomnipotent17/IMUTrack
cd IMUTrack
mkdir build
cd build
cmake ../src/klt_tracker_v1.0/
make
```
Add a folder for the logfiles
```
mkdir LogFiles
```

### Download Datasets

Download and extract the preformatted data
```
cd ../src/klt_tracker_v1.0
wget -O data.zip https://www.dropbox.com/s/1hk46a3i51aayws/KLT_data.zip?dl=0
unzip data.zip
```


## 2) Run
1) Run

To run the program, head back to the build folder
```
cd ../../build
```
And select a sequence and model to run using the following format (there are other options as well but for simplicity we'll just show these)
```
./klt_tracker -f ConfigFile.cfg -seq (SEQUENCE#) -model (MODEL#)
```
Where the sequence and Config File must match in order for the log files to generate properly and also for the sensor extrinsics to be loaded properly for the correct sequence.



| ConfigFile.cfg      |     SEQUENCE# |
| -----------------  | --------------- |
| EuRoC_mh_01_c0.cfg |       1         |
| EuRoC_mh_02_c0.cfg |      2 |
| EuRoC_mh_03_c0.cfg |      3 |
| EuRoC_mh_04_c0.cfg |      4 |
| EuRoC_mh_05_c0.cfg |        5 |
| data_aerial_uav.cfg |      6 |
| data_desk_scene.cfg |      7 |
| EuRoC_v1_01_c0.cfg |       8 |
| EuRoC_v1_02_c0.cfg |       9 |
| EuRoC_v1_03_c0.cfg |       10 |
| EuRoC_v2_01_c0.cfg |       11 |
| EuRoC_v2_02_c0.cfg |       12 |
| EuRoC_v2_03_c0.cfg |       13 |


And the current model setup is as follows

| Integration Model  |     MODEL# |
| ----------------   | ---------- |
| noIMU |   1 |
| MyungIMU |  2 |
| velocitySwitch |  3 |
| axisAngleIntegration |   4 |


So, therefore to run the EuRoC machine hall #2 sequence with the velocity Switch model we would do the following
```
./klt_tracker -f ../src/klt_tracker_v1.0/configs/EuRoC_mh_02_c0.cfg -seq 2 -model 3
```



## 3) Automated Running 
I have made a script that runs the requested imu methods with whatever sequences are desired. The log files are then zipped up together and saved with their timestamp. I have also created a MATLAB script that parses these log files and plots the results...

## 4) Using a New Sequence
Because the source code expected the IMU and Image data in a specific format and named specific things I have included all of the data already. However, if you were to use an arbitrary data set there are certain things that you would have to change...

I've made a few scripts that rename files to what they are expected to be...

Currently there are 13 different sequences contained in the proper format for this code. They are:

EuRoC Mav Dataset (11 sequences)
	http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

Myung Dataset (2 sequences)
	http://www.cs.cmu.edu/~myung/IMU_KLT/



