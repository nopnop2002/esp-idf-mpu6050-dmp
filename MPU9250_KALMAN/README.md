# MPU9250_KALMAN
This demo shows roll, pitch and yaw angles in 3D without using a DMP.   
Calculate roll, pitch and yaw using the very famous KalmanFilter.   
I used [this](https://github.com/TKJElectronics/KalmanFilter) library.   
I used [this](https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/tree/master/IMU/MPU6050) as a reference.   

# Configuration

![config-MPU6050_DMP6-1](https://user-images.githubusercontent.com/6020549/224453334-ad69a635-0767-4d94-8193-c11160b10eb7.jpg)
![config-MPU6050_DMP6-2](https://user-images.githubusercontent.com/6020549/224453337-8529aa7f-76dd-4b70-9bff-a43888973534.jpg)

# Drift comparison using MPU9250
Value after 10 minutes at rest.   
- DMP
```
I (600053) IMU: roll:-0.003362 pitch:0.174831 yaw:-2.203673
I (600153) IMU: roll:-0.003362 pitch:0.174831 yaw:-2.203673
```

- KALMAN Filter
```
I (1000105) MPU: roll:0.072056 pitch=-0.234558 yaw=-0.129212
I (1000225) MPU: roll:-0.014875 pitch=0.233770 yaw=0.131443
```

