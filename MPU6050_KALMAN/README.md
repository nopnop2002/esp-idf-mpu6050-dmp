# MPU6050_KALMAN
This demo shows roll and pitch angles in 3D without using a DMP.   
Calculate roll and pitch using the very famous KalmanFilter.   
Since the MPU6050 does not have a magnetic sensor, yaw cannot be calculated.   
No drift occurs at rest.   
I used [this](https://github.com/TKJElectronics/KalmanFilter) library.   
I used [this](https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/tree/master/IMU/MPU6050) as a reference.   

# Configuration

![config-MPU6050_DMP6-1](https://user-images.githubusercontent.com/6020549/224453334-ad69a635-0767-4d94-8193-c11160b10eb7.jpg)
![config-MPU6050_DMP6-2](https://user-images.githubusercontent.com/6020549/224453337-8529aa7f-76dd-4b70-9bff-a43888973534.jpg)

# Drift comparison using MPU6050
Value after 10 minutes at rest.   
- DMP
```
I (600053) IMU: roll:-0.003362 pitch:0.174831 yaw:-2.203673
I (600153) IMU: roll:-0.003362 pitch:0.174831 yaw:-2.203673
```

- KALMAN Filter
```
I (600117) MPU: roll:-0.028039 pitch=-0.056162
I (600237) MPU: roll:-0.028626 pitch=-0.027323
```
