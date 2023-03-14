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
I (600025) MPU: roll:0.008345 pitch=-0.230621 yaw=1.071427
I (600145) MPU: roll:0.071426 pitch=0.014303 yaw=-0.710934
```

