# MPU9250_KALMAN
This demo shows roll, pitch and yaw angles in 3D without using a DMP.   
Calculate roll, pitch and yaw using the very famous KalmanFilter.   
I used [this](https://github.com/TKJElectronics/KalmanFilter) library.   
I used [this](https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/tree/master/IMU/MPU6050_HMC5883L) as a reference.   

# Hardware requirements
MPU9150/9150/9225/9250/9255 9DoF MotionTracking device.   
These are package that integrates the MPU6515 and a chip (AK8963) with a 3-axis magnetic sensor.   
They have an internal processing function called DMP (Digital Motion Processor).   
But this sample doesn't use DMP, just 9DoF data.   

# Configuration

![config-MPU6050_DMP6-1](https://user-images.githubusercontent.com/6020549/224453334-ad69a635-0767-4d94-8193-c11160b10eb7.jpg)
![config-MPU6050_DMP6-2](https://user-images.githubusercontent.com/6020549/224453337-8529aa7f-76dd-4b70-9bff-a43888973534.jpg)

# Drift comparison using MPU9250
Value after 10 minutes at rest.   
- DMP
```
I (600093) IMU: roll:-0.035195 pitch:0.062826 yaw:-0.405707
I (600193) IMU: roll:-0.035195 pitch:0.062826 yaw:-0.405707
```

- KALMAN Filter
```
I (600025) MPU: roll:0.008345 pitch=-0.230621 yaw=1.071427
I (600145) MPU: roll:0.071426 pitch=0.014303 yaw=-0.710934
```

