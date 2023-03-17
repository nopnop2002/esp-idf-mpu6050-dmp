# MPU6050_MADGWICK
This demo estimates roll, pitch and yaw angles in software without using a DMP.   
Estimate roll, pitch and yaw using the Madgwick Filter used in ROS.   
The MPU6050 does not have a magnetic sensor, but it can estimate yaw.   
I based [this](https://github.com/arduino-libraries/MadgwickAHRS) library.   
I used [this](https://github.com/arduino-libraries/MadgwickAHRS/blob/master/examples/Visualize101/Visualize101.ino) as a reference.   

# Change from original library
The time difference used for estimation was changed to a parameter.   

# Hardware requirements
MPU6000/6050/6500/6555 6DoF MotionTracking device.   
They have an internal processing function called DMP (Digital Motion Processor).   
But this sample doesn't use DMP, just 6DoF data.   

# Configuration

![config-MPU6050_DMP6-1](https://user-images.githubusercontent.com/6020549/224453334-ad69a635-0767-4d94-8193-c11160b10eb7.jpg)
![config-MPU6050_DMP6-2](https://user-images.githubusercontent.com/6020549/224453337-8529aa7f-76dd-4b70-9bff-a43888973534.jpg)

# Display roll and pitch in 3D
Use [this](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation).   

# Drift comparison using MPU6050
Value after 10 minutes at rest.   
- When using DMP   
```
I (600093) IMU: roll:-0.035195 pitch:0.062826 yaw:-0.405707
I (600193) IMU: roll:-0.035195 pitch:0.062826 yaw:-0.405707
```

- When using Madgwick Filter   
The yaw estimate is useless.   
```
I (600057) MPU: roll:-0.139635 pitch=-0.201925 yaw=-157.518829
I (600177) MPU: roll:-0.194135 pitch=-0.134018 yaw=-157.696198
```
