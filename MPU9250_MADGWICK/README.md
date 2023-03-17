# MPU9250_MADGWICK
This demo estimates roll, pitch and yaw angles in software without using a DMP.   
Estimate roll, pitch and yaw using the Madgwick Filter used in ROS.   
I based [this](https://github.com/arduino-libraries/MadgwickAHRS) library.
I used [this](https://github.com/arduino-libraries/MadgwickAHRS/blob/master/examples/Visualize101/Visualize101.ino) as a reference.   

# Change from original library
In the ESP-IDF environment, it is difficult to acquire data from sensors at regular intervals.   
Therefore, the time difference used for estimation was changed to a parameter.   

# Hardware requirements
MPU9150/9225/9250/9255PU9250 9DoF MotionTracking device.   
They have an internal processing function called DMP (Digital Motion Processor).   
But this sample doesn't use DMP, just 9DoF data.   

# Configuration

![config-MPU6050_DMP6-1](https://user-images.githubusercontent.com/6020549/224453334-ad69a635-0767-4d94-8193-c11160b10eb7.jpg)
![config-MPU6050_DMP6-2](https://user-images.githubusercontent.com/6020549/224453337-8529aa7f-76dd-4b70-9bff-a43888973534.jpg)

# Display roll and pitch in 3D
Use [this](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation).   

# Drift comparison using MPU9250
Value after 10 minutes at rest.   
- When using DMP   
```
I (600093) IMU: roll:-0.035195 pitch:0.062826 yaw:-0.405707
I (600193) IMU: roll:-0.035195 pitch:0.062826 yaw:-0.405707
```

- When using Madgwick Filter   
It takes at least 20 seconds for the estimate to stabilize.   
Slight YAW drift will occur.   
```
I (600063) MPU: roll:0.036059 pitch=-0.122939 yaw=-0.337929
I (600203) MPU: roll:0.035848 pitch=-0.115790 yaw=-0.367256
```
