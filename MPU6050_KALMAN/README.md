# MPU6050_KALMAN
This demo estimates roll and pitch angles in software without using a DMP.   
Estimate roll and pitch using the very famous Kalman Filter.   
Since the MPU6050 does not have a magnetic sensor, yaw cannot be estimated.   
I used [this](https://github.com/TKJElectronics/KalmanFilter) library.   
I used [this](https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/tree/master/IMU/MPU6050) as a reference.   

# Hardware requirements
MPU6000/6050/6500/6555 6DoF MotionTracking device.   
They have an internal processing function called DMP (Digital Motion Processor).   
But this sample doesn't use DMP, just 6DoF data.   

# Get Euler angles from MPU and display them in 3D
```
cd esp-idf-mpu6050-dmp/MPU9250_KALMAN
git clone https://github.com/Molorius/esp32-websocket components/websocket
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3}
idf.py menuconfig
idf.py flash
```

# Configuration

![config-MPU6050_DMP6-1](https://user-images.githubusercontent.com/6020549/224453334-ad69a635-0767-4d94-8193-c11160b10eb7.jpg)
![config-MPU6050_DMP6-2](https://user-images.githubusercontent.com/6020549/224453337-8529aa7f-76dd-4b70-9bff-a43888973534.jpg)

# View Euler angles in 3D
Use [this](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation).   

# View Euler angles with built-in web server   
ESP32 acts as a web server.   
I used [this](https://github.com/Molorius/esp32-websocket) component.   
This component can communicate directly with the browser.   
It's a great job.   
Enter the following in the address bar of your web browser.   
```
http:://{IP of ESP32}/
or
http://esp32.local/
```

![browser-roll-pitch](https://user-images.githubusercontent.com/6020549/226090764-983cf7f6-6bbf-45cd-9065-8cb053a00138.JPG)

# Drift comparison using MPU6050
Value after 10 minutes at rest.   
- When using DMP
```
I (600093) IMU: roll:-0.035195 pitch:0.062826 yaw:-0.405707
I (600193) IMU: roll:-0.035195 pitch:0.062826 yaw:-0.405707
```

- When using Kalman Filter
```
I (600117) MPU: roll:-0.028039 pitch=-0.056162
I (600237) MPU: roll:-0.028626 pitch=-0.027323
```
