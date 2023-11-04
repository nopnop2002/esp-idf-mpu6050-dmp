# MPU9250_MADGWICK
This demo estimates roll, pitch and yaw angles in software without using a DMP.   
Estimate roll, pitch and yaw using the Madgwick Filter used in ROS.   
I used [this](https://github.com/arduino-libraries/MadgwickAHRS) Madgwick Filter.   
I used [this](https://github.com/arduino-libraries/MadgwickAHRS/blob/master/examples/Visualize101/Visualize101.ino) as a reference.   

# Change from original library
In the ESP-IDF environment, it is difficult to acquire data from sensors at regular intervals.   
Therefore, the time difference used for estimation was changed to a parameter.   

# Hardware requirements
MPU9150/9225/9250/9255PU9250 9DoF MotionTracking device.   
They have an internal processing function called DMP (Digital Motion Processor).   
But this sample doesn't use DMP, just 9DoF data.   

# Get compass offset
Use [this](https://github.com/nopnop2002/esp-idf-mpu6050-dmp/tree/main/AK8963_CALIBRATE) to find the compass offset.


# Get Euler angles from IMU
```
cd esp-idf-mpu6050-dmp/MPU9250_MADGWICK
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

# Configuration
Sets the compass offset obtained by calibration.   
![config-mpu9250-1](https://user-images.githubusercontent.com/6020549/227431656-0a076ccd-7cad-4fd0-a33f-1564cd773b98.jpg)
![config-mpu9250-2](https://user-images.githubusercontent.com/6020549/227431659-2d08a2f9-bfba-4491-a18f-e9fcdde918fe.jpg)

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

![browser-roll-pitch-yaw](https://user-images.githubusercontent.com/6020549/232365926-ccc6198b-42ec-44f7-891d-6caa93c3411c.JPG)

WEB pages are stored in the html folder.   
I used [this](https://canvas-gauges.com/) for gauge display.   
I used [this](https://threejs.org/) for 3D display.   
You can change it as you like.   

# View Euler angles using PyTeapot   
You can view Euler angles using [this](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation) tool.   
It works as a UDP display server.   
This is a great application.   

```
+-------------+     +-------------+     +-------------+
|     IMU     | i2c |    ESP32    | UDP | pyteapot.py |
|             |---->|             |---->|             |
|             |     |             |     |             |
+-------------+     +-------------+     +-------------+
```

### Installation
```
$ sudo apt install python3-pip python3-setuptools
$ python3 -m pip install -U pip
$ python3 -m pip install pygame
$ python3 -m pip install PyOpenGL PyOpenGL_accelerate
$ git clone https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation
$ cd PyTeapot-Quaternion-Euler-cube-rotation
$ python3 pyteapot.py
```

The posture of your sensor is displayed.   
![pyteapot_2023-03-11_09-11-46](https://user-images.githubusercontent.com/6020549/224452173-2350704d-1fc4-4a12-8324-434c11f62c52.png)

# Drift comparison using MPU9250
Value after 10 minutes at rest.   
- When using DMP   
```
I (600093) IMU: roll:-0.035195 pitch:0.062826 yaw:-0.405707
I (600193) IMU: roll:-0.035195 pitch:0.062826 yaw:-0.405707
```

- When using Madgwick Filter   
It takes at least ___60 seconds___ for the estimate of YAW to stabilize.   
Slight YAW drift will occur.   
```
I (600063) IMU: roll:0.036059 pitch=-0.122939 yaw=-0.337929
I (600203) IMU: roll:0.035848 pitch=-0.115790 yaw=-0.367256
```
