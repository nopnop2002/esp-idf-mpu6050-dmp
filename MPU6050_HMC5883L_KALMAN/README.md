# MPU6050_HMC5883L_KALMAN
This demo estimates roll, pitch and yaw angles in software without using a DMP.   
Estimate roll, pitch and yaw using the very famous KalmanFilter.   
I used [this](https://github.com/TKJElectronics/KalmanFilter) Kalman Filter by TKJ Electronics.   
I used [this](https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/tree/master/IMU/MPU6050_HMC5883L) as a reference.   
I used [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino) I2Cdev library collection by Jeff Rowberg.   

# Hardware requirements
GY86/GY87 9DoF MotionTracking device.   
These are package that integrates the MPU6050 and a chip (HMC5883L) with a 3-axis magnetic sensor.   
They have an internal processing function called DMP (Digital Motion Processor).   
But this sample doesn't use DMP, just 9DoF data.   

# Wireing
|GY-86/87||ESP32|ESP32-S2/S3|ESP32-C2/C3/C6||
|:-:|:-:|:-:|:-:|:-:|:-:|
|VCC_IN|--|N/C|N/C|N/C||
|3.3V|--|3.3V|3.3V|3.3V||
|GND|--|GND|GND|GND||
|SCL|--|GPIO22|GPIO12|GPIO5|(*1)|
|SDA|--|GPIO21|GPIO11|GPIO4|(*1)|

(*1)You can change it to any pin using menuconfig.   

# Get compass offset
Use [this](https://github.com/nopnop2002/esp-idf-mpu6050-dmp/tree/main/HMC5883L_CALIBRATE) to find the compass offset.


# Get Euler angles from IMU
```
cd esp-idf-mpu6050-dmp/MPU6050_HMC5883L_KALMAN
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

# Configuration
Sets the compass offset obtained by calibration.   
![config-MPU6050-HMC5883L-1](https://github.com/nopnop2002/esp-idf-mpu6050-dmp/assets/6020549/6a58f047-52ff-406f-8f07-3c5be3384938)
![config-MPU6050-HMC5883L-2](https://github.com/nopnop2002/esp-idf-mpu6050-dmp/assets/6020549/f6909d38-0304-47b6-bf1f-048e69b4a81e)

# View Euler angles with built-in web server   
ESP32 acts as a web server.   
I used [this](https://github.com/Molorius/esp32-websocket) component.   
This component can communicate directly with the browser.   
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

- When using KALMAN Filter   
Slight YAW drift will occur.   
```
I (600025) IMU: roll:0.008345 pitch=-0.230621 yaw=1.071427
I (600145) IMU: roll:0.071426 pitch=0.014303 yaw=-0.710934
```

