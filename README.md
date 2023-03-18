# esp-idf-mpu6050-dmp
A demo showing the pose of the mpu6050 in 3D using esp-idf.

MPU6050 has an internal processing function called DMP (Digital Motion Processor).   
You can use this to get Euler angles.   
Euler angles are roll, pitch and yaw.   
![a-Pitch-yaw-and-roll-angles-of-an-aircraft-with-body-orientation-O-u-v-original](https://user-images.githubusercontent.com/6020549/224452743-d4cf419d-f936-4e46-9ece-a12f21bf2e32.jpg)

View roll, pitch and yaw with [this](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation) 3D viewer.   

# Installation overview

- Perform a calibration of your sensor.   

- Embed the calibration value of the sensor into the source.

- Install a 3D viewer.   

- Get Euler angles from MPU and display them in 3D.

# Software requiment
ESP-IDF V4.4/V5.0.   
ESP-IDF V5 is required when using ESP32-C2.   

# Hardware requirements
MotionTracking device with DMP function.   
MPU6000/6050/6500/6555/9150/9225/9250/9255 are abailable.   
MPU9150/9225/9250/9255PU9250 is a package that integrates a 3-axis magnetic sensor (AK8963).   
Since this sample uses DMP, it does not use a 3-axis magnetic sensor.   

|Device|Reg#117|sensors|Compass|
|:-:|:-:|:-:|:-:|
|MPU6000|0x68|Gyro/Accel||
|MPU6050|0x68|Gyro/Accel||
|MPU6500|0x70|Gyro/Accel||
|MPU6555|0x7C|Gyro/Accel||
|MPU9150|0x68|Gyro/Accel/Compass|AK8963|
|MPU9225|0x73|Gyro/Accel/Compass|AK8963|
|MPU9250|0x71|Gyro/Accel/Compass|AK8963|
|MPU9255|0x73|Gyro/Accel/Compass|AK8963|

# Wireing
|MPU6050||ESP32|ESP32-S2/S3|ESP32-C2/C3||
|:-:|:-:|:-:|:-:|:-:|:-:|
|SCL|--|GPIO22|GPIO12|GPIO5|(*1)|
|SDA|--|GPIO21|GPIO11|GPIO4|(*1)|
|GND|--|GND|GND|GND||
|VCC|--|3.3V|3.3V|3.3V||

(*1)You can change it to any pin using menuconfig.   

# Perform a calibration of your sensor
I based on [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/IMU_Zero).
```
git clone https://github.com/nopnop2002/esp-idf-mpu6050-dmp
cd esp-idf-mpu6050-dmp/IMU_Zero
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3}
idf.py menuconfig
idf.py flash
```

More details on configuration is [here](https://github.com/nopnop2002/esp-idf-mpu6050-dmp/tree/main/IMU_Zero).   

It will take a few minutes for the calibration to complete.   
The serial monitor should show results similar to the following.   
If -- done -- is displayed, it is completed.   

```
....................    XAccel                  YAccel                          ZAccel                  XGyro                   YGyro                   ZGyro
 [-2891,-2889] --> [-10,3]      [-445,-444] --> [-17,1] [697,698] --> [16381,16400]     [148,149] --> [-1,2]    [26,27] --> [-1,2]      [16,17] --> [-2,1]
.................... [-2890,-2889] --> [-5,3]   [-445,-444] --> [-17,1] [697,698] --> [16375,16400]     [148,149] --> [0,2]     [26,27] --> [0,2]       [16,17] --> [-2,1]
.................... [-2890,-2889] --> [-10,3]  [-445,-444] --> [-16,1] [697,698] --> [16373,16400]     [148,149] --> [0,2]     [26,27] --> [0,2]       [16,17] --> [-1,1]
-------------- done --------------
```

The last line is your offset value.   
We need 6 values XAccelOffset, YAccelOffset, ZAccelOffset, XGyroOffset, YGyroOffset, ZGyroOffset. In this example:   

XAccelOffset = -2889   
YAccelOffset = -444   
ZAccelOffset = 698   
XGyroOffset = 149   
YGyroOffset = 27   
ZGyroOffset = 17   


# Install a 3D viewer   
I used pyteapot.py.   
It works as a UDP display server.   
```
+-------------+     +-------------+     +-------------+
|     IMU     | i2c |    ESP32    | UDP | pyteapot.py |
|             |---->|             |---->|             |
|             |     |             |     |             |
+-------------+     +-------------+     +-------------+
```

This is a great application.   

```
$ sudo apt install python3-pip python3-setuptools
$ python3 -m pip install -U pip
$ python3 -m pip install pygame
$ python3 -m pip install PyOpenGL PyOpenGL_accelerate
$ git clone https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation
$ cd PyTeapot-Quaternion-Euler-cube-rotation
$ python3 pyteapot.py
```

If this screen is displayed, the installation was successful.
![pyteapot_2023-03-10_17-25-56](https://user-images.githubusercontent.com/6020549/224452171-3c65c911-b1c6-4e92-b1f8-ea4ee88ecbee.png)

# Embed the calibration value of the sensor into the source   
Change the offset values below:
```
cd esp-idf-mpu6050-dmp/MPU6050_DMP6/main
vi mpu6050.cpp
    // This need to be setup individually
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-2889);
    mpu.setYAccelOffset(-444);
    mpu.setZAccelOffset(698);
    mpu.setXGyroOffset(149);
    mpu.setYGyroOffset(27);
    mpu.setZGyroOffset(17);
```


# Get Euler angles from MPU and display them in 3D
I based on [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_DMP6).
```
cd esp-idf-mpu6050-dmp/MPU6050_DMP6
git clone https://github.com/Molorius/esp32-websocket components/websocket
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3}
idf.py menuconfig
idf.py flash
```

More details on configuration is [here](https://github.com/nopnop2002/esp-idf-mpu6050-dmp/tree/main/MPU6050_DMP6).


The posture of your sensor is displayed.   
![pyteapot_2023-03-11_09-11-46](https://user-images.githubusercontent.com/6020549/224452173-2350704d-1fc4-4a12-8324-434c11f62c52.png)

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

![browser](https://user-images.githubusercontent.com/6020549/226087870-f06e17b6-44f4-4889-a179-0bb6b2723386.JPG)

WEB pages are stored in the html folder.   
You can change it as you like.   

# Using IMU Filter
You can use Kalman and Madgwick filters instead of DMP.   
DMP estimates Euler angles in-device, whereas these filters estimate Euler angles purely in software.   
