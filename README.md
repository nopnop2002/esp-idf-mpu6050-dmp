# esp-idf-mpu6050-dmp
A demo showing the pose of the mpu6050 in 3D using esp-idf.   

MPU6050 has an internal processing function called DMP (Digital Motion Processor).   
You can use this to get Euler angles.   
Euler angles are roll, pitch and yaw.   
It's very intuitive and easy to understand.   
![a-Pitch-yaw-and-roll-angles-of-an-aircraft-with-body-orientation-O-u-v-original](https://user-images.githubusercontent.com/6020549/224452743-d4cf419d-f936-4e46-9ece-a12f21bf2e32.jpg)


# Installation overview

- Perform a calibration of your sensor.   

- Embed the calibration value of the sensor into the source.

- Get Euler angles from IMU.

- Display Euler angles in browser.

# Software requiment
ESP-IDF V4.4/V5.x.   
ESP-IDF V5.0 is required when using ESP32-C2.   
ESP-IDF V5.1 is required when using ESP32-C6.   

# Hardware requirements
MotionTracking device with DMP function.   

From the left MPU9250 MPU6500 MPU6050 GY-87   
![Hardware](https://user-images.githubusercontent.com/6020549/227761892-5c533f1e-2dc5-4164-8754-7bd6b0063336.JPG)

### MPU6000/6050/6500/6555/9150/9225/9250/9255   
MPU9150/9225/9250/9255 is a multi-chip module (MCM) consisting of two dies integrated into a single LGA/QFN package.   
One die houses the 3-Axis gyroscope and the 3-Axis accelerometer.   
The other die houses the AK8975 3-Axis magnetometer from Asahi Kasei Microdevices Corporation.   
Since the AK8963 is connected as a slave device, it is not recognized as an i2c device.   


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


### GY-86   
GY-86 consists of three chips, MPU6050, HMC5883L and MS5611.   
The HMC5883L is 3-Axis magnetometer from Honeywell.   
The MS5611 is Barometric Pressure Sensor from TE Connectivity.   
Since the HMC5883L is connected as a slave device of MPU6050, it is not recognized as an i2c device.   


### GY-87   
GY-87 consists of three chips, MPU6050, HMC5883L and BMP180.   
The HMC5883L is 3-Axis magnetometer from Honeywell.   
The BMP180 is Barometric Pressure Sensor from Bosch Sensortec.   
Since the HMC5883L is connected as a slave device of MPU6050, it is not recognized as an i2c device.   

### M5 Product   
Some M5 products have an IMU built in.   
M5Stack Gray/Fire/Faces/M5GO is equipped with MPU9250 or MPU6886.   
M5Core2 is equipped with MPU9250.   
M5StickC is equipped with SH200Q or MPU6886.   
M5StickC Plus/Plus2 is equipped with MPU6886.   
ATOM Matrix/ATOM S3 is equipped with MPU6886.   
__MPU6886 does not support DMP.__   
MPU6050_KALMAN/MPU6050_MADGWICK can be used in MPU6886.   

About M5Stick   
M5Stick has a regular version and an MPU9250 version.   
The regular version does not have an IMU.   
MPU9250 version supports DMP.   

You can check the built-in IMU using IMU_Detect.   

# Wireing
|MPU6xxx/9xxx||ESP32|ESP32-S2/S3|ESP32-C2/C3/C6||
|:-:|:-:|:-:|:-:|:-:|:-:|
|VCC|--|3.3V|3.3V|3.3V||
|GND|--|GND|GND|GND||
|SCL|--|GPIO22|GPIO12|GPIO5|(*1)|
|SDA|--|GPIO21|GPIO11|GPIO4|(*1)|

|GY-86/87||ESP32|ESP32-S2/S3|ESP32-C2/C3/C6||
|:-:|:-:|:-:|:-:|:-:|:-:|
|VCC_IN|--|N/C|N/C|N/C||
|3.3V|--|3.3V|3.3V|3.3V||
|GND|--|GND|GND|GND||
|SCL|--|GPIO22|GPIO12|GPIO5|(*1)|
|SDA|--|GPIO21|GPIO11|GPIO4|(*1)|

(*1)You can change it to any pin using menuconfig.   

# Perform a calibration of your sensor
I based on [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/IMU_Zero).
```
git clone https://github.com/nopnop2002/esp-idf-mpu6050-dmp
cd esp-idf-mpu6050-dmp/IMU_Zero
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6}
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


# Get Euler angles from IMU
I based on [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_DMP6).
```
cd esp-idf-mpu6050-dmp/MPU6050_DMP6
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

More details on configuration is [here](https://github.com/nopnop2002/esp-idf-mpu6050-dmp/tree/main/MPU6050_DMP6).

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
You can change the design and color according to your preference.   



# View Euler angles using PyTeapot   
You can view Euler angles using [this](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation) tool.   
It works as a UDP display server.   
This is a great application.   

```
+-------------+         +-------------+         +-------------+
|     IMU     |         |    ESP32    |         | pyteapot.py |
|             |--(ic2)->|             |--(UDP)->|             |
|             |         |             |         |             |
+-------------+         +-------------+         +-------------+
```

### Installation for Linux
```
$ python3 --version
Python 3.11.2
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

### Installation for Windows   
Install Git for Windows from [here](https://gitforwindows.org/).   
Install Python Releases for Windows from [here](https://www.python.org/downloads/windows/).   
Open Git Bash and run:   
```
$ python --version
Python 3.11.9
$ python -m pip install -U pip
$ python -m pip install pygame
$ python -m pip install PyOpenGL PyOpenGL_accelerate
$ git clone https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation
$ cd PyTeapot-Quaternion-Euler-cube-rotation
$ python pyteapot.py
```

The posture of your sensor is displayed.   
![PyTeapot-Windows](https://github.com/user-attachments/assets/2b0a1a70-40cb-47e5-8f51-eb4fe3adb1ab)


### String passed over Wifi to pyteapot.py
Yaw angle should be betweem two ```y```.   
Pitch angle should be between two ```p```.   
Roll angles should be between two ```r```.   
```
# yaw = 168.8099
# pitch = 12.7914
# roll = -11.8401
# Euler angles only
y168.8099yp12.7914pr-11.8401r

```

# Using IMU Filter
You can use Kalman and Madgwick filters instead of DMP.   
DMP estimates Euler angles in-device, whereas these filters estimate Euler angles purely in software.   

- MPU6050_KALMAN   
 Estimation using MPU6050 and KALMAN filter.
- MPU6050_MADGWICK   
 Estimation using MPU6050 and MADGWICK filter.
- MPU9250_KALMAN   
 Estimation using MPU9250 and KALMAN filter.
- MPU9250_MADGWICK   
 Estimation using MPU9250 and MADGWICK filter.
- MPU6050_HMC5883L_KALMAN   
 Estimation using MPU6050 + HMC5883L and KALMAN filter.
- MPU6050_HMC5883L_MADGWICK   
 Estimation using MPU6050 + HMC5883L and MADGWICK filter.
