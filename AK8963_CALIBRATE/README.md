# AK8963_CALIBRATE
AK8963 is connected as slave device of MPU9XXX.   
So scanning for i2c devices will not find AK8963.   
You can use this to get the compass offset value for each axis.   
I used [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino) I2Cdev library collection by Jeff Rowberg.   

# Software requiment   
ESP-IDF V4.4/V5.x.   
ESP-IDF V5.0 is required when using ESP32-C2.   
ESP-IDF V5.1 is required when using ESP32-C6.   


# Hardware requirements
MPU9XXX 9DoF MotionTracking device.   

|Device|Reg#117|sensors|Compass|
|:-:|:-:|:-:|:-:|
|MPU9150|0x68|Gyro/Accel/Compass|AK8963|
|MPU9225|0x73|Gyro/Accel/Compass|AK8963|
|MPU9250|0x71|Gyro/Accel/Compass|AK8963|
|MPU9255|0x73|Gyro/Accel/Compass|AK8963|

# Wireing
|MPU9XXX||ESP32|ESP32-S2/S3|ESP32-C2/C3/C6||
|:-:|:-:|:-:|:-:|:-:|:-:|
|VCC|--|3.3V|3.3V|3.3V||
|GND|--|GND|GND|GND||
|SCL|--|GPIO22|GPIO12|GPIO5|(*1)|
|SDA|--|GPIO21|GPIO11|GPIO4|(*1)|

(*1)You can change it to any pin using menuconfig.   

# Compass calibration
```
git clone https://github.com/nopnop2002/esp-idf-mpu6050-dmp
cd esp-idf-mpu6050-dmp/AK8963_CALIBRATE
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

### Configuration   
To find the offset value, set the compass offset to 0.   
![config-AK8963-1](https://user-images.githubusercontent.com/6020549/227429885-7326b087-f37e-4f42-9f7b-0928e27e1b01.jpg)
![config-AK8963-2](https://user-images.githubusercontent.com/6020549/227429891-0a10160d-e845-4a79-b188-7e3ae59c279f.jpg)

### Execute calibration   
ESP32 acts as a web server.   
I used [this](https://github.com/Molorius/esp32-websocket) component.   
This component can communicate directly with the browser.   
Enter the following in the address bar of your web browser.   
```
http:://{IP of ESP32}/
or
http://esp32.local/
```

As you move the IMU it plots the X, Y and Z values.   
X, Y, Z offset are displayed.   

![ak8963-1](https://user-images.githubusercontent.com/6020549/227429940-de9ae64b-e340-4dde-aab5-0922136cc132.jpg)
![ak8963-2](https://user-images.githubusercontent.com/6020549/227429942-aca6f504-757c-489e-891d-c677c6434bbf.jpg)pg)

### Execute calibration again   
If you set the offset you got from the calibration and run it again, the circle position will change.   

![ak8963-11](https://user-images.githubusercontent.com/6020549/227429988-2e2da23e-8fbb-4217-9627-c5608f6a94a4.jpg)
![ak8963-12](https://user-images.githubusercontent.com/6020549/227429992-b4d6c1fa-c572-4495-bfc8-0170a26f8fbd.jpg)

