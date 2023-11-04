# HMC5883L_CALIBRATE
HMC5883L is connected as slave device of MPU6050.   
So scanning for i2c devices will not find HMC5883L.   
You can use this to get the compass offset value for each axis.   
I used [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino) I2Cdev library collection by Jeff Rowberg.   

# Software requiment   
ESP-IDF V4.4/V5.x.   
ESP-IDF V5.0 is required when using ESP32-C2.   
ESP-IDF V5.1 is required when using ESP32-C6.   


# Hardware requirements
GY86/GY87 9DoF MotionTracking device.   

# Wireing
|GY-86/87||ESP32|ESP32-S2/S3|ESP32-C2/C3/C6||
|:-:|:-:|:-:|:-:|:-:|:-:|
|VCC_IN|--|N/C|N/C|N/C||
|3.3V|--|3.3V|3.3V|3.3V||
|GND|--|GND|GND|GND||
|SCL|--|GPIO22|GPIO12|GPIO5|(*1)|
|SDA|--|GPIO21|GPIO11|GPIO4|(*1)|

(*1)You can change it to any pin using menuconfig.   

# Compass calibration
```
git clone https://github.com/nopnop2002/esp-idf-mpu6050-dmp
cd esp-idf-mpu6050-dmp/HMC5883L_CALIBRATE
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

### Configuration   
To find the offset value, set the compass offset to 0.   
![config-HMC5883L-1](https://github.com/nopnop2002/esp-idf-mpu6050-dmp/assets/6020549/da3af174-60c0-42e7-9704-66bf509f95eb)
![config-HMC5883L-2](https://github.com/nopnop2002/esp-idf-mpu6050-dmp/assets/6020549/4d5b1a4c-e36b-4591-a37d-86d5a55d90ca)

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

![hmc5883l-1](https://github.com/nopnop2002/esp-idf-mpu6050-dmp/assets/6020549/1182eb65-7e8f-44f7-92ec-cff05cf40a57)

### Execute calibration again   
If you set the offset you got from the calibration and run it again, the circle position will change.   

![hmc5883l-2](https://github.com/nopnop2002/esp-idf-mpu6050-dmp/assets/6020549/ab691224-b41b-4282-be30-233d554f1491)
