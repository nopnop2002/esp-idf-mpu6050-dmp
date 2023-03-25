# HMC5883L_CALIBRATE
HMC5883L is connected as slave device of MPU9XXX.   
You can use this to get the compass offset value for each axis.   

# Software requiment   
ESP-IDF V4.4/V5.0.   
ESP-IDF V5 is required when using ESP32-C2.   


# Hardware requirements
GY-86/87 9DoF MotionTracking device.   

# Wireing
|GY86/87||ESP32|ESP32-S2/S3|ESP32-C2/C3||
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
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3}
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
It's a great job.   
Enter the following in the address bar of your web browser.   
```
http:://{IP of ESP32}/
or
http://esp32.local/
```

As you move the IMU it plots the X, Y and Z values.   
X, Y, Z offset are displayed.   

![hmc5883l-1](https://user-images.githubusercontent.com/6020549/227674738-9d9eb071-e494-4de3-8829-d1426c398f3d.jpg)
![hmc5883l-2](https://user-images.githubusercontent.com/6020549/227674741-15c93ca8-17d4-487d-bc3e-12792c536a46.jpg)

### Execute calibration again   
If you set the offset you got from the calibration and run it again, the circle position will change.   

![hmc5883l-11](https://user-images.githubusercontent.com/6020549/227674769-63d77758-c217-4f0f-b7ac-1f310876191b.jpg)
![hmc5883l-12](https://user-images.githubusercontent.com/6020549/227674771-85e8e65c-44c4-4bac-b964-07f62f8bf31b.jpg)


