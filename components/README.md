# KalmanFilter   
I used [this](https://github.com/TKJElectronics/KalmanFilter) Kalman Filter by TKJ Electronics.   

# MadgwickAHRS   
I based it on [this](https://github.com/arduino-libraries/MadgwickAHRS).   
The original is for Arduino IDE.   
Arduino IDE is a single-tasking development environment, so you can call functions periodically.   
On the other hand, ESP-IDF is a multi-tasking development environment and cannot call functions periodically.   
There will always be a slight time lag.   
For this filter, the sampling period (time difference from the previous execution) is very important.   
The original is fixed at 1.0 second, but I changed this to a parameter.   


# I2Cdev library collection   
I used [this](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino) I2Cdev library collection by Jeff Rowberg.   
However, I2Cdev has been rewritten for ESP-IDF.   

# websocket   
I used [this](https://github.com/Molorius/esp32-websocket) ESP-IDF WebSocket Component.   
