# MadgwickAHRS
I based it on [this](https://github.com/arduino-libraries/MadgwickAHRS).   
The original is for Arduino IDE.   
Arduino IDE is a single-tasking development environment, so you can call functions periodically.   
On the other hand, ESP-IDF is a multi-tasking development environment and cannot call functions periodically.   
There will always be a slight time lag.   
For this filter, the sampling period (time difference from the previous execution) is very important.   
The original is fixed at 1.0 second, but I changed this to a parameter.   

