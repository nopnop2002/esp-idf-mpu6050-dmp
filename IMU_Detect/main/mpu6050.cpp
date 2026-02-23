// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//		2023-03-10 - Fit to esp-idf v5
//		2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//		2016-04-18 - Eliminated a potential infinite loop
//		2013-05-08 - added seamless Fastwire support
//				   - added note about gyro calibration
//		2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//		2012-06-20 - improved FIFO overflow handling and simplified read process
//		2012-06-19 - completely rearranged DMP initialization code and simplification
//		2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//		2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//		2012-06-05 - add gravity-compensated initial reference frame acceleration output
//				   - add 3D math helper file to DMP6 example sketch
//				   - add Euler output and Yaw/Pitch/Roll output formats
//		2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//		2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//		2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "IMU";

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

void mpu6050(void *pvParameters){
	// Scan I2C slave device
	bool ret = I2Cdev::scanDevice(MPU6050_ADDRESS_AD0_LOW);
	if (ret == false) {
		ESP_LOGE(TAG, "InvenSense device not found");
		vTaskDelete(NULL);
	}

	// Initialize mpu6050
	mpu.initialize(400000);

	// Get Device Row ID
	uint8_t rowid = mpu.getDeviceRowID();
	ESP_LOGI(TAG, "getDeviceRowID=0x%x", rowid);
	if (rowid == 0x19) {
		ESP_LOGI(TAG, "Your IMU is MPU6886");
	} else if (rowid == 0x68) {
		ESP_LOGI(TAG, "Your IMU is MPU6000/MPU6050/MPU9150");
	} else if (rowid == 0x70) {
		ESP_LOGI(TAG, "Your IMU is MPU6500");
	} else if (rowid == 0x71) {
		ESP_LOGI(TAG, "Your IMU is MPU9250");
	} else if (rowid == 0x73) {
		ESP_LOGI(TAG, "Your IMU is MPU9225/MPU9255");
	} else if (rowid == 0x7C) {
		ESP_LOGI(TAG, "Your IMU is MPU6550");
	} else {
		ESP_LOGE(TAG, "Your IMU is Unknown IMU");
	}

	vTaskDelete(NULL);
}
