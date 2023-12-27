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
#include "driver/i2c.h"

static const char *TAG = "IMU";

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

bool i2c_scan(uint16_t target) {
	bool result = false;
	esp_err_t res;
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");
	for (uint8_t i = 3; i < 0x78; i++) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
		i2c_master_stop(cmd);

		res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
		if (i % 16 == 0)
			printf("\n%.2x:", i);
		if (res == 0) {
			printf(" %.2x", i);
			if (i == target) result = true;
		} else {
			printf(" --");
		}
		i2c_cmd_link_delete(cmd);
	}
	printf("\n\n");
	return result;
}


void mpu6050(void *pvParameters){
	bool ret = i2c_scan(MPU6050_ADDRESS_AD0_LOW);
	if (ret == false) {
		ESP_LOGE(TAG, "InvenSense device not found");
		vTaskDelete(NULL);
	}

	// Initialize mpu6050
	mpu.initialize();

	// Get Device ID
	uint8_t buffer[1];
	I2Cdev::readByte(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_WHO_AM_I, buffer);
	ESP_LOGI(TAG, "getDeviceID=0x%x", buffer[0]);
	if (buffer[0] == 0x19) {
		ESP_LOGI(TAG, "Your IMU is MPU6886");
	} else if (buffer[0] == 0x68) {
		ESP_LOGI(TAG, "Your IMU is MPU6000/MPU6050/MPU9150");
	} else if (buffer[0] == 0x70) {
		ESP_LOGI(TAG, "Your IMU is MPU6500");
	} else if (buffer[0] == 0x71) {
		ESP_LOGI(TAG, "Your IMU is MPU9250");
	} else if (buffer[0] == 0x73) {
		ESP_LOGI(TAG, "Your IMU is MPU9225/MPU9255");
	} else if (buffer[0] == 0x7C) {
		ESP_LOGI(TAG, "Your IMU is MPU6550");
	} else {
		ESP_LOGE(TAG, "Your IMU is Unknown IMU");
	}

	vTaskDelete(NULL);
}
