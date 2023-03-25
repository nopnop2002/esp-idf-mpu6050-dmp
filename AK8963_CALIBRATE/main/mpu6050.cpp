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

#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "cJSON.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "IMU";

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "AK8963.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

#define MAG_ADDRESS 0x0C

MPU6050 mpu;
AK8963 mag(MAG_ADDRESS);

// MAG Data Sensitivity adjustment data
// Sensitivity adjustment data for each axis is stored to fuse ROM on shipment.
uint8_t MagAdjustmentValue[3];
float magCalibration[3];

bool getMagRaw(int16_t *mx, int16_t *my, int16_t *mz) {
	uint8_t rawData[7];
	for (int i=0;i<7;i++) {
		uint8_t reg = i + 0x03;
		uint8_t _raw;
		I2Cdev::readByte(MAG_ADDRESS, reg, &_raw);
		rawData[i] = _raw;
		ESP_LOGD(TAG, "read_mag(0x%d)=%x", reg, rawData[i]);
	}

	if(rawData[6] & 0x08) {
		ESP_LOGE(TAG, "*****magnetic sensor overflow*****");
		return false;
	}

#if 0
	*mx = ((int16_t)rawData[0] << 8) | rawData[1];
	*my = ((int16_t)rawData[2] << 8) | rawData[3];
	*mz = ((int16_t)rawData[4] << 8) | rawData[5];
#else
	*mx = ((int16_t)rawData[1] << 8) | rawData[0];
	*my = ((int16_t)rawData[3] << 8) | rawData[2];
	*mz = ((int16_t)rawData[5] << 8) | rawData[4];
#endif
	ESP_LOGD(TAG, "mx=0x%x my=0x%x mz=0x%x", *mx, *my, *mz);
	return true;
}

bool getMagData(int16_t *mx, int16_t *my, int16_t *mz) {
	ESP_LOGD(TAG, "mag.getDeviceID()=0x%x", mag.getDeviceID());
	if (mag.getDeviceID() != 0x48) {
		ESP_LOGE(TAG, "*****AK8963 connection lost*****");
		ESP_LOGE(TAG, "mag.getDeviceID()=0x%x", mag.getDeviceID());
		// Bypass Enable Configuration
		mpu.setI2CBypassEnabled(true);
		vTaskDelay(100);
		return false;
	}

	ESP_LOGD(TAG, "mag.getMode()=0x%x", mag.getMode());
	if (mag.getMode() != 0x06) {
		ESP_LOGE(TAG, "*****AK8963 illegal data mode*****");
		ESP_LOGE(TAG, "mag.getMode()=0x%x", mag.getMode());
		// Bypass Enable Configuration
		mpu.setI2CBypassEnabled(true);
		vTaskDelay(100);
		return false;
	}

	// Wait until DataReady
	ESP_LOGD(TAG, "mag.getDataReady()=0x%x", mag.getDataReady());
	for (int retry=0;retry<10;retry++) {
		if (mag.getDataReady()) break;
		vTaskDelay(1);
	}

	if (mag.getDataReady()) {
		if (getMagRaw(mx, my, mz)) {
			return true;
		} else {
			ESP_LOGE(TAG, "*****AK8963 magnetic sensor overflow*****");
			return false;
		}
	} else {
		ESP_LOGE(TAG, "*****AK8963 data not ready*****");
		ESP_LOGE(TAG, "mag.getDataReady()=0x%x", mag.getDataReady());
		//vTaskDelay(10);
		return false;
	}
	return false;
}

void mpu6050(void *pvParameters){
	// Initialize mpu6050
	mpu.initialize();

	// Bypass Enable Configuration
	mpu.setI2CBypassEnabled(true);

	// Get MAG Device ID
	uint8_t MagDeviceID = mag.getDeviceID();
	ESP_LOGI(TAG, "MagDeviceID=0x%x", MagDeviceID);
	if (MagDeviceID != 0x48) {
		ESP_LOGE(TAG, "AK8963 not found");
		vTaskDelete(NULL);
	}

	// Goto Powerdown Mode
	mag.setMode(0x00);
	// After power-down mode is set, at least 100ms(Twat) is needed before setting another mode.
	delay(200);

	// Goto Fuse ROM access mode
	mag.setMode(0x0F);
	delay(200);

	// Get Sensitivity adjustment value from fuse ROM
	// Sensitivity adjustment values for each axis are written in the fuse ROM at the time of shipment.
	MagAdjustmentValue[0] = mag.getAdjustmentX();
	MagAdjustmentValue[1] = mag.getAdjustmentY();
	MagAdjustmentValue[2] = mag.getAdjustmentZ();
	ESP_LOGI(TAG, "MagAdjustmentValue: %x %x %x", MagAdjustmentValue[0], MagAdjustmentValue[1], MagAdjustmentValue[2]);

	// Calculate sensitivity
	// from datasheet 8.3.11
	for (int i=0;i<3;i++) {
		magCalibration[i] = (float)(MagAdjustmentValue[i] - 128)/256.0f + 1.0f;
		ESP_LOGI(TAG, "magCalibration[%d]=%f",i, magCalibration[i]);
	}

	// Goto Powerdown Mode
	mag.setMode(0x00);
	delay(200);

	// Goto oparation mode
	// mode1:Automatically repeat sensor measurements at 8Hz
	// mode2:Automatically repeat sensor measurements at 100Hz
	// 0x?0:Power doen mode.
	// 0x?1:Single measurement mode.
	// 0x?2:Continuous measurement mode 1.
	// 0x?6:Continuous measurement mode 2.
	// 0x?8:External trigger measurement mode.
	// 0x0?:14 bit output 0.60 uT/LSB --> 1 = 0.60uT
	// 0x1?:16 bit output 0.15 uT/LSB --> 1 = 0.15uT
	mag.setMode(0x16);
	//mag.setMode(0x06);

	while(1){

		int16_t mx, my, mz;
		float _mx, _my, _mz;
		if (getMagData(&mx, &my, &mz)) {
			ESP_LOGD(TAG, "mag=%d %d %d", mx, my, mz);
			mx = mx + CONFIG_MAGX;
			my = my + CONFIG_MAGY;
			mz = mz + CONFIG_MAGZ;
			// adjust sensitivity
			// from datasheet 8.3.11
			_mx = mx * magCalibration[0];
			_my = my * magCalibration[1];
			_mz = mz * magCalibration[2];
			ESP_LOGD(TAG, "mag=%f %f %f", _mx, _my, _mz);
			
			float __mx = _mx * 0.6;
			float __my = _my * 0.6;
			float __mz = _mz * 0.6;
			ESP_LOGI(TAG, "mag[uT]=%f %f %f", __mx, __my, __mz);

			// Send WEB request
			cJSON *request;
			request = cJSON_CreateObject();
			cJSON_AddStringToObject(request, "id", "data-request");
			cJSON_AddNumberToObject(request, "roll", _mx);
			cJSON_AddNumberToObject(request, "pitch", _my);
			cJSON_AddNumberToObject(request, "yaw", _mz);
			char *my_json_string = cJSON_Print(request);
			ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
			size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
			if (xBytesSent != strlen(my_json_string)) {
				ESP_LOGE(TAG, "xMessageBufferSend fail");
			}
			cJSON_Delete(request);
			cJSON_free(my_json_string);
	        vTaskDelay(10);
		}
		vTaskDelay(1);
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}
