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
#include "HMC5883L.h"

// Source: https://github.com/arduino-libraries/MadgwickAHRS
#include "MadgwickAHRS.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

MPU6050 mpu;
HMC5883L mag(HMC5883L_DEFAULT_ADDRESS);
Madgwick madgwick;

// Accel & Gyro scale factor
float accel_sensitivity;
float gyro_sensitivity;

bool getMagData(int16_t *mx, int16_t *my, int16_t *mz) {
	if (!mag.testConnection()) {
		ESP_LOGE(TAG, "*****HMC5883L connection lost*****");
		// Bypass Enable Configuration
		mpu.setI2CBypassEnabled(true);
		vTaskDelay(100);
		return false;
	}

	// Wait until DataReady
	ESP_LOGD(TAG, "mag.getReadyStatus()=0x%x", mag.getReadyStatus());
	for (int retry=0;retry<10;retry++) {
		if (mag.getReadyStatus()) break;
		vTaskDelay(1);
	}

	if (mag.getReadyStatus()) {
		mag.getHeading(mx, my, mz);
		return true;
	} else {
		ESP_LOGE(TAG, "*****HMC5883L data not ready*****");
		ESP_LOGE(TAG, "mag.getReadyStatus()=0x%x", mag.getReadyStatus());
		//vTaskDelay(10);
		return false;
	}
	return false;
}

// Get scaled value
void _getMotion6(float *_ax, float *_ay, float *_az, float *_gx, float *_gy, float *_gz) {
	int16_t ax,ay,az;
	int16_t gx,gy,gz;
	// read raw accel/gyro measurements from device
	// The accelerometer output is a 16-bit signed integer relative value.
	// The gyroscope output is a relative value in degrees per second (dps).
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	// Convert relative to absolute
#if 1
	*_ax = (float)ax / accel_sensitivity;
	*_ay = (float)ay / accel_sensitivity;
	*_az = (float)az / accel_sensitivity;
#else
	*_ax = (float)ax;
	*_ay = (float)ay;
	*_az = (float)az;
#endif

	// Convert relative to absolute
#if 1
	*_gx = ((float)gx / gyro_sensitivity);
	*_gy = ((float)gy / gyro_sensitivity);
	*_gz = ((float)gz / gyro_sensitivity);
#else
	*_gx = (float)gx;
	*_gy = (float)gy;
	*_gz = (float)gz;
#endif
}

// Get time in seconds since boot
// Compatible with ROS's time.toSec() function
double TimeToSec() {
	int64_t _time = esp_timer_get_time(); // Get time in microseconds since boot
	double __time = (double)_time / 1000000;
	return __time;
}

void mpu6050(void *pvParameters){
	// Initialize mpu6050
	mpu.initialize();

	// Get the sample rate
	ESP_LOGI(TAG, "getRate()=%d", mpu.getRate());
	// Set the sample rate to 8kHz
	if (mpu.getRate() != 0) mpu.setRate(0);

	// Get FSYNC configuration value
	ESP_LOGI(TAG, "getExternalFrameSync()=%d", mpu.getExternalFrameSync());
	// Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering
	if (mpu.getExternalFrameSync() != 0) mpu.setExternalFrameSync(0);

	// Set Digital Low Pass Filter
	ESP_LOGI(TAG, "getDLPFMode()=%d", mpu.getDLPFMode());
	if (mpu.getDLPFMode() != 6) mpu.setDLPFMode(6);

	// Get Accelerometer Scale Range
	ESP_LOGI(TAG, "getFullScaleAccelRange()=%d", mpu.getFullScaleAccelRange());
	// Set Accelerometer Full Scale Range to ±2g
	if (mpu.getFullScaleAccelRange() != 0) mpu.setFullScaleAccelRange(0);
	accel_sensitivity = 16384.0;

	// Get Gyro Scale Range
	ESP_LOGI(TAG, "getFullScaleGyroRange()=%d", mpu.getFullScaleGyroRange());
	// Set Gyro Full Scale Range to ±250deg/s
	if (mpu.getFullScaleGyroRange() != 0) mpu.setFullScaleGyroRange(0);
	gyro_sensitivity = 131.0;

	// Bypass Enable Configuration
	mpu.setI2CBypassEnabled(true);

	// Initialize HMC5883L
	// The number of samples averaged per measured output is 8.
	// Data Output Rate is 15Hz.
	// Normal measurement configuration.
	// -1.3Ga-->+1.3Ga 1090 counts / Gauss
	// Single-Measurement Mode.
	mag.initialize();

	// Verify the I2C connection
	if (!mag.testConnection()) {
		ESP_LOGE(TAG, "HMC5883L not found");
		vTaskDelete(NULL);
	}

	uint8_t ida = mag.getIDA();
	ESP_LOGI(TAG, "ida=0x%x", ida);
	if (ida != 0x48) {
		ESP_LOGE(TAG, "Identification Register A not correct [0x%x]", ida);
		vTaskDelete(NULL);
	}

	uint8_t idb = mag.getIDB();
	ESP_LOGI(TAG, "idb=0x%x", idb);
	if (idb != 0x34) {
		ESP_LOGE(TAG, "Identification Register B not correct [0x%x]", idb);
		vTaskDelete(NULL);
	}

	uint8_t idc = mag.getIDC();
	ESP_LOGI(TAG, "idc=0x%x", idc);
	if (idc != 0x33) {
		ESP_LOGE(TAG, "Identification Register C not correct [0x%x]", idc);
		vTaskDelete(NULL);
	}

	double last_time_ = TimeToSec();
	int elasped = 0;
	bool initialized = false;
	float initial_roll = 0.0;
	float initial_pitch = 0.0;

	float roll = 0.0, pitch = 0.0, yaw = 0.0;
	float _roll = 0.0, _pitch = 0.0;
	while(1){
		// Get scaled value
		float ax, ay, az;
		float gx, gy, gz;
		_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		int16_t mx, my, mz;
		float _mx, _my, _mz;
		if (getMagData(&mx, &my, &mz)) {
			ESP_LOGD(TAG, "mag=%d %d %d", mx, my, mz);
			mx = mx + CONFIG_MAGX;
			my = my + CONFIG_MAGY;
			mz = mz + CONFIG_MAGZ;
			_mx = mx / 10.9; //uT
			_my = my / 10.9; //uT
			_mz = mz / 10.9; //uT
			ESP_LOGD(TAG, "mag=%f %f %f", _mx, _my, _mz);

			// Get the elapsed time from the previous
			float dt = (TimeToSec() - last_time_);
			ESP_LOGD(TAG, "dt=%f",dt);
			last_time_ = TimeToSec();

			// Get Euler
			madgwick.update(gx, gy, gz, ax, ay, az, _mx, _my, _mz, dt);
			roll = madgwick.getRoll();
			pitch = madgwick.getPitch();
			yaw = madgwick.getYaw();
			ESP_LOGD(TAG, "roll=%f pitch=%f yaw=%f", roll, pitch, yaw);
		}

		/* Print Data every 10 times */
		if (elasped > 10) {
			// Set the first data
			TickType_t nowTicks = xTaskGetTickCount();
			if (initialized == false && nowTicks > 6000) {
				initial_roll = roll;
				initial_pitch = pitch;
				initialized = true;
			}
			_roll = roll-initial_roll;
			_pitch = pitch-initial_pitch;
			ESP_LOGD(TAG, "roll=%f pitch=%f yaw=%f", roll, pitch, yaw);
			ESP_LOGD(TAG, "roll:%f pitch=%f yaw=%f", _roll, _pitch, yaw);

			if (initialized) {
				ESP_LOGI(TAG, "roll:%f pitch=%f yaw=%f", _roll, _pitch, yaw);
				// Send UDP packet
				POSE_t pose;
				pose.roll = _roll;
				pose.pitch = _pitch;
				pose.yaw = yaw;
				if (xQueueSend(xQueueTrans, &pose, 100) != pdPASS ) {
					ESP_LOGE(pcTaskGetName(NULL), "xQueueSend fail");
				}

				// Send WEB request
				cJSON *request;
				request = cJSON_CreateObject();
				cJSON_AddStringToObject(request, "id", "data-request");
				cJSON_AddNumberToObject(request, "roll", _roll);
				cJSON_AddNumberToObject(request, "pitch", _pitch);
				cJSON_AddNumberToObject(request, "yaw", yaw);
				char *my_json_string = cJSON_Print(request);
				ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
				size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
				if (xBytesSent != strlen(my_json_string)) {
					ESP_LOGE(TAG, "xMessageBufferSend fail");
				}
				cJSON_Delete(request);
				cJSON_free(my_json_string);
			} else {
				ESP_LOGI(TAG, "unstable roll:%f pitch=%f yaw=%f", _roll, _pitch, yaw);
			}

			vTaskDelay(1);
			elasped = 0;
		}
	
		elasped++;
		vTaskDelay(1);
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}
