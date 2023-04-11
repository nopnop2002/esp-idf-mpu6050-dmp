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

// Source: https://github.com/TKJElectronics/KalmanFilter
#include "Kalman.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())
#define delay(ms) esp_rom_delay_us(ms*1000)

MPU6050 mpu;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double roll, pitch; // Roll and pitch are calculated using the accelerometer

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

void updateMPU6050() {
	// Read raw data from imu. Units don't care.
	mpu.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
	accX = raw_ax;
	accY = raw_ay;
	accZ = raw_az;
	gyroX = raw_gx;
	gyroY = raw_gy;
	gyroZ = raw_gz;
}

void updatePitchRoll() {
	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -πto π(radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	roll = atan2(accY, accZ) * RAD_TO_DEG;
	pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
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

	// Get Gyro Scale Range
	ESP_LOGI(TAG, "getFullScaleGyroRange()=%d", mpu.getFullScaleGyroRange());
	// Set Gyro Full Scale Range to ±250deg/s
	if (mpu.getFullScaleGyroRange() != 0) mpu.setFullScaleGyroRange(0);


	/* Set Kalman and gyro starting angle */
	updateMPU6050();
	updatePitchRoll();

	kalmanX.setAngle(roll); // First set roll starting angle
	gyroXangle = roll;
	compAngleX = roll;

	kalmanY.setAngle(pitch); // Then pitch
	gyroYangle = pitch;
	compAngleY = pitch;

	int elasped = 0;
	uint32_t timer = micros();

	bool initialized = false;
	double initial_roll = 0.0;
	double initial_pitch = 0.0;

	while(1){
		updateMPU6050();

		double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
		timer = micros();

		/* Roll and pitch estimation */
		updatePitchRoll();
		double gyroXrate = gyroX / 131.0; // Convert to deg/s
		double gyroYrate = gyroY / 131.0; // Convert to deg/s


#ifdef RESTRICT_PITCH
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
			kalmanX.setAngle(roll);
			compAngleX = roll;
			kalAngleX = roll;
			gyroXangle = roll;
		} else
			kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	

		if (abs(kalAngleX) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
			kalmanY.setAngle(pitch);
			compAngleY = pitch;
			kalAngleY = pitch;
			gyroYangle = pitch;
		} else
			kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleY) > 90)
			gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

		/* Estimate angles using gyro only */
		gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
		gyroYangle += gyroYrate * dt;
		//gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
		//gyroYangle += kalmanY.getRate() * dt;

		/* Estimate angles using complimentary filter */
		compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
		compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

		// Reset the gyro angle when it has drifted too much
		if (gyroXangle < -180 || gyroXangle > 180) gyroXangle = kalAngleX;
		if (gyroYangle < -180 || gyroYangle > 180) gyroYangle = kalAngleY;

		/* Print Data every 10 times */
		if (elasped > 10) {
			// Set the first data
			if (!initialized) {
				initial_roll = roll;
				initial_pitch = pitch;
				initialized = true;
			}

#if 0
			printf("roll:%f", roll); printf(" ");
			printf("initial_roll:%f", initial_roll); printf(" ");
			printf("roll-initial_roll:%f", roll-initial_roll); printf(" ");
			printf("gyroXangle:%f", gyroXangle); printf(" ");
			printf("compAngleX:%f", compAngleX); printf(" ");
			printf("kalAngleX:%f", kalAngleX); printf(" ");
			printf("\n");

			printf("pitch: %f", pitch); printf(" ");
			printf("initial_pitch: %f", initial_pitch); printf(" ");
			printf("pitch-initial_pitch: %f", pitch-initial_pitch); printf(" ");
			printf("gyroYangle:%f", gyroYangle); printf(" ");
			printf("compAngleY:%f", compAngleY); printf(" ");
			printf("kalAngleY:%f", kalAngleY); printf("\t");
			printf("\n");
#endif

			// Send UDP packet
			float _roll = roll-initial_roll;
			float _pitch = pitch-initial_pitch;
			ESP_LOGI(TAG, "roll:%f pitch=%f", _roll, _pitch);

			POSE_t pose;
			pose.roll = _roll;
			pose.pitch = _pitch;
			pose.yaw = 0.0;
			if (xQueueSend(xQueueTrans, &pose, 100) != pdPASS ) {
				ESP_LOGE(TAG, "xQueueSend fail");
			}

			// Send WEB request
			cJSON *request;
			request = cJSON_CreateObject();
			cJSON_AddStringToObject(request, "id", "data-request");
			cJSON_AddNumberToObject(request, "roll", _roll);
			cJSON_AddNumberToObject(request, "pitch", _pitch);
			cJSON_AddNumberToObject(request, "yaw", 0.0);
			char *my_json_string = cJSON_Print(request);
			ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
			size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
			if (xBytesSent != strlen(my_json_string)) {
				ESP_LOGE(TAG, "xMessageBufferSend fail");
			}
			cJSON_Delete(request);
			cJSON_free(my_json_string);

			vTaskDelay(1);
			elasped = 0;
		}
	
		elasped++;
		vTaskDelay(1);
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}
