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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/i2c.h>

#include "parameter.h"

extern QueueHandle_t xQueueTrans;

static const char *TAG = "MPU";

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "AK8963.h"

// Source: https://github.com/TKJElectronics/KalmanFilter
#include "Kalman.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
#define RAD_TO_DEG (180.0/PI)
#define DEG_TO_RAD 0.0174533

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())
#define delay(ms) esp_rom_delay_us(ms*1000)

#define MPU_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

MPU6050 mpu;
AK8963 mag(MAG_ADDRESS);
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ;

/* IMU Data */
int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double magX, magY, magZ;

// Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer
double roll, pitch, yaw;

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

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

	*mx = ((int16_t)rawData[0] << 8) | rawData[1];
	*my = ((int16_t)rawData[2] << 8) | rawData[3];
	*mz = ((int16_t)rawData[4] << 8) | rawData[5];
	ESP_LOGD(TAG, "mx=%d my=%d mz=%d", *mx, *my, *mz);
	return true;
}

bool getMagData(int16_t *mx, int16_t *my, int16_t *mz) {
	ESP_LOGD(TAG, "mag.getDeviceID()=0x%x", mag.getDeviceID());
	if (mag.getDeviceID() != 0x48) {
		ESP_LOGE(TAG, "*****AK8963 connection lost*****");
		ESP_LOGE(TAG, "mag.getDeviceID()=0x%x", mag.getDeviceID());
		I2Cdev::writeByte(MPU_ADDRESS, 0x37, 0x02); // try to connect AK8963
		vTaskDelay(100);
		return false;
	}

	ESP_LOGD(TAG, "mag.getMode()=0x%x", mag.getMode());
	if (mag.getMode() != 0x06) {
		ESP_LOGE(TAG, "*****AK8963 illegal data mode*****");
		ESP_LOGE(TAG, "mag.getMode()=0x%x", mag.getMode());
		I2Cdev::writeByte(MPU_ADDRESS, 0x37, 0x02); // try to connect AK8963
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

void updateAK8963() {
	// Read raw data from mag. Units don't care.
	int16_t mx, my, mz;
	if (getMagData(&mx, &my, &mz)) {
		//magX = mx;
		//magY = my;
		//magZ = mz;
		// adjust sensitivity
		// from datasheet 8.3.11
		magX = mx * magCalibration[0];
		magY = my * magCalibration[1];
		magZ = mz * magCalibration[2];
	}
}

void updatePitchRoll() {
	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	roll = atan2(accY, accZ) * RAD_TO_DEG;
	pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

void updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
	magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
	magZ *= -1;

#if 0
	magX *= magGain[0];
	magY *= magGain[1];
	magZ *= magGain[2];

	magX -= magOffset[0];
	magY -= magOffset[1];
	magZ -= magOffset[2];
#endif

	double rollAngle = kalAngleX * DEG_TO_RAD;
	double pitchAngle = kalAngleY * DEG_TO_RAD;

	double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
	double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
	yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

	yaw *= -1;
}

void mpu6050(void *pvParameters){
	// Initialize mpu6050
	mpu.initialize();

#if 0
	// Get Device ID
	uint8_t DeviceID = mpu.getDeviceID();
	ESP_LOGI(TAG, "DeviceID=0x%x", DeviceID);
	if (DeviceID != 0x34) {
		ESP_LOGE(TAG, "MPU6050 not found");
		vTaskDelete(NULL);
	}
#endif

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

	// Bypass Enable Configuration. Connect AK8963
	I2Cdev::writeByte(MPU_ADDRESS, 0x37, 0x02); // connect AK8963

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


	/* Set Kalman and gyro starting angle */
	updateMPU6050();
	updateAK8963();
	updatePitchRoll();
	updateYaw();

	kalmanX.setAngle(roll); // First set roll starting angle
	gyroXangle = roll;
	compAngleX = roll;

	kalmanY.setAngle(pitch); // Then pitch
	gyroYangle = pitch;
	compAngleY = pitch;

	kalmanZ.setAngle(yaw); // And finally yaw
	gyroZangle = yaw;
	compAngleZ = yaw;

	int elasped = 0;
	uint32_t timer = micros();

	bool initialized = false;
	double initial_roll = 0.0;
	double initial_pitch = 0.0;
	double initial_yaw = 0.0;

	while(1){
		/* Update all the IMU values */
		updateMPU6050();
		updateAK8963();

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
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
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
			gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

		/* Yaw estimation */
		updateYaw();
		double gyroZrate = gyroZ / 131.0; // Convert to deg/s
		// This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
		if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
			kalmanZ.setAngle(yaw);
			compAngleZ = yaw;
			kalAngleZ = yaw;
			gyroZangle = yaw;
		} else
			kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter

		/* Estimate angles using gyro only */
		gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
		gyroYangle += gyroYrate * dt;
		gyroZangle += gyroZrate * dt;
		//gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate from the Kalman filter
		//gyroYangle += kalmanY.getRate() * dt;
		//gyroZangle += kalmanZ.getRate() * dt;

		/* Estimate angles using complimentary filter */
		compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
		compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
		compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

		// Reset the gyro angles when they has drifted too much
		if (gyroXangle < -180 || gyroXangle > 180) gyroXangle = kalAngleX;
		if (gyroYangle < -180 || gyroYangle > 180) gyroYangle = kalAngleY;
		if (gyroZangle < -180 || gyroZangle > 180) gyroZangle = kalAngleZ;

		
		/* Print Data every 10 times */
		if (elasped > 10) {
			// Set the first data
			if (!initialized) {
				initial_roll = roll;
				initial_pitch = pitch;
				initial_yaw = yaw;
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

			printf("yaw:%f", yaw); printf("\t");
			printf("initial_yaw: %f", initial_yaw); printf(" ");
			printf("gyroZangle:%f", gyroZangle); printf("\t");
			printf("compAngleZ:%f", compAngleZ); printf("\t");
			printf("kalAngleZ:%f", kalAngleZ); printf("\t");
			printf("\n");
#endif

			/* Send packet */
			float _roll = roll-initial_roll;
			float _pitch = pitch-initial_pitch;
			float _yaw = yaw-initial_yaw;
			ESP_LOGI(TAG, "roll:%f pitch=%f yaw=%f", _roll, _pitch, _yaw);

			POSE_t pose;
			pose.roll = _roll;
			pose.pitch = _pitch;
			pose.yaw = _yaw;
			if (xQueueSend(xQueueTrans, &pose, 100) != pdPASS ) {
				ESP_LOGE(pcTaskGetName(NULL), "xQueueSend fail");
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
