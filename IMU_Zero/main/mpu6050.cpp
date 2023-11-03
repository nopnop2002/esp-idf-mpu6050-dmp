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

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050.h" // not necessary if using MotionApps include file
//#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 accelgyro;

const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA	= ',';
const char BLANK	= ' ';
const char PERIOD	= '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150;	// empirical, to hold sampling to 200 Hz
const int NFast =  1000;	// the bigger, the better (but slower)
const int NSlow = 10000;	// ..
const int LinesBetweenHeaders = 5;
int LowValue[6];
int HighValue[6];
int Smoothed[6];
int LowOffset[6];
int HighOffset[6];
int Target[6];
int LinesOut;
int N;

void ForceHeader()
	{ LinesOut = 99; }

void GetSmoothed()
	{ int16_t RawValue[6];
	int i;
	long Sums[6];
	for (i = iAx; i <= iGz; i++)
		{ Sums[i] = 0; }
//	  unsigned long Start = micros();

	for (i = 1; i <= N; i++) { // get sums
		accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz], 
							 &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
		if ((i % 500) == 0)
			printf("%c", PERIOD);
		delayMicroseconds(usDelay);
		for (int j = iAx; j <= iGz; j++)
			Sums[j] = Sums[j] + RawValue[j];
	} // get sums
//	unsigned long usForN = micros() - Start;
//	printf(" reading at ");
//	printf("%f", 1000000/((usForN+N/2)/N));
//	printf(" Hz\n");
	for (i = iAx; i <= iGz; i++)
		{ Smoothed[i] = (Sums[i] + N/2) / N ; }
	} // GetSmoothed

void Initialize() {
	// initialize device
	printf("Initializing I2C devices...\n");
	accelgyro.initialize();

	// verify connection
#if 0
	printf("Testing device connections...");
	printf(accelgyro.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
#endif
	printf("PID tuning Each Dot = 100 readings\n");
	/*A tidbit on how PID (PI actually) tuning works. 
	When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and 
	integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral 
	uses the error from set-point (set-point is zero), it takes a fraction of this error (error * ki) and adds it 
	to the integral value. Each reading narrows the error down to the desired offset. The greater the error from 
	set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the 
	integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the 
	noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100 
	readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to 
	the fact it reacts to any noise.
	*/
	accelgyro.CalibrateAccel(6);
	accelgyro.CalibrateGyro(6);
	printf("\nat 600 Readings\n");
	accelgyro.PrintActiveOffsets();
	printf("\n");
	accelgyro.CalibrateAccel(1);
	accelgyro.CalibrateGyro(1);
	printf("700 Total Readings\n");
	accelgyro.PrintActiveOffsets();
	printf("\n");
	accelgyro.CalibrateAccel(1);
	accelgyro.CalibrateGyro(1);
	printf("800 Total Readings\n");
	accelgyro.PrintActiveOffsets();
	printf("\n");
	accelgyro.CalibrateAccel(1);
	accelgyro.CalibrateGyro(1);
	printf("900 Total Readings\n");
	accelgyro.PrintActiveOffsets();
	printf("\n");	
	accelgyro.CalibrateAccel(1);
	accelgyro.CalibrateGyro(1);
	printf("1000 Total Readings\n");
	accelgyro.PrintActiveOffsets();
	printf("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:\n"); 
  } // Initialize

void SetOffsets(int TheOffsets[6]) {
	accelgyro.setXAccelOffset(TheOffsets [iAx]);
	accelgyro.setYAccelOffset(TheOffsets [iAy]);
	accelgyro.setZAccelOffset(TheOffsets [iAz]);
	accelgyro.setXGyroOffset (TheOffsets [iGx]);
	accelgyro.setYGyroOffset (TheOffsets [iGy]);
	accelgyro.setZGyroOffset (TheOffsets [iGz]);
} // SetOffsets

void SetAveraging(int NewN) {
	N = NewN;
	printf("averaging ");
	printf("%d", N);
	printf(" readings each time\n");
} // SetAveraging

void ShowProgress() {
	if (LinesOut >= LinesBetweenHeaders)
	{ // show header
		printf("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro\n");
		LinesOut = 0;
	} // show header
	printf("%c", BLANK);
	for (int i = iAx; i <= iGz; i++)
	  { printf("%c", LBRACKET);
		printf("%d", LowOffset[i]),
		printf("%c", COMMA);
		printf("%d", HighOffset[i]);
		printf("] --> [");
		printf("%d", LowValue[i]);
		printf("%c", COMMA);
		printf("%d", HighValue[i]);
		if (i == iGz)
		  { printf("%c\n",RBRACKET); }
		else
		  { printf("]\t"); }
	  }
	LinesOut++;
} // ShowProgress

void PullBracketsIn() {
	bool AllBracketsNarrow;
	bool StillWorking;
	int NewOffset[6];
  
	printf("\nclosing in:\n");
	AllBracketsNarrow = false;
	ForceHeader();
	StillWorking = true;
	while (StillWorking) 
	  { StillWorking = false;
		if (AllBracketsNarrow && (N == NFast))
		  { SetAveraging(NSlow); }
		else
		  { AllBracketsNarrow = true; }// tentative
		for (int i = iAx; i <= iGz; i++)
		  { if (HighOffset[i] <= (LowOffset[i]+1))
			  { NewOffset[i] = LowOffset[i]; }
			else
			  { // binary search
				StillWorking = true;
				NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
				if (HighOffset[i] > (LowOffset[i] + 10))
				  { AllBracketsNarrow = false; }
			  } // binary search
		  }
		SetOffsets(NewOffset);
		GetSmoothed();
		for (int i = iAx; i <= iGz; i++)
		  { // closing in
			if (Smoothed[i] > Target[i])
			  { // use lower half
				HighOffset[i] = NewOffset[i];
				HighValue[i] = Smoothed[i];
			  } // use lower half
			else
			  { // use upper half
				LowOffset[i] = NewOffset[i];
				LowValue[i] = Smoothed[i];
			  } // use upper half
		  } // closing in
		ShowProgress();
	  } // still working
   
} // PullBracketsIn

void PullBracketsOut() {
	bool Done = false;
	int NextLowOffset[6];
	int NextHighOffset[6];

	printf("expanding:\n");
	ForceHeader();
 
	while (!Done)
	  { Done = true;
		SetOffsets(LowOffset);
		GetSmoothed();
		for (int i = iAx; i <= iGz; i++)
		  { // got low values
			LowValue[i] = Smoothed[i];
			if (LowValue[i] >= Target[i])
			  { Done = false;
				NextLowOffset[i] = LowOffset[i] - 1000;
			  }
			else
			  { NextLowOffset[i] = LowOffset[i]; }
		  } // got low values
	  
		SetOffsets(HighOffset);
		GetSmoothed();
		for (int i = iAx; i <= iGz; i++)
		  { // got high values
			HighValue[i] = Smoothed[i];
			if (HighValue[i] <= Target[i])
			  { Done = false;
				NextHighOffset[i] = HighOffset[i] + 1000;
			  }
			else
			  { NextHighOffset[i] = HighOffset[i]; }
		  } // got high values
		ShowProgress();
		for (int i = iAx; i <= iGz; i++)
		  { LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
			HighOffset[i] = NextHighOffset[i]; // ..
		  }
	 } // keep going
} // PullBracketsOut

void mpu6050(void *pvParameters){
	Initialize();
	for (int i = iAx; i <= iGz; i++)
	  { // set targets and initial guesses
		Target[i] = 0; // must fix for ZAccel 
		HighOffset[i] = 0;
		LowOffset[i] = 0;
	  } // set targets and initial guesses
	Target[iAz] = 16384;
	SetAveraging(NFast);
	
	PullBracketsOut();
	PullBracketsIn();
	
	printf("-------------- done --------------\n");

	vTaskDelete(NULL);
}
