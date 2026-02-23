#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "I2Cdev.h"

static const char *TAG = "MAIN";

extern "C" {
	void app_main(void);
}

void mpu6050(void *pvParameters);

void app_main(void)
{
	// Initialize i2c
	I2Cdev::initialize();

	// Start imu task
	xTaskCreate(&mpu6050, "IMU", 1024*8, NULL, 5, NULL);

	vTaskDelay(100);
}
