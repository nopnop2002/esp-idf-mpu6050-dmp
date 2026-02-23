#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "I2Cdev.h"

extern "C" {
	void app_main(void);
}

void mpu6050(void *pvParameters);

void app_main(void)
{
	// Initialize i2c
	I2Cdev::initialize();

	vTaskDelay(500/portTICK_PERIOD_MS);
	xTaskCreate(&mpu6050, "IMU", 1024*8, NULL, 5, NULL);
}
