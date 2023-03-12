#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <driver/i2c.h>

#include "parameter.h"

QueueHandle_t xQueueTrans;

extern "C" {
	void app_main(void);
}

void mpu6050(void *pvParameters);

#ifdef __cplusplus
extern "C" {
#endif
void start_wifi(void);
void udp_trans(void *pvParameters);
#ifdef __cplusplus
}
#endif

void app_main(void)
{
	start_wifi();

	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)CONFIG_GPIO_SDA;
	conf.scl_io_num = (gpio_num_t)CONFIG_GPIO_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	/* Create Queue */
	xQueueTrans = xQueueCreate(10, sizeof(POSE_t));
	configASSERT( xQueueTrans );

	vTaskDelay(500/portTICK_PERIOD_MS);
	xTaskCreate(&mpu6050, "IMU", 1024*8, NULL, 5, NULL);
	xTaskCreate(&udp_trans, "TRANS", 1024*2, NULL, 5, NULL);

	while(1) {
		vTaskDelay(100);
	}
}
