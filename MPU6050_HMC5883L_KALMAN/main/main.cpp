#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include "mdns.h"
#include "driver/i2c.h"

#include "parameter.h"

#include "websocket_server.h"

MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "MAIN";
static const char *MDNS_HOSTNAME = "ESP32";

QueueHandle_t xQueueTrans;

extern "C" {
	void app_main(void);
}

void mpu6050(void *pvParameters);

#ifdef __cplusplus
extern "C" {
#endif
void start_wifi(void);
void start_mdns(void);
void start_i2c(void);
int ws_server_start(void);
void udp_trans(void *pvParameters);
void server_task(void *pvParameters);
void client_task(void *pvParameters);
#ifdef __cplusplus
}
#endif

void start_mdns(void)
{
	//initialize mDNS
	ESP_ERROR_CHECK( mdns_init() );
	//set mDNS hostname (required if you want to advertise services)
	ESP_ERROR_CHECK( mdns_hostname_set(MDNS_HOSTNAME) );
	ESP_LOGI(TAG, "mdns hostname set to: [%s]", MDNS_HOSTNAME);

	//initialize service
	ESP_ERROR_CHECK( mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0) );

#if 0
	//set default mDNS instance name
	ESP_ERROR_CHECK( mdns_instance_name_set("ESP32 with mDNS") );
#endif
}

void start_i2c(void) {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)CONFIG_GPIO_SDA;
	conf.scl_io_num = (gpio_num_t)CONFIG_GPIO_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	conf.clk_flags = 0;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

void app_main(void)
{
	// Initialize WiFi
	start_wifi();

	// Initialize mDNS
	start_mdns();

	// Initialize i2c
	start_i2c();

	// Create Queue
	xQueueTrans = xQueueCreate(10, sizeof(POSE_t));
	configASSERT( xQueueTrans );

	// Create Message Buffer
	xMessageBufferToClient = xMessageBufferCreate(1024);
	configASSERT( xMessageBufferToClient );

	/* Get the local IP address */
	esp_netif_ip_info_t ip_info;
	ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info));
	char cparam0[64];
	sprintf(cparam0, IPSTR, IP2STR(&ip_info.ip));
	ESP_LOGI(TAG, "cparam0=[%s]", cparam0);

	// Start web socket server
	ws_server_start();

	// Start web server
	xTaskCreate(&server_task, "SERVER", 1024*3, (void *)cparam0, 5, NULL);

	// Start web client
	xTaskCreate(&client_task, "CLIENT", 1024*3, (void *)0x111, 5, NULL);

	// Start imu task
	xTaskCreate(&mpu6050, "IMU", 1024*8, NULL, 5, NULL);

	// Start udp task
	xTaskCreate(&udp_trans, "UDP", 1024*3, NULL, 5, NULL);

	vTaskDelay(100);
}
