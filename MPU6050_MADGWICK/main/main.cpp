#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include "mdns.h"
#include "I2Cdev.h"

#include "parameter.h"

#include "websocket_server.h"

MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "MAIN";
static const char *MDNS_HOSTNAME = "ESP32";

QueueHandle_t xQueueTrans;

extern "C" {
	void start_wifi(void);
	void start_mdns(void);
	int ws_server_start(void);
	void udp_trans(void *pvParameters);
	void server_task(void *pvParameters);
	void client_task(void *pvParameters);
	void app_main(void);
}

void mpu6050(void *pvParameters);

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

void app_main(void)
{
	// Initialize WiFi
	start_wifi();

	// Initialize mDNS
	start_mdns();

	// Initialize i2c
	I2Cdev::initialize();

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
	xTaskCreate(&client_task, "CLIENT", 1024*3, (void *)0x011, 5, NULL);

	// Start imu task
	xTaskCreate(&mpu6050, "IMU", 1024*8, NULL, 5, NULL);

	// Start udp task
	xTaskCreate(&udp_trans, "UDP", 1024*3, NULL, 5, NULL);

	vTaskDelay(100);
}
