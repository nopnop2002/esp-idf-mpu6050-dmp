/* UDP Transmitter

	 This example code is in the Public Domain (or CC0 licensed, at your option.)

	 Unless required by applicable law or agreed to in writing, this
	 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	 CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "esp_log.h"
#include "lwip/sockets.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;

static const char *TAG = "SEND";

// UDP Send Task
void udp_trans(void *pvParameters) {
	ESP_LOGI(TAG, "Start");
#if 0
	PARAMETER_t *task_parameter = pvParameters;
	PARAMETER_t param;
	memcpy((char *)&param, task_parameter, sizeof(PARAMETER_t));
	ESP_LOGI(TAG, "Start:param.port=%d param.ipv4=[%s]", param.port, param.ipv4);
#endif

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	//addr.sin_port = htons(param.port);
	addr.sin_port = htons(5005); // port of pyteapot.py
	addr.sin_addr.s_addr = htonl(INADDR_BROADCAST); /* send message to 255.255.255.255 */
	//addr.sin_addr.s_addr = inet_addr("255.255.255.255"); /* send message to 255.255.255.255 */
	//addr.sin_addr.s_addr = inet_addr(param.ipv4);

	/* create the socket */
	int fd;
	int ret;
	fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP ); // Create a UDP socket.
	LWIP_ASSERT("fd >= 0", fd >= 0);

	POSE_t pose;
	char buffer[64];
	while(1) {
		if(xQueueReceive(xQueueTrans, &pose, portMAX_DELAY)) {
			ESP_LOGD(TAG, "pose=%f %f %f", pose.roll, pose.pitch, pose.yaw);
			//sprintf(buffer, "y168.8099yp12.7914pr-11.8401r");
			int buflen = sprintf(buffer, "y%fyp%fpr%fr", pose.yaw, pose.pitch, pose.roll);
		 	ret = lwip_sendto(fd, buffer, buflen, 0, (struct sockaddr *)&addr, sizeof(addr));
			LWIP_ASSERT("ret == buflen", ret == buflen);
			ESP_LOGD(TAG, "lwip_sendto ret=%d",ret);
		} else {
			ESP_LOGE(TAG, "xQueueReceive fail");
			break;
		}
	}

	/* close socket. Don't reach here. */
	ret = lwip_close(fd);
	LWIP_ASSERT("ret == 0", ret == 0);
	vTaskDelete( NULL );

}

