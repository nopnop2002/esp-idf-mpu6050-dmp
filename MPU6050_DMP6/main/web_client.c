/*
	Example using WEB Socket.
	This example code is in the Public Domain (or CC0 licensed, at your option.)
	Unless required by applicable law or agreed to in writing, this
	software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include "cJSON.h"

static const char *TAG = "client_task";

#include "websocket_server.h"

extern MessageBufferHandle_t xMessageBufferToClient;

void client_task(void* pvParameters) {
	int32_t task_parameter = (int32_t)pvParameters;
	ESP_LOGI(TAG,"Starting. task_parameter=0x%"PRIx32, task_parameter);

	char meter1[8];
	memset(meter1, 0, sizeof(meter1));
	if (task_parameter & 0x0001) strcpy(meter1, "ROLL");
	char meter2[8];
	memset(meter2, 0, sizeof(meter2));
	if (task_parameter & 0x0010) strcpy(meter2, "PITCH");
	char meter3[8];
	memset(meter3, 0, sizeof(meter3));
	if (task_parameter & 0x0100) strcpy(meter3, "YAW");

	char cRxBuffer[512];
	char DEL = 0x04;
	char outBuffer[64];

	while (1) {
		size_t readBytes = xMessageBufferReceive(xMessageBufferToClient, cRxBuffer, sizeof(cRxBuffer), portMAX_DELAY );
		ESP_LOGD(TAG, "readBytes=%d", readBytes);
		ESP_LOGD(TAG, "cRxBuffer=[%.*s]", readBytes, cRxBuffer);
		cJSON *root = cJSON_Parse(cRxBuffer);
		if (cJSON_GetObjectItem(root, "id")) {
			char *id = cJSON_GetObjectItem(root,"id")->valuestring;
			ESP_LOGD(TAG, "id=[%s]",id);

			if ( strcmp (id, "init") == 0) {
				sprintf(outBuffer,"HEAD%cEuler Display using ESP32", DEL);
				ESP_LOGD(TAG, "outBuffer=[%s]", outBuffer);
				ws_server_send_text_all(outBuffer,strlen(outBuffer));

				sprintf(outBuffer,"METER%c%s%c%s%c%s", DEL,meter1,DEL,meter2,DEL,meter3);
				ESP_LOGD(TAG, "outBuffer=[%s]", outBuffer);
				ws_server_send_text_all(outBuffer,strlen(outBuffer));
			} // end if

			if ( strcmp (id, "data-request") == 0) {
				double roll = cJSON_GetObjectItem(root,"roll")->valuedouble;
				double pitch = cJSON_GetObjectItem(root,"pitch")->valuedouble;
				double yaw = cJSON_GetObjectItem(root,"yaw")->valuedouble;
				ESP_LOGD(TAG,"roll=%f pitch=%f yaw=%f", roll, pitch, yaw);

				sprintf(outBuffer,"DATA%c%f%c%f%c%f", DEL, roll, DEL, pitch, DEL, yaw);
				ESP_LOGD(TAG, "outBuffer=[%s]", outBuffer);
				ws_server_send_text_all(outBuffer,strlen(outBuffer));

				ESP_LOGD(TAG,"free_size:%d %d", heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_free_size(MALLOC_CAP_32BIT));

			} // end if
		} // end if

		// Delete a cJSON structure
		cJSON_Delete(root);

	} // end while

	// Never reach here
	vTaskDelete(NULL);
}
