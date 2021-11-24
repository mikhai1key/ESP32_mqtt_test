/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/i2c.h"
#include "mpu6050_sensor.h"

#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

uint8_t sensor_data_h, sensor_data_l;
uint16_t gyrox_data, gyroy_data, gyroz_data;
char gyrox_ch[6], gyroy_ch[6], gyroz_ch[6];
char ch_mes_buf[100];
SemaphoreHandle_t print_mux = NULL;
int msgs_id;
esp_mqtt_client_handle_t my_client;

static const char *TAG = "MQTT_EXAMPLE";


static void log_error_if_nonzero(const char * message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/* reverse:  переворачиваем строку s на месте */
void my_reverse(char s[])
{
    int i, j;
    char c;

    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

void my_itoa(int n, char s[])
{
    int i, sign;

    if ((sign = n) < 0)  /* записываем знак */
        n = -n;          /* делаем n положительным числом */
    i = 0;
    do {       /* генерируем цифры в обратном порядке */
        s[i++] = n % 10 + '0';   /* берем следующую цифру */
    } while ((n /= 10) > 0);     /* удаляем */
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    my_reverse(s);
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    msg_id = 0;
    //msg_id = esp_mqtt_client_publish(client, "/mytopic/qos1", ch_mes_buf, 0, 1, 0);
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_publish(client, "/mytopic/qos1", "ESP32 is connected", 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "/mytopic/qos0", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "/mytopic/qos1", 1);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_unsubscribe(client, "/mytopic/qos1");
            ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            //msg_id = esp_mqtt_client_publish(client, "/mytopic/qos0", "data", 0, 0, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            //msg_id = esp_mqtt_client_publish(client, "/mytopic/qos1", mes2pub, 0, 1, 0);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
    };
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    my_client = client;
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}


static void mpu6050_task(void *arg)
{
	int ret;
	uint32_t task_idx = (uint32_t)arg;
	int cnt = 0;
	while (1)
	{
		ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
	    ret =  i2c_master_read_gyro(I2C_NUM_0, &gyrox_data, &gyroy_data, &gyroz_data);
	    my_itoa((int16_t)gyrox_data,gyrox_ch);
	    my_itoa((int16_t)gyroy_data,gyroy_ch);
	    my_itoa((int16_t)gyroz_data,gyroz_ch);
	    snprintf(&ch_mes_buf[0], sizeof ch_mes_buf, "gx:%s, gy: %s, gz: %s ", gyrox_ch, gyroy_ch, gyroz_ch);
	    ESP_LOGI(TAG, "%s", ch_mes_buf);
	    xSemaphoreTake(print_mux, portMAX_DELAY);
	    if (ret == ESP_ERR_TIMEOUT) {
	    	ESP_LOGE(TAG, "I2C Timeout");
	        } else if (ret == ESP_OK) {
	        	    ESP_LOGI(TAG, "MPU6050 gyrox: %i, gyroy: %i, gyroz: %i", (int16_t) gyrox_data, (int16_t) gyroy_data, (int16_t) gyroz_data);
	        	    //msg_id = esp_mqtt_client_publish(client, "/mytopic/qos1", "data_3", 0, 1, 0);
	        } else {
	            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
	        }
	        xSemaphoreGive(print_mux);
	        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
	        //---------------------------------------------------
	    }
	        vSemaphoreDelete(print_mux);
	        vTaskDelete(NULL);
}

static void mqtt_send_sen(void *arg)
{
	int msg_id = 0;
	//esp_mqtt_client_handle_t client = event->client;
	while(1)
	{
		//int msg_id = esp_mqtt_client_publish(mqtt_client, CONFIG_EXAMPLE_PUBLISH_TOPIC, msg, len, 2, 0);
		msg_id = esp_mqtt_client_publish(my_client, "/mytopic/qos1", ch_mes_buf, 0, 1, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void app_main(void)
{
	print_mux = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());


    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    ESP_ERROR_CHECK(i2c_master_init());

    if(i2c_master_sensor_test(I2C_NUM_0, &sensor_data_h, &sensor_data_l))
    {
    	ESP_LOGI(TAG, "mpu6050 read_data fail");
    }
    else
    {
    	if (sensor_data_h == 0x68)
    	{
    		ESP_LOGI(TAG, "MPU6050 is connected");
    	}
    	else
    	{
    		ESP_LOGI(TAG, "MPU6050 is disconnected");
    	}
    }
    ESP_ERROR_CHECK(mpu6050_init(I2C_NUM_0));
    mqtt_app_start();
    xTaskCreate(mpu6050_task, "mpu6050_test_task", 1024 * 2, (void *)0, 10, NULL);
    xTaskCreate(mqtt_send_sen, "mqtt_send_sen", 1024 * 2, (void *)1, 10, NULL);
}
