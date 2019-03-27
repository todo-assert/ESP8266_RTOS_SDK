/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "common.h"

#define TAG "main"
/******************************************************************************
 * FunctionName : app_main
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void app_main(void)
{
    start_smart_config_main();
    spilcd_init();
    printf("SDK version:%s\n", esp_get_idf_version());
    while(!smart_config_over()) {
        vTaskDelay(500 / portTICK_RATE_MS);
    }
    draw_background(background);
    start_sntp_main();
    ESP_LOGI(TAG, "smartconfig done !!!!!!!!!!!!!!!!!!!!!!!!!!!");
    time_t now;
    struct tm timeinfo;
    char strftime_buf[64];
    // const uint8_t *all[] = {cloudy,  overcast};//,  rain}; // ,  snow,  sunny,  thunder};
    while(1) {
extern void lcd_print(uint16_t x, uint16_t y, char *str, uint16_t color);
        // update 'now' variable with current time
        time(&now);
        localtime_r(&now, &timeinfo);

        if (timeinfo.tm_year < (2016 - 1900)) {
            ESP_LOGE(TAG, "The current date/time error");
        } else {
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            // draw_background_anywhere(0, 0, 239, 239, all[i]);
            lcd_print(20, 40, strftime_buf, 0xffff);
            ESP_LOGI(TAG, "The current date/time in Shanghai is: %s", strftime_buf);
        }

        ESP_LOGI(TAG, "Free heap size: %d\n", esp_get_free_heap_size());
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
