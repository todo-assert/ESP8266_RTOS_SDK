/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

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
// while(1);
}
