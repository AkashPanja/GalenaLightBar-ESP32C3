#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "galena_protocol.h"
#include "ledc.h"
#include "nvs_driver.h"
#include "espnow.h"

#define TAG "GLB"

void app_main(void)
{
    ESP_LOGI(TAG, "Galena Light Bar booting...");

    nvs_init();
    ledc_init();
    apply_light();

    QueueHandle_t q = get_espnow_queue();
    if (q == NULL) {
        ESP_LOGE(TAG, "Failed to create espnow queue — aborting");
        abort();
    }

    wifi_init();
    espnow_init();

    uint8_t my_mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, my_mac);
    ESP_LOGI(TAG, "My MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);
    ESP_LOGI(TAG, "Peer MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             LIGHT_BAR_MAC_B0, LIGHT_BAR_MAC_B1, LIGHT_BAR_MAC_B2,
             LIGHT_BAR_MAC_B3, LIGHT_BAR_MAC_B4, LIGHT_BAR_MAC_B5);

    xTaskCreate(espnow_task,       "espnow_task", 4096, NULL, 5, NULL);
    xTaskCreate(espnow_sync_task,  "espnow_sync", 2048, NULL, 2, NULL);
    xTaskCreate(espnow_stats_task, "espnow_stats", 2048, NULL, 2, NULL);

    espnow_send_boot();
    espnow_send_light_state();
    espnow_send_brightness();

    ESP_LOGI(TAG, "Galena Light Bar ready.");
}
