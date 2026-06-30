#include <math.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "nvs_driver.h"
#include "ledc.h"

#define TAG                 "GLB"

static nvs_handle_t s_nvs_handle;

extern float   g_brightness;
extern bool    g_light_on;
extern uint8_t g_last_saved_bri_pct;

void nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_err_t nvs_open_ret = nvs_open("lightbar", NVS_READWRITE, &s_nvs_handle);
    uint8_t nvs_bri = 100, nvs_on = 1;
    if (nvs_open_ret == ESP_OK) {
        if (nvs_get_u8(s_nvs_handle, "bri", &nvs_bri) != ESP_OK)
            ESP_LOGW(TAG, "NVS: no saved brightness, using default");
        if (nvs_get_u8(s_nvs_handle, "on", &nvs_on) != ESP_OK)
            ESP_LOGW(TAG, "NVS: no saved light state, using default");
    } else {
        ESP_LOGW(TAG, "NVS open failed: %d, using defaults", nvs_open_ret);
    }

    g_brightness = fmaxf(0.01f, fminf(1.0f, nvs_bri / 100.0f));
    g_light_on   = (nvs_on != 0);
    g_last_saved_bri_pct = (uint8_t)(g_brightness * 100.0f + 0.5f);
}

void nvs_save_brightness(void)
{
    uint8_t pct = (uint8_t)(g_brightness * 100.0f + 0.5f);
    if (abs((int)pct - (int)g_last_saved_bri_pct) < 1) return;
    g_last_saved_bri_pct = pct;
    esp_err_t e = nvs_set_u8(s_nvs_handle, "bri", pct);
    if (e != ESP_OK) { ESP_LOGW(TAG, "NVS set bri failed: %d", e); return; }
    e = nvs_commit(s_nvs_handle);
    if (e != ESP_OK) ESP_LOGW(TAG, "NVS commit bri failed: %d", e);
}

void nvs_save_light_on(void)
{
    uint8_t val = g_light_on ? 1 : 0;
    esp_err_t e = nvs_set_u8(s_nvs_handle, "on", val);
    if (e != ESP_OK) { ESP_LOGW(TAG, "NVS set on failed: %d", e); return; }
    e = nvs_commit(s_nvs_handle);
    if (e != ESP_OK) ESP_LOGW(TAG, "NVS commit on failed: %d", e);
}
