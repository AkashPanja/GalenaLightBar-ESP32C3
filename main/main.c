#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

#include "galena_protocol.h"

#define TAG                 "GLB"

#define PIN_PWM             GPIO_NUM_4

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY      1000

#define BRIGHTNESS_STEP     3.0f
#define MAX_ENCODER_DELTA   10

static float   g_brightness          = 1.0f;
static bool    g_light_on            = true;
static float   g_brightness_step     = BRIGHTNESS_STEP / 100.0f;
static nvs_handle_t s_nvs_handle;

static QueueHandle_t s_espnow_queue;
static uint8_t s_action_ring_mac[] = ACTION_RING_MAC;

static volatile int32_t g_send_ok   = 0;
static volatile int32_t g_send_fail = 0;

// ─── LED / PWM ───────────────────────────────────────────────────────────────

static void ledc_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = PIN_PWM,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

static void nvs_save_brightness(void)
{
    uint8_t pct = (uint8_t)(g_brightness * 100.0f + 0.5f);
    nvs_set_u8(s_nvs_handle, "bri", pct);
    nvs_commit(s_nvs_handle);
}

static void nvs_save_light_on(void)
{
    uint8_t val = g_light_on ? 1 : 0;
    nvs_set_u8(s_nvs_handle, "on", val);
    nvs_commit(s_nvs_handle);
}

static void set_brightness(float brightness)
{
    brightness = fmaxf(0.01f, fminf(1.0f, brightness));
    g_brightness = brightness;
    nvs_save_brightness();
    uint32_t duty = (uint32_t)(brightness * ((1 << 13) - 1));
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static void apply_light(void)
{
    if (g_light_on) {
        set_brightness(g_brightness);
    } else {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
}

// ─── ESP-NOW Send ─────────────────────────────────────────────────────────────

static void espnow_send_boot(void)
{
    galena_packet_t pkt = {
        .type  = PKT_BOOT,
        .value = 0,
    };
    esp_now_send(s_action_ring_mac, (uint8_t *)&pkt, sizeof(pkt));
    ESP_LOGI(TAG, "TX boot notification");
}

static void espnow_send_light_state(void)
{
    galena_packet_t pkt = {
        .type  = PKT_LIGHT_STATE,
        .value = g_light_on ? 1 : 0,
    };
    esp_err_t ret = esp_now_send(s_action_ring_mac, (uint8_t *)&pkt, sizeof(pkt));
    ESP_LOGI(TAG, "TX light_state=%s ret=%d", g_light_on ? "ON" : "OFF", ret);
}

static void espnow_send_brightness(void)
{
    galena_packet_t pkt = {
        .type  = PKT_BRIGHTNESS,
        .value = (int32_t)(g_brightness * 100.0f),
    };
    esp_err_t ret = esp_now_send(s_action_ring_mac, (uint8_t *)&pkt, sizeof(pkt));
    ESP_LOGI(TAG, "TX brightness=%d%% ret=%d", (int)pkt.value, ret);
}

// ─── ESP-NOW Receive ──────────────────────────────────────────────────────────

typedef struct {
    galena_packet_t pkt;
} espnow_event_t;

static void espnow_recv_cb(const esp_now_recv_info_t *info,
                            const uint8_t *data, int len)
{
    if (len != sizeof(galena_packet_t)) return;
    espnow_event_t evt;
    memcpy(&evt.pkt, data, sizeof(galena_packet_t));
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(s_espnow_queue, &evt, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
        g_send_ok++;
    else
        g_send_fail++;
}

static void espnow_task(void *arg)
{
    espnow_event_t evt;
    for (;;) {
        if (xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
            continue;

        ESP_LOGI(TAG, "RX type=%d val=%d show=%d conn=%d",
                 evt.pkt.type, evt.pkt.value, evt.pkt.osd_show, evt.pkt.connected);

        switch (evt.pkt.type) {

        case PKT_ENCODER: {
            if (evt.pkt.connected && evt.pkt.osd_show) break;
            int delta = (int)evt.pkt.value;
            if (delta == 0 || abs(delta) > MAX_ENCODER_DELTA) break;

            float current = g_light_on ? g_brightness : 0.5f;
            float target  = fmaxf(0.01f, fminf(1.0f,
                                current + g_brightness_step * delta));

            g_brightness = target;
            if (g_light_on) set_brightness(target);

            espnow_send_brightness();

            ESP_LOGI(TAG, "ENCODER delta=%+d bri=%.2f", delta, target);
            break;
        }

        case PKT_BUTTON: {
            if (evt.pkt.connected && evt.pkt.osd_show) break;
            if (evt.pkt.value == 2) {
                g_light_on = !g_light_on;
                nvs_save_light_on();
                apply_light();
                espnow_send_light_state();
                espnow_send_brightness();
                ESP_LOGI(TAG, "TOGGLE -> %s", g_light_on ? "ON" : "OFF");
            }
            break;
        }

        default:
            break;
        }
    }
}

// ─── Periodic Sync (retry until ACK) ──────────────────────────────────────────

static void espnow_sync_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        if (g_send_ok == 0 || g_send_fail > g_send_ok) {
            ESP_LOGW(TAG, "No ACK yet or high fail ratio — retrying sync (ok=%ld fail=%ld)", (long)g_send_ok, (long)g_send_fail);
            espnow_send_light_state();
            espnow_send_brightness();
        }
    }
}

// ─── Debug Stats ──────────────────────────────────────────────────────────────

static void espnow_stats_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "ESPNOW stats: ok=%ld fail=%ld bri=%.0f%% on=%d",
                 (long)g_send_ok, (long)g_send_fail,
                 g_brightness * 100.0f, g_light_on);
    }
}

// ─── Wi-Fi + ESP-NOW Init ─────────────────────────────────────────────────────

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&primary, &second);
    ESP_LOGI(TAG, "Wi-Fi channel=%d (requested 1)", primary);
}

static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    // Set PMK for unencrypted ESP-NOW (avoids key derivation delay)
    uint8_t pmk[] = "galena-pmk-1234";
    ESP_ERROR_CHECK(esp_now_set_pmk(pmk));

    esp_now_peer_info_t peer = { };
    peer.channel = 1;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_action_ring_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

// ─── Main ─────────────────────────────────────────────────────────────────────

void app_main(void)
{
    ESP_LOGI(TAG, "Galena Light Bar booting...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    nvs_open("lightbar", NVS_READWRITE, &s_nvs_handle);
    uint8_t nvs_bri = 100, nvs_on = 1;
    nvs_get_u8(s_nvs_handle, "bri", &nvs_bri);
    nvs_get_u8(s_nvs_handle, "on", &nvs_on);
    g_brightness = fmaxf(0.01f, fminf(1.0f, nvs_bri / 100.0f));
    g_light_on   = (nvs_on != 0);

    ledc_init();
    apply_light();

    s_espnow_queue = xQueueCreate(10, sizeof(espnow_event_t));

    wifi_init();
    espnow_init();

    uint8_t my_mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, my_mac);
    ESP_LOGI(TAG, "My MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             my_mac[0], my_mac[1], my_mac[2], my_mac[3], my_mac[4], my_mac[5]);
    ESP_LOGI(TAG, "Peer MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             s_action_ring_mac[0], s_action_ring_mac[1], s_action_ring_mac[2],
             s_action_ring_mac[3], s_action_ring_mac[4], s_action_ring_mac[5]);

    xTaskCreate(espnow_task,        "espnow_task", 4096, NULL, 5, NULL);
    xTaskCreate(espnow_sync_task,   "espnow_sync", 2048, NULL, 2, NULL);
    xTaskCreate(espnow_stats_task,  "espnow_stats", 2048, NULL, 2, NULL);

    espnow_send_boot();
    espnow_send_light_state();
    espnow_send_brightness();

    ESP_LOGI(TAG, "Galena Light Bar ready.");
}
