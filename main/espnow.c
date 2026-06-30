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

#include "galena_protocol.h"

#define TAG                 "GLB"

#define MAX_ENCODER_DELTA   10
#define BRIGHTNESS_STEP_F   0.03f

extern float   g_brightness;
extern bool    g_light_on;
extern float   g_brightness_step;
extern uint8_t g_last_saved_bri_pct;

extern void set_brightness(float brightness);

static QueueHandle_t s_espnow_queue;
static uint8_t s_action_ring_mac[] = ACTION_RING_MAC;

static volatile int32_t g_send_ok   = 0;
static volatile int32_t g_send_fail = 0;

typedef struct {
    galena_packet_t pkt;
} espnow_event_t;

static void espnow_send_light_state(void);
static void espnow_send_brightness(void);

static void espnow_send_boot(void)
{
    galena_packet_t pkt = {
        .type  = PKT_BOOT,
        .value = 0,
    };
    esp_err_t ret = esp_now_send(s_action_ring_mac, (uint8_t *)&pkt, sizeof(pkt));
    if (ret != ESP_OK) ESP_LOGW(TAG, "TX boot FAILED: %d", ret);
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

void espnow_task(void *arg)
{
    espnow_event_t evt;
    for (;;) {
        if (xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
            continue;

        ESP_LOGI(TAG, "RX type=%d val=%d show=%d conn=%d",
                 evt.pkt.type, evt.pkt.value, evt.pkt.osd_show, evt.pkt.connected);

        switch (evt.pkt.type) {

        case PKT_BRIGHTNESS: {
            float val = evt.pkt.value / 100.0f;
            g_brightness = fmaxf(0.01f, fminf(1.0f, val));
            if (g_light_on) set_brightness(g_brightness);
            ESP_LOGI(TAG, "BRIGHTNESS set=%.2f", g_brightness);
            break;
        }

        case PKT_ENCODER: {
            if (evt.pkt.connected && evt.pkt.osd_show) break;
            int delta = (int)evt.pkt.value;
            if (delta == 0 || abs(delta) > MAX_ENCODER_DELTA) break;

            float current = g_brightness;
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

extern void nvs_save_light_on(void);
extern void apply_light(void);

void espnow_sync_task(void *arg)
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

void espnow_stats_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "ESPNOW stats: ok=%ld fail=%ld bri=%.0f%% on=%d",
                 (long)g_send_ok, (long)g_send_fail,
                 g_brightness * 100.0f, g_light_on);
    }
}

void wifi_init(void)
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

void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    uint8_t pmk[] = "galena-pmk-1234";
    ESP_ERROR_CHECK(esp_now_set_pmk(pmk));

    esp_now_peer_info_t peer = { };
    peer.channel = 1;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_action_ring_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

QueueHandle_t get_espnow_queue(void)
{
    if (s_espnow_queue == NULL)
        s_espnow_queue = xQueueCreate(10, sizeof(espnow_event_t));
    return s_espnow_queue;
}
