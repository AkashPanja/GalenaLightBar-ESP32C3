/*
 * Galena Light Bar — ESP-IDF Main
 *
 * Responsibilities:
 *   - LEDC PWM dimmer on GPIO16
 *   - GPIO outputs: 19 (MENU), 21 (LEFT), 22 (RIGHT), 23 (OK)
 *   - GPIO25 local push button (INPUT_PULLUP, active low)
 *   - ESP-NOW peer: Galena Action Ring
 *       TX → PKT_BRIGHTNESS (brightness 0–100) on every change
 *       RX ← PKT_ENCODER (delta) → adjust brightness
 *       RX ← PKT_BUTTON  (state) → toggle light on hold
 */

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

// ─── Config ──────────────────────────────────────────────────────────────────
#define TAG                 "GLB"           // Galena Light Bar

#define PIN_PWM             GPIO_NUM_16
#define PIN_MENU            GPIO_NUM_19
#define PIN_LEFT            GPIO_NUM_21
#define PIN_RIGHT           GPIO_NUM_10
#define PIN_OK              GPIO_NUM_9
#define PIN_BUTTON          GPIO_NUM_8

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT   // 0–8191
#define LEDC_FREQUENCY      1000                // Hz

#define BRIGHTNESS_STEP     3.0f                // % per encoder tick
#define MAX_ENCODER_DELTA   10
#define PULSE_MS            300                 // GPIO button pulse duration
#define DEBOUNCE_MS         50

// ─── Globals ─────────────────────────────────────────────────────────────────
static float   g_brightness        = 1.0f;     // 0.0 – 1.0
static bool    g_light_on          = true;
static float   g_brightness_step   = BRIGHTNESS_STEP / 100.0f;

static QueueHandle_t s_espnow_queue;

static uint8_t s_action_ring_mac[] = ACTION_RING_MAC;

// ─── LEDC helpers ────────────────────────────────────────────────────────────
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

static void set_brightness(float brightness)
{
    brightness = fmaxf(0.01f, fminf(1.0f, brightness));
    g_brightness = brightness;
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

// ─── ESP-NOW TX ──────────────────────────────────────────────────────────────
static void espnow_send_brightness(void)
{
    galena_packet_t pkt = {
        .type  = PKT_BRIGHTNESS,
        .value = (int32_t)(g_brightness * 100.0f),
    };
    esp_now_send(s_action_ring_mac, (uint8_t *)&pkt, sizeof(pkt));
    ESP_LOGI(TAG, "TX brightness → %d%%", (int)pkt.value);
}

// ─── ESP-NOW RX callback ─────────────────────────────────────────────────────
typedef struct {
    galena_packet_t pkt;
} espnow_event_t;

static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len)
{
    if (len != sizeof(galena_packet_t)) return;

    espnow_event_t evt;
    memcpy(&evt.pkt, data, sizeof(galena_packet_t));
    xQueueSendFromISR(s_espnow_queue, &evt, NULL);
}

// ─── GPIO pulse helper ───────────────────────────────────────────────────────
static void pulse_gpio(gpio_num_t pin)
{
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(PULSE_MS));
    gpio_set_level(pin, 0);
}

// ─── Button hold tracker ─────────────────────────────────────────────────────
static TickType_t s_btn_press_tick = 0;
static bool       s_btn_held       = false;

static void IRAM_ATTR button_isr_handler(void *arg)
{
    // handled in polling task to avoid heap use in ISR
}

// ─── ESP-NOW processing task ─────────────────────────────────────────────────
static void espnow_task(void *arg)
{
    espnow_event_t evt;
    for (;;) {
        if (xQueueReceive(s_espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
            continue;

        switch (evt.pkt.type) {

        case PKT_ENCODER: {
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
            bool pressed = (bool)evt.pkt.value;
            // hold (value == 2) → toggle
            if (evt.pkt.value == 2) {
                g_light_on = !g_light_on;
                apply_light();
                espnow_send_brightness();
                ESP_LOGI(TAG, "TOGGLE → %s", g_light_on ? "ON" : "OFF");
            }
            (void)pressed;
            break;
        }

        default:
            break;
        }
    }
}

// ─── Local button polling task ────────────────────────────────────────────────
static void button_task(void *arg)
{
    bool last_state = false;
    TickType_t press_start = 0;
    bool hold_fired = false;

    for (;;) {
        bool pressed = (gpio_get_level(PIN_BUTTON) == 0); // active low

        if (pressed && !last_state) {
            press_start = xTaskGetTickCount();
            hold_fired  = false;
        }

        if (pressed && !hold_fired) {
            TickType_t held = xTaskGetTickCount() - press_start;
            if (held >= pdMS_TO_TICKS(700)) {
                // hold → toggle
                g_light_on = !g_light_on;
                apply_light();
                espnow_send_brightness();
                ESP_LOGI(TAG, "LOCAL HOLD → %s", g_light_on ? "ON" : "OFF");
                hold_fired = true;
            }
        }

        last_state = pressed;
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
    }
}

// ─── WiFi / ESP-NOW init ─────────────────────────────────────────────────────
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
}

static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    esp_now_peer_info_t peer = { };
    peer.channel = 1;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_action_ring_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

// ─── GPIO init ───────────────────────────────────────────────────────────────
static void gpio_init(void)
{
    // Output pins
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << PIN_MENU)  |
                        (1ULL << PIN_LEFT)  |
                        (1ULL << PIN_RIGHT) |
                        (1ULL << PIN_OK),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);

    // Local push button
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << PIN_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);
}

// ─── app_main ────────────────────────────────────────────────────────────────
void app_main(void)
{
    ESP_LOGI(TAG, "Galena Light Bar booting...");

    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ledc_init();
    gpio_init();
    apply_light();  // start at full brightness

    s_espnow_queue = xQueueCreate(10, sizeof(espnow_event_t));

    wifi_init();
    espnow_init();

    xTaskCreate(espnow_task, "espnow_task", 4096, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 4, NULL);

    // Broadcast initial brightness so Action Ring is in sync
    espnow_send_brightness();

    ESP_LOGI(TAG, "Galena Light Bar ready.");
}
