#include <math.h>

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "ledc.h"
#include "nvs_driver.h"

#define TAG                 "GLB"

#define PIN_PWM             GPIO_NUM_4

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY      1000

float   g_brightness          = 1.0f;
bool    g_light_on            = true;
float   g_brightness_step     = 3.0f / 100.0f;
uint8_t g_last_saved_bri_pct  = 100;

void ledc_init(void)
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

void set_brightness(float brightness)
{
    brightness = fmaxf(0.01f, fminf(1.0f, brightness));
    g_brightness = brightness;
    nvs_save_brightness();
    uint32_t duty = (uint32_t)(brightness * ((1 << 13) - 1));
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void apply_light(void)
{
    if (g_light_on) {
        set_brightness(g_brightness);
    } else {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
}
