#pragma once
#include "esp_err.h"

void wifi_init(void);
void espnow_init(void);
void espnow_task(void *arg);
void espnow_sync_task(void *arg);
void espnow_stats_task(void *arg);
void espnow_send_boot(void);
void espnow_send_light_state(void);
void espnow_send_brightness(void);
