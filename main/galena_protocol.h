#pragma once
#include <stdint.h>

// ─── Packet Types ────────────────────────────────────────────────────────────
#define PKT_BRIGHTNESS   0x01   // Action Ring ← Light Bar : brightness 0–100
#define PKT_ENCODER      0x02   // Light Bar   ← Action Ring : encoder delta
#define PKT_BUTTON       0x03   // Light Bar   ← Action Ring : button state

// ─── MAC Addresses ───────────────────────────────────────────────────────────
#define LIGHT_BAR_MAC    {0x88, 0x56, 0xA6, 0x2A, 0x43, 0xB0}
#define ACTION_RING_MAC  {0xE0, 0x72, 0xA1, 0xE9, 0x39, 0x44}

// ─── Shared Packet ───────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    uint8_t  type;      // PKT_* above
    int32_t  value;     // brightness*100 (int) | encoder delta | button state
} galena_packet_t;
