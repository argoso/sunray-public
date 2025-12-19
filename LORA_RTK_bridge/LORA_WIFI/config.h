#pragma once
#include <Arduino.h>

// =========== WIFI ===========
static const char WIFI_SSID[]     = "wifi_ssid";
static const char WIFI_PASSWORD[] = "wifi_pass";

// =========== RTCM SERVER ===========
static const IPAddress RTCM_SERVER_IP(192, 168, 8, 149);
static const uint16_t  RTCM_SERVER_PORT = 2102;

// =========== AJASTUSED ===========
static const unsigned long WIFI_RETRY_MS   = 5000UL;
static const unsigned long SERVER_RETRY_MS = 5000UL;
static const unsigned long TCP_TIMEOUT_MS  = 15000UL;

// =========== LoRa (TTGO ESP32 LoRa, SX1276) ===========
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_SS    18
#define LORA_RST   14
#define LORA_DIO0  26

static const long     LORA_FREQ_HZ   = 868E6;   // 868 MHz
static const long     LORA_BW_HZ     = 250E3;   // 250 kHz
static const uint8_t  LORA_SF        = 7;       // SF7
static const uint8_t  LORA_CR        = 5;       // 4/5
static const uint8_t  LORA_SYNC_WORD = 0x12;
static const int8_t   LORA_TX_POWER  = 20;      // dBm

// max payload ühes LoRa paketis (SX1276 piir 255)
static const uint8_t  LORA_PACKET_MAX = 250;

// =========== OLED ===========
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    16

static const unsigned long OLED_TIMEOUT_MS = 5UL * 60UL * 1000UL;
