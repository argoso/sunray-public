#pragma once
#include <Arduino.h>

// UART pinnid – TTGO ESP32 LoRa jaoks
const int GPS_RX_PIN = 21;  // ESP32 RX, LC29 TX
const int GPS_TX_PIN = 25;  // ESP32 TX, LC29 RX
const int STM_RX_PIN = 17;  // ESP32 RX, STM TX
const int STM_TX_PIN = 23;  // ESP32 TX, STM RX

// UART kiirused
const uint32_t GPS_BAUD = 115200;
const uint32_t STM_BAUD = 115200;

// LED pin (TTGO sisemine LED)
const int ESP_LED_PIN = 2;

// LoRa pinnid (samad mis TX)
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_SS    18
#define LORA_RST   14
#define LORA_DIO0  26

// LoRa seaded – peavad klappima TX-ga
static const long     LORA_FREQ_HZ   = 868E6;
static const long     LORA_BW_HZ     = 250E3;
static const uint8_t  LORA_SF        = 7;
static const uint8_t  LORA_CR        = 5;
static const uint8_t  LORA_SYNC_WORD = 0x12;
static const uint8_t  LORA_TX_POWER  = 20;  // dBm (võimsus maksimumini)

static const unsigned long OLED_TIMEOUT_MS = 5UL * 60UL * 1000UL;