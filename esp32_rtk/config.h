#ifndef CONFIG_H
#define CONFIG_H

// WiFi seadistus
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";

// AP režiimi seadistus (kui WiFi ühendus ebaõnnestub)
const char* AP_SSID = "ESP32-RTK-AP";
const char* AP_PASS = "12345678";

// NTRIP seadistus
const bool USE_NTRIP = true;
const char* NTRIP_HOST = "192.168.1.111"; 
const uint16_t NTRIP_PORT = 2101;
const char* NTRIP_MOUNT = "some";
const char* NTRIP_USER = "";  
const char* NTRIP_PASS = "";  

// UART pinnid
const int GPS_RX_PIN = 16;
const int GPS_TX_PIN = 17;
const int STM_RX_PIN = 4;
const int STM_TX_PIN = 5;

// UART kiirused
const uint32_t GPS_BAUD = 460800;
const uint32_t STM_BAUD = 115200;

// LED pin
const int ESP_LED_PIN = 2;

// Ühenduse seadistused
const unsigned long RECONNECT_INTERVAL = 10000;
const unsigned long NTRIP_TIMEOUT = 30000;
const unsigned long WIFI_TIMEOUT = 15000;

#endif