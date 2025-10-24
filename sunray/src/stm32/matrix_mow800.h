// Adafruit Grand Central M4  (SAMD51P20A, 1024KB Flash, 256KB RAM)


#if defined(__MOW800__)

#ifndef MATRIX_MOW800
#define MATRIX_MOW800

// extern HardwareSerial Serial2;
// extern HardwareSerial Serial3;

extern HardwareSerial Serial1; // CONSOLE
extern HardwareSerial Serial6; // ESP32 / BLE
extern HardwareSerial Serial4; // GPS

extern void watchdogReset();
extern void watchdogEnable(uint32_t timeout);

#endif  // MATRIX_MOW800

#endif  // __MOW800__

