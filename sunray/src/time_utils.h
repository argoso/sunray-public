#pragma once
#include <Arduino.h>
#include <stdint.h>

// alias — lihtsalt ühtne koht
static inline uint32_t nowMs() { return millis(); }

// möödunud aeg (rollover-safe)
static inline uint32_t elapsedMs(uint32_t since) { return (uint32_t)(millis() - since); }

// kas vähemalt 'ms' on möödunud?
static inline bool hasElapsed(uint32_t since, uint32_t ms) { return elapsedMs(since) >= ms; }
