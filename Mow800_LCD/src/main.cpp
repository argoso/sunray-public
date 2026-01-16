#include <Arduino.h>
#include <U8g2lib.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ================= LCD pin mapping =================
#define LCD_CS PB12
#define LCD_RST PB13
#define LCD_DC PB14
#define LCD_RW PB15
#define LCD_E PC6
#define LCD_D0 PC7
#define LCD_D1 PC8
#define LCD_D2 PC9
#define LCD_D3 PA8
#define LCD_D4 PA9
#define LCD_D5 PA10
#define LCD_D6 PA11
#define LCD_D7 PA12

// ===== LCD Configuration =====
#define AUTO_OFFSET 0
#define FIXED_XOFF 0
#define CONTRAST 60

#define FONT_SMALL  u8g2_font_5x8_tr
#define FONT_MEDIUM u8g2_font_6x13_tr
#define FONT_LARGE  u8g2_font_9x15B_tr

static const uint8_t LCD_HW_XOFF = (AUTO_OFFSET ? 0 : FIXED_XOFF);
static const uint8_t LCD_UI_XOFF = 0;
static uint8_t LCD_CONTRAST = CONTRAST;

U8G2_ST7565_JLX12864_F_6800 u8g2(
  U8G2_R0,
  LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7,
  LCD_E, LCD_CS, LCD_DC, LCD_RST
);

static void emulateOnOffButton(uint32_t ms);

// ===== DS18B20 temperature sensor =====
#define TEMP_SENSOR_PIN PB7  // Use free pin (examples: PB7, PA3, PC1)
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensors(&oneWire);
DeviceAddress tempDeviceAddress;
#define TEMP_WARN  72.0f
#define TEMP_CRIT  77.0f

static bool tempWarnActive = false;
static bool tempCritTriggered = false;



// Temperature sensor data structure
struct TemperatureData {
  float celsius = -127.0f;
  float fahrenheit = -196.6f;
  uint32_t lastUpdate = 0;
  bool sensorFound = false;
  char addressStr[17] = "Not found";
} tempData;

static void initTemperatureSensor() {
  Serial.print("Initializing DS18B20 on pin ");
  Serial.println(TEMP_SENSOR_PIN);
  
  tempSensors.begin();
  
  if (tempSensors.getDeviceCount() == 0) {
    Serial.println("ERROR: No DS18B20 sensors found!");
    tempData.sensorFound = false;
    return;
  }
  
  Serial.print("Found ");
  Serial.print(tempSensors.getDeviceCount());
  Serial.println(" DS18B20 sensor(s)");
  
  if (tempSensors.getAddress(tempDeviceAddress, 0)) {
    tempData.sensorFound = true;
    tempSensors.setResolution(tempDeviceAddress, 12);  // 12-bit (0.0625°C)
    
    // Convert address to hex string
    char addrBuf[17];
    for (uint8_t i = 0; i < 8; i++) {
      sprintf(&addrBuf[i*2], "%02X", tempDeviceAddress[i]);
    }
    addrBuf[16] = '\0';
    strcpy(tempData.addressStr, addrBuf);
    
    Serial.print("Sensor Address: ");
    Serial.println(tempData.addressStr);
    Serial.print("Resolution: ");
    Serial.println(tempSensors.getResolution(tempDeviceAddress));
  }
}

static void updateTemperature() {
  static uint32_t lastTempRequest = 0;
  const uint32_t TEMP_REQUEST_INTERVAL = 5000;  // Request every 5 seconds
  
  uint32_t now = millis();
  
  
  if (!tempData.sensorFound) return;
  
  if (now - lastTempRequest >= TEMP_REQUEST_INTERVAL) {
    lastTempRequest = now;
    tempSensors.requestTemperatures();
    
    // Read temperature (takes ~5s at 12-bit resolution)
    tempData.celsius = tempSensors.getTempC(tempDeviceAddress);
    
    if (tempData.celsius == DEVICE_DISCONNECTED_C) {
      tempData.celsius = -127.0f;
      tempData.fahrenheit = -196.6f;
      Serial.println("DS18B20: Sensor disconnected");
    } else {
      tempData.fahrenheit = tempData.celsius * 9.0f / 5.0f + 32.0f;
      tempData.lastUpdate = now;

      // ===== TEMPERATURE SAFETY WITH HYSTERESIS =====
      // Priority 1: Check for critical temperature
      if (tempData.celsius >= TEMP_CRIT && !tempCritTriggered) {
        tempCritTriggered = true;
        tempWarnActive = false;  // Override warning when critical
        Serial.println("CRITICAL TEMP -> POWER OFF");
        emulateOnOffButton(5000);
      }
      // Priority 2: Recovery from critical (hysteresis: 5°C below threshold)
      else if (tempCritTriggered && tempData.celsius < (TEMP_CRIT - 5.0f)) {
        tempCritTriggered = false;
        Serial.println("Temp restored to safe range");
      }
      // Priority 3: Warning threshold
      else if (tempData.celsius >= TEMP_WARN && !tempCritTriggered) {
        tempWarnActive = true;
      }
      // Priority 4: Recovery from warning (hysteresis: 5°C below threshold)
      else if (tempData.celsius < (TEMP_WARN - 5.0f)) {
        tempWarnActive = false;
      }

      // Debug output (every 10 seconds)
      static uint32_t lastPrint = 0;
      if (now - lastPrint > 10000) {
        lastPrint = now;
        Serial.print("Temp.: ");
        Serial.print(tempData.celsius);
        Serial.print("C | Warn: ");
        Serial.print(tempWarnActive ? "YES" : "NO");
        Serial.print(" | Crit: ");
        Serial.println(tempCritTriggered ? "YES" : "NO");
      }
    }
  }
}




// ================= Backlight (PC12) =================
#define BL_PIN PC12
#define BL_ACTIVE_HIGH 0

static bool blIsOn = true;
static uint32_t lastActivityMs = 0;
static const uint32_t BL_TIMEOUT_MS = 30UL * 1000UL;

// Backlight brightness (0-255). If platform supports PWM, analogWrite() is used.
static uint8_t bl_brightness = 255;
static const uint8_t BL_BRIGHT_STEP = 16; // increment/decrement step

static inline void bl_set(bool on) {
  // Preserve brightness when toggling
  if (on) {
    // Restore previous brightness
    uint8_t b = bl_brightness;
    if (b == 0) b = 255;
    pinMode(BL_PIN, OUTPUT);
#if defined(analogWrite)
    analogWrite(BL_PIN, b);
#else
    if (b > 127) {
#if BL_ACTIVE_HIGH
      digitalWrite(BL_PIN, HIGH);
#else
      digitalWrite(BL_PIN, LOW);
#endif
    } else {
#if BL_ACTIVE_HIGH
      digitalWrite(BL_PIN, LOW);
#else
      digitalWrite(BL_PIN, HIGH);
#endif
    }
#endif
    blIsOn = true;
  } else {
    // Turn off (Hi-Z)
    pinMode(BL_PIN, INPUT); // High-impedance OFF
    blIsOn = false;
  }
}

// Set explicit brightness (0-255). 0 turns backlight off (Hi-Z).
static inline void bl_set_brightness(uint8_t b) {
  bl_brightness = b;
  if (b == 0) {
    pinMode(BL_PIN, INPUT); // Hi-Z OFF
    blIsOn = false;
    return;
  }
  pinMode(BL_PIN, OUTPUT);
#if defined(analogWrite)
  analogWrite(BL_PIN, b);
#else
  if (b > 127) {
#if BL_ACTIVE_HIGH
    digitalWrite(BL_PIN, HIGH);
#else
    digitalWrite(BL_PIN, LOW);
#endif
  } else {
#if BL_ACTIVE_HIGH
    digitalWrite(BL_PIN, LOW);
#else
    digitalWrite(BL_PIN, HIGH);
#endif
  }
#endif
  blIsOn = true;
}

static inline void noteActivity() {
  lastActivityMs = millis();
  if (!blIsOn) bl_set(true);
}

// ================= ON/OFF latch outputs =================
#define PWR_PC3 PC3
#define PWR_PB11 PB11

static bool pwrLatched = false;

static inline void pwr_apply() {
  pinMode(PWR_PC3, OUTPUT);
  pinMode(PWR_PB11, OUTPUT);

  if (pwrLatched) {
    digitalWrite(PWR_PC3, HIGH);
    digitalWrite(PWR_PB11, HIGH);
  } else {
    digitalWrite(PWR_PC3, LOW);
    digitalWrite(PWR_PB11, LOW);
  }
}

// ================= UART link to mower =================
#define MOWER_BAUD 115200
HardwareSerial& MowerUart = Serial2;

// ================= RX-only (TX pin disabled) =================
#ifndef MOWER_UART_TX_PIN
#if defined(PIN_SERIAL2_TX)
#define MOWER_UART_TX_PIN PIN_SERIAL2_TX
#elif defined(SERIAL2_TX)
#define MOWER_UART_TX_PIN SERIAL2_TX
#elif defined(PA2)
#define MOWER_UART_TX_PIN PA2
#else
#define MOWER_UART_TX_PIN -1
#endif
#endif

static uint8_t crc8_sum(const char* s) {
  uint8_t c = 0;
  while (*s) c += (uint8_t)(*s++);
  return c;
}

// ================= Keypad pins =================
static const uint32_t P20 = PA4;
static const uint32_t P21 = PA5;
static const uint32_t P22 = PA6;
static const uint32_t P23 = PA7;
static const uint32_t P24 = PC4;
static const uint32_t P25 = PC5;
static const uint32_t P26 = PB0;
static const uint32_t P27 = PB1;
static const uint32_t P28 = PB2;
static const uint32_t P29 = PB10; // ON/OFF -> GND

static const uint16_t SETTLE_US = 150;     // Time for circuit to settle
static const uint8_t SAMPLE_N = 3;         // Number of samples per read
static const uint8_t SAMPLE_MIN = 2;       // Minimum samples needed to confirm
static const uint16_t DEBOUNCE_MS = 30;    // Debounce delay in milliseconds
static const uint8_t HOLD_SCANS = 2;

#ifndef OUTPUT_OPEN_DRAIN
#define OUTPUT_OPEN_DRAIN OUTPUT
#endif

// ===== AT+Y3 POWER OFF COMMAND =====
static bool powerOffInProgress = false;

static void emulateOnOffButton(uint32_t pressMs) {
  powerOffInProgress = true;

  // Press ON/OFF button (P29 is connected to GND)
  pinMode(PWR_PB11, OUTPUT_OPEN_DRAIN);
  digitalWrite(PWR_PC3, LOW);
  digitalWrite(PWR_PB11, LOW);

  delay(pressMs);  // Hold button pressed

  // MCU does not recover (watchdog will reset)
  while (1) {
    delay(3000);
  }
}



// Safe string copy
static void safe_strncpy(char* dest, const char* src, size_t dest_size) {
  if (dest_size == 0) return;
  size_t i = 0;
  while (i < dest_size - 1 && src[i] != '\0') {
    dest[i] = src[i];
    i++;
  }
  dest[i] = '\0';
}

// Check if pin reads LOW for majority of samples (fast debounce)
static inline bool lowQuorum(uint32_t pin) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < SAMPLE_N; i++) {
    if (digitalRead(pin) == LOW) c++;
    delayMicroseconds(10);  // Very short delay between samples
  }
  return c >= SAMPLE_MIN;
}

static inline void asPullup(uint32_t pin) {
  pinMode(pin, INPUT_PULLUP);
}

static inline void driveLowOD(uint32_t pin) {
  pinMode(pin, OUTPUT_OPEN_DRAIN);
  digitalWrite(pin, LOW);
}

static inline void releasePullup(uint32_t pin) {
  pinMode(pin, INPUT_PULLUP);
}

static bool linkEither(uint32_t A, uint32_t B) {
  asPullup(B);
  driveLowOD(A);
  delayMicroseconds(SETTLE_US);
  bool ab = lowQuorum(B);
  releasePullup(A);

  asPullup(A);
  driveLowOD(B);
  delayMicroseconds(SETTLE_US);
  bool ba = lowQuorum(A);
  releasePullup(B);

  return ab || ba;
}

static bool onoffPressedRaw() {
  asPullup(P29);
  delayMicroseconds(SETTLE_US);
  return lowQuorum(P29);
}

// Key functions
static bool k_start() { return linkEither(P22, P24); }
static bool k_home() { return linkEither(P22, P27); }
static bool k_left() { return linkEither(P23, P24); }
static bool k_ok() { return linkEither(P23, P25); }
static bool k_right() { return linkEither(P23, P26); }
static bool k_1() { return linkEither(P20, P24); }
static bool k_2() { return linkEither(P20, P25); }
static bool k_3() { return linkEither(P20, P26); }
static bool k_4() { return linkEither(P21, P27); }
static bool k_5() { return linkEither(P21, P24); }
static bool k_6() { return linkEither(P21, P25); }
static bool k_7() { return linkEither(P21, P26); }
static bool k_8() { return linkEither(P22, P26); }
static bool k_9() { return linkEither(P22, P25); }
static bool k_0() { return linkEither(P20, P27); }
static bool k_onoff() { return onoffPressedRaw(); }

// ================= Debounce =================
struct KeyDef {
  const char* name;
  bool (*readfn)();
};

static KeyDef KEYS[] = {
  {"start", k_start},
  {"home", k_home},
  {"left", k_left},
  {"ok", k_ok},
  {"right", k_right},
  {"1", k_1},
  {"2", k_2},
  {"3", k_3},
  {"4", k_4},
  {"5", k_5},
  {"6", k_6},
  {"7", k_7},
  {"8", k_8},
  {"9", k_9},
  {"0", k_0},
  {"onoff", k_onoff},
};

static constexpr uint8_t NKEYS = sizeof(KEYS)/sizeof(KEYS[0]);

struct KState {
  bool last = false;
  uint8_t mismatch = 0;
  uint32_t tms = 0;
  uint32_t pressStart = 0;
};

static KState ks[NKEYS];

// ================= Ardumower summary =================
struct MowerSummary {
  float batV = 0;
  float x = 0, y = 0, delta = 0;
  int gpsSol = 0;
  int op = 0;
  int mowIdx = 0;
  float dgpsAge = 0;
  int sensor = 0;
  float targetX = 0, targetY = 0;
  float acc = 0;
  int sv = 0;
  float current = 0;
  int svDgps = 0;
  uint32_t mapCRC = 0;
  float latErr = 0;
  int nextDow = -1;
  int nextHour = 0;
  uint32_t lastRxMs = 0;
  uint32_t lastUpdateMs = 0;
} ms;

static char rxLine[192]; // Reduced buffer size
static uint16_t rxPos = 0;
static bool rxOverflow = false;
static uint32_t rxBytes = 0;
static uint32_t rxLines = 0;
static uint32_t rxOverflowCount = 0;
static uint32_t seenAT = 0;
static uint32_t seenS = 0;
static uint32_t parsedS = 0;
static char lastLine[100] = {0};
static char lastSLine[100] = {0};

// ===== RAW log buffer =====
static constexpr uint8_t RAW_ROWS = 7;
static constexpr uint8_t RAW_COLS = 33;
static char rawBuf[RAW_ROWS][RAW_COLS];
static uint8_t rawHead = 0;

static void rawPushLine(const char* s) {
  if (!s) return;
  safe_strncpy(rawBuf[rawHead], s, RAW_COLS);
  rawHead = (rawHead + 1) % RAW_ROWS;
}

// ================= Improved float -> string =================
static void ftoa_fixed(char* out, size_t outsz, float v, uint8_t prec) {
  if (!out || outsz == 0) return;
  out[0] = 0;

  // Check for NaN/Inf using macros compatible with STM32
  #ifndef isnan
  #define isnan(x) ((x) != (x))
  #endif
  
  #ifndef isinf
  #define isinf(x) (!isnan(x) && isnan((x) - (x)))
  #endif
  
  if (isnan(v)) { safe_strncpy(out, "nan", outsz); return; }
  if (isinf(v)) { safe_strncpy(out, "inf", outsz); return; }

  // Use dtostrf for STM32
  char buf[24];
  dtostrf(v, 0, prec, buf);
  
  // Remove trailing zeros
  if (prec > 0) {
    char* p = buf + strlen(buf) - 1;
    while (p > buf && *p == '0') *p-- = '\0';
    if (p >= buf && *p == '.') *p = '\0';
  }
  
  safe_strncpy(out, buf, outsz);
}

// ================= Parser utilities =================
static void rstrip(char* s) {
  size_t n = strlen(s);
  while (n > 0) {
    char c = s[n - 1];
    if (c == '\r' || c == '\n' || c == ' ' || c == '\t') s[--n] = 0;
    else break;
  }
}

static void stripTrailingCrc8Field(char* s) {
  char* lastComma = strrchr(s, ',');
  if (!lastComma) return;

  char* t = lastComma + 1;
  if (!(t[0] == '0' && (t[1] == 'x' || t[1] == 'X'))) return;

  const char* h = t + 2;
  int hexCount = 0;
  while (*h) {
    if (!isxdigit((unsigned char)*h)) return;
    hexCount++;
    h++;
  }
  if (hexCount < 1 || hexCount > 2) return;

  *lastComma = 0;
}

// ================= 7S Battery calculation (delayed data friendly) =================
typedef struct {
    float v; // cell voltage
    int   p; // percent
} SocPoint;

static const SocPoint soc_table[] = {
    {4.15f,100},{4.10f,95},{4.05f,90},{4.00f,85},
    {3.95f,80},{3.90f,70},{3.85f,60},{3.80f,50},
    {3.75f,40},{3.70f,30},{3.65f,20},{3.60f,15},
    {3.55f,10},{3.50f,5}, {3.45f,3}, {3.40f,2},
    {3.30f,1}
};
static const int SOC_TABLE_N = sizeof(soc_table)/sizeof(soc_table[0]);

// Battery SOC state with filtering (for smooth display)
static float soc_filtered = 0.0f;       // Filtered SOC value
static bool soc_initialized = false;    // True when first valid voltage received
static uint32_t soc_last_update_ms = 0;

// Battery SOC calculation parameters
static const uint32_t SOC_UPDATE_INTERVAL_MS = 500; // Update filter max once per 500ms
static const float SOC_ALPHA = 0.10f;               // Low-pass filter strength (0=off, 1=full)

// ===== Binary Search for SOC Interpolation =====
static float interpolateSoc(float cv) {
    if (cv >= soc_table[0].v) return 100.0f;
    if (cv <= soc_table[SOC_TABLE_N-1].v) return 0.0f;

    int low = 0, high = SOC_TABLE_N - 1;
    while (high - low > 1) {
        int mid = (low + high) / 2;
        if (cv > soc_table[mid].v) high = mid;
        else low = mid;
    }

    float t = (cv - soc_table[high].v) / (soc_table[low].v - soc_table[high].v);
    return soc_table[high].p + t * (soc_table[low].p - soc_table[high].p);
}

// ===== Main Battery Percent Calculation =====
static int batteryPercent(float v) {
    // Return 0 if no valid data received yet
    if (v <= 0.0f) return 0;

    // Quick boundary check (fully charged or empty)
    if (v >= 29.4f) {
        soc_filtered = 100.0f;
        soc_initialized = true;
        return 100;
    }
    if (v <= 23.1f) {
        soc_filtered = 0.0f;
        soc_initialized = true;
        return 0;
    }

    // Calculate per-cell voltage (7S = 7 cells in series)
    float cv = v / 7.0f;

    // Find SOC using linear interpolation from lookup table
    float soc_raw = interpolateSoc(cv);

    // On first valid data, jump directly to calculated SOC (no filtering)
    if (!soc_initialized) {
        soc_filtered = soc_raw;
        soc_initialized = true;
        soc_last_update_ms = millis();
        return (int)(soc_filtered + 0.5f);
    }

    // Apply low-pass filter for smooth SOC display
    uint32_t now = millis();
    if (now - soc_last_update_ms >= SOC_UPDATE_INTERVAL_MS) {
        soc_last_update_ms = now;
        soc_filtered += SOC_ALPHA * (soc_raw - soc_filtered);
        if (soc_filtered > 100.0f) soc_filtered = 100.0f;
        if (soc_filtered < 0.0f) soc_filtered = 0.0f;
    }

    return (int)(soc_filtered + 0.5f);
}




// ===== Ardumower Status Parser (CSV format) =====
static bool parseSummaryCSV(const char* line) {
  if (!line || line[0] != 'S' || line[1] != ',') return false;

  char tmp[192];
  safe_strncpy(tmp, line, sizeof(tmp));
  rstrip(tmp);
  
  stripTrailingCrc8Field(tmp);

  MowerSummary nm{};
  nm.nextDow = -1;

  int field = 0;
  char* token = tmp + 2;
  char* p = token;

  while (true) {
    if (*p == ',' || *p == '\0') {
      char saved = *p;
      *p = 0;

      if (*token) {
        switch (field) {
          case 0:  nm.batV     = atof(token); break;
          case 1:  nm.x        = atof(token); break;
          case 2:  nm.y        = atof(token); break;
          case 3:  nm.delta    = atof(token); break;
          case 4:  nm.gpsSol   = atoi(token); break;
          case 5:  nm.op       = atoi(token); break;
          case 6:  nm.mowIdx   = atoi(token); break;
          case 7:  nm.dgpsAge  = atof(token); break;
          case 8:  nm.sensor   = atoi(token); break;
          case 9:  nm.targetX  = atof(token); break;
          case 10: nm.targetY  = atof(token); break;
          case 11: nm.acc      = atof(token); break;
          case 12: nm.sv       = atoi(token); break;
          case 13: nm.current  = atof(token); break;
          case 14: nm.svDgps   = atoi(token); break;
          case 15: nm.mapCRC   = strtoul(token, nullptr, 0); break;
          case 16: nm.latErr   = atof(token); break;
          case 17: nm.nextDow  = atoi(token); break;
          case 18: nm.nextHour = atoi(token); break;
          default: break;
        }
      }

      field++;

      if (saved == '\0') break;
      token = p + 1;
    }

    p++;
    if (field > 30) break;  // Safety limit to prevent runaway
  }

  if (field < 1) return false;

  nm.lastRxMs = millis();
  nm.lastUpdateMs = millis();
  ms = nm;
  parsedS++;
  return true;
}

static void parseLine(char* line) {
  while (*line == '\r' || *line == '\n' || *line == ' ' || *line == '\t') line++;
  size_t n = strlen(line);
  while (n && (line[n-1] == '\r' || line[n-1] == '\n')) line[--n] = 0;
  if (!n) return;

    // Check for power-off command from mower
  if (strncmp(line, "Y3", 2) == 0) {
    Serial.println("AT+Y3 -> POWER OFF");
    emulateOnOffButton(5000);  // Simulate 5-second button press
    return;
  }

  rawPushLine(line);
  safe_strncpy(lastLine, line, sizeof(lastLine));
  rxLines++;

  if (strncmp(line, "AT+", 3) == 0 || strncmp(line, "AT", 2) == 0) {
    seenAT++;
  }

  char* p = nullptr;
  if (line[0] == 'S' && line[1] == ',') {
    p = line;
  } else {
    p = strstr(line, "S,");
  }

  if (p) {
    seenS++;
    safe_strncpy(lastSLine, p, sizeof(lastSLine));
    parseSummaryCSV(p);
  }
}

static void pollUart() {
  while (MowerUart.available()) {
    int c_int = MowerUart.read();
    if (c_int == -1) continue;
    
    char c = (char)c_int;
    rxBytes++;

    if (c == '\n' || c == '\r') {
      if (rxPos && !rxOverflow) {
        rxLine[rxPos] = 0;
        parseLine(rxLine);
      }
      if (rxOverflow) rxOverflowCount++;
      rxPos = 0;
      rxOverflow = false;
      continue;
    }

    if (rxOverflow) continue;

    if (rxPos < sizeof(rxLine)-1) {
      rxLine[rxPos++] = c;
    } else {
      rxOverflow = true;
    }
  }
}

// ===== Battery Bar Drawing (legacy, currently unused) =====
static void drawBatteryBar(int x, int y, int pct) {
  int bars = pct / 20;
  for (int i = 0; i < 5; i++) {
    if (i < bars)
      u8g2.drawBox(x + i*4, y, 3, 6);
    else
      u8g2.drawFrame(x + i*4, y, 3, 6);
  }
}

// Forward declarations
static const char* opNameLower(int op);

// GPS solution type names (based on ArduMower RTKLIB values)
static const char* gpsNameLower(int sol) {
  // Solution types:
  //   0 = NO SOLUTION (no fix)
  //   1 = FLOAT (RTK float solution)
  //   2 = FIX (RTK fixed solution)
 
  
  switch (sol) {
    case 0: return "no";
    case 1: return "flt";
    case 2: return "fix";
   
    default: {
      // Unknown solution type: show numeric value
      static char buf[8];
      snprintf(buf, sizeof(buf), "%d", sol);
      return buf;
    }
  }
}

// ===== Temperature Sensor Display Page =====
static void drawTemperaturePage() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_MEDIUM);
  
  // Page title
  u8g2.drawStr(0, 12, "DS18B20");
  u8g2.drawHLine(0, 14, 128);
  
  // Temperature display
  if (!tempData.sensorFound) {
    u8g2.drawStr(0, 32, "Sensor not found!");
  } 
  else if (tempData.celsius < -100.0f) {
    u8g2.drawStr(0, 32, "Temp: --.-");
    u8g2.drawStr(0, 48, "Sensor error");
  } 
  else {
    char tempStr[16];
    dtostrf(tempData.celsius, 5, 1, tempStr);  // Format: " 23.4"

    // Draw "Temp:" label
    u8g2.drawStr(0, 32, "Temp:");

    // Calculate position for temperature value
    int xVal = u8g2.getStrWidth("Temp: ") + 2;
    int yVal = 32;

    // Draw temperature value
    u8g2.drawStr(xVal, yVal, tempStr);
    int tempW = u8g2.getStrWidth(tempStr);

    // Draw small degree symbol
    int xDeg = xVal + tempW;
    int yDeg = yVal - 7;  // Raised above baseline
    u8g2.drawCircle(xDeg + 2, yDeg, 1, U8G2_DRAW_ALL);

    // Draw "C" unit
    u8g2.drawStr(xDeg + 5, yVal, "C");
  }

  u8g2.sendBuffer();
}


// ================= Screen Settings Page =================
// Settings: simple local time variables (editable in Settings page)
static int settingsHour = 0;
static int settingsMinute = 0;

static void drawSettingsPage() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_MEDIUM);

  // Title
  u8g2.drawStr(0, 12, "SETTINGS");
  u8g2.drawHLine(0, 14, 128);

  // Contrast display and bar
  u8g2.setFont(FONT_SMALL);
  u8g2.drawStr(0, 28, "Contrast");
  
  char pctStr[6];
  snprintf(pctStr, sizeof(pctStr), "%d%%", (LCD_CONTRAST * 100) / 255);
  
  const int barX = 0, barY = 32, barW = 50, barH = 7;
  u8g2.drawFrame(barX, barY, barW, barH);
  int cFill = (LCD_CONTRAST * (barW - 2)) / 255;
  if (cFill > 0) u8g2.drawBox(barX + 1, barY + 1, cFill, barH - 2);
  
  
  
  
  
  u8g2.drawStr(0, 62, "Keys: 8/2 adjust");

  u8g2.sendBuffer();
}




// ===== UI State Variables =====
static uint8_t page = 0;  // Current display page (0-6)
static uint32_t lastDrawMs = 0;
static uint8_t last_bat_percent = 255;
static int last_gps_sol = -1;
static float last_acc = -1.0f;

// Settings: simple local time variables (editable in Settings page)

static void drawStr(uint8_t x, uint8_t y, const char* s) {
  u8g2.drawStr(LCD_UI_XOFF + x, y, s);
}

static float distMeters(float x1, float y1, float x2, float y2) {
  const float dx = x2 - x1;
  const float dy = y2 - y1;
  return sqrtf(dx*dx + dy*dy);
}

static int bearingDeg(float x1, float y1, float x2, float y2) {
  const float dx = x2 - x1;
  const float dy = y2 - y1;
  float a = atan2f(dx, dy) * 180.0f / 3.14159265358979323846f;
  int deg = (int)lroundf(a);
  deg %= 360;
  if (deg < 0) deg += 360;
  return deg;
}

static int angDiffDeg(int a, int b) {
  int d = a - b;
  while (d > 180) d -= 360;
  while (d < -180) d += 360;
  return d;
}

static const char* opNameLower(int op) {
  switch (op) {
    case 0: return "idle";
    case 1: return "mowing";
    case 2: return "charging";
    case 3: return "dock";
    case 4: return "error";
    default: return "unk";
  }
}

static const char* sensorEventLong(int s) {
  return (s == 0) ? "all ok" : "sensor";
}

// ================= Helper functions for drawStatus =================
static void drawTopBar() {
  u8g2.setFont(FONT_MEDIUM);
  
  char voltStr[8], currStr[8], pctStr[6];
  dtostrf(ms.batV, 4, 1, voltStr);
  // Keep sign so charging shows as negative current
  dtostrf(ms.current, 5, 2, currStr);
  int pct = batteryPercent(ms.batV);
  snprintf(pctStr, sizeof(pctStr), "%d%%", pct);

  const int lcdW = 128;
  const int y = 10;
  const int spacing = 2;
  const int leftMargin = 0;
  const int rightMargin = 1;
  const int iconW = 22;
  const int plusW = 2;
  int textAscent = u8g2.getAscent();
  int textDescent = u8g2.getDescent();
  int textH = textAscent - textDescent;
  int iconH = textH - 2;
  if (iconH < 6) iconH = 6;
  const int iconMargin = 1;

  int voltW = u8g2.getStrWidth(voltStr);
  int voltUnitW = u8g2.getStrWidth("V");
  int currW = u8g2.getStrWidth(currStr);
  int currUnitW = u8g2.getStrWidth("A");
  int pctW = u8g2.getStrWidth(pctStr);

  int xIcon = lcdW - rightMargin - plusW - iconW;
  int xPct = xIcon - spacing - pctW;
  int xVolt = leftMargin;
  int voltEnd = xVolt + voltW + voltUnitW + spacing;
  int avail = xPct - voltEnd;
  int currBlockW = currW + currUnitW;

  int xA;
  if (avail <= 0) {
    xA = voltEnd;
  } else if (avail < currBlockW) {
    xA = xPct - currBlockW;
    if (xA < voltEnd) xA = voltEnd;
  } else {
    xA = voltEnd + (avail - currBlockW) / 2;
  }

  if (xIcon + iconW + plusW > lcdW) {
    xIcon = lcdW - (iconW + plusW) - rightMargin;
    xPct = xIcon - spacing - pctW;
  }
  if (xPct < (voltEnd + currBlockW)) {
    xA = voltEnd;
  }
  if (xA < 0) xA = 0;
  if (xPct < 0) xPct = 0;

  u8g2.drawStr(xVolt, y, voltStr);
  u8g2.drawStr(xVolt + voltW + 1, y, "V");
  u8g2.drawStr(xA, y, currStr);
  u8g2.drawStr(xA + currW + 1, y, "A");
  u8g2.drawStr(xPct, y, pctStr);

  int textMid = (y - textAscent) + (textH / 2);
  int boxY = textMid - (iconH / 2) - 1;
  u8g2.drawFrame(xIcon, boxY, iconW, iconH);
  u8g2.drawBox(xIcon + iconW, boxY + 2, plusW, iconH - 4);

  int fill = (iconW - 2*iconMargin) * pct / 100;
  if (fill > 0) u8g2.drawBox(xIcon + iconMargin, boxY + iconMargin, fill, iconH - 2*iconMargin);
}

static void drawMainOp() {
  u8g2.setFont(FONT_LARGE);
  const char* op = opNameLower(ms.op);
  char opU[16];
  strncpy(opU, op, sizeof(opU) - 1);
  opU[sizeof(opU) - 1] = '\0';
  for (int i = 0; opU[i]; i++) opU[i] = toupper((unsigned char)opU[i]);
  int textW = u8g2.getStrWidth(opU);
  int x = (128 - textW) / 2;
  if (x < 0) x = 0;
  u8g2.drawStr(x, 36, opU);
}

static void drawFooter() {
  u8g2.setFont(FONT_MEDIUM);

  char accStr[8];
  dtostrf(ms.acc, 4, 2, accStr);

  char gpsBuf[32];
  snprintf(gpsBuf, sizeof(gpsBuf), "Sol:%s Acc:%sm", gpsNameLower(ms.gpsSol), accStr);
  u8g2.drawStr(0, 64, gpsBuf);

  char tempStr[6];
  if (tempData.sensorFound && tempData.celsius > -100.0f) {
    int t = (int)round(tempData.celsius);
    snprintf(tempStr, sizeof(tempStr), "%d", t);
  } else {
    strcpy(tempStr, "--");
  }

  int tempW = u8g2.getStrWidth(tempStr);
  int cW = u8g2.getStrWidth("C");
  int xTemp = 128 - tempW - cW - 4;

  u8g2.drawStr(xTemp, 64, tempStr);
  int xDeg = xTemp + tempW;
  int yDeg = 64 - 8;
  u8g2.drawCircle(xDeg + 2, yDeg, 1, U8G2_DRAW_ALL);
  u8g2.drawStr(xTemp + tempW + 4, 64, "C");
}

static void drawTempWarningOverlay() {
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawFrame(14, 28, 100, 12);
  const char *msg = "TEMP HIGH";
  int msgW = u8g2.getStrWidth(msg);
  int msgX = (128 - msgW) / 2;
  u8g2.drawStr(msgX, 38, msg);

  if (tempData.sensorFound && tempData.celsius > -100.0f) {
    int tRounded = (int)roundf(tempData.celsius);
    char tempInfo[16];
    snprintf(tempInfo, sizeof(tempInfo), "%dC", tRounded);
    int tempWidth = u8g2.getStrWidth(tempInfo);
    u8g2.drawStr(128 - tempWidth - 5, 64, tempInfo);
  }
}

// ================= TEMP ALERT SCREEN =================
static void drawTempAlertScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(FONT_LARGE);

  if (tempCritTriggered) {
    u8g2.drawStr(5, 28, "OVERHEAT!");
  } else if (tempWarnActive) {
    u8g2.drawStr(12, 28, "TEMP HIGH");
  } else {
    u8g2.drawStr(20, 28, "TEMP OK");
  }

  u8g2.setFont(FONT_MEDIUM);
  char t[10];
  if (!tempData.sensorFound || tempData.celsius < -100.0f) {
    u8g2.drawStr(40, 52, "--.-");
  } else {
    dtostrf(tempData.celsius, 4, 1, t);
    u8g2.drawStr(40, 52, t);
  }
  u8g2.drawStr(75, 52, "C");

  u8g2.sendBuffer();
}


// ================= MAIN STATUS DISPLAY (refactored) =================
static void drawStatus() {
    u8g2.clearBuffer();

    // Priority 1: Show temperature warning overlay if active (but not critical)
    if (tempWarnActive && !tempCritTriggered) {
        drawTempWarningOverlay();
    } else {
        // Priority 2: Show normal status view
        drawTopBar();
        drawMainOp();
        drawFooter();
    }

    u8g2.sendBuffer();
} 



static void drawGps() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);

  char xb[16], yb[16], txb[16], tyb[16];
  ftoa_fixed(xb, sizeof(xb), ms.x, 2);
  ftoa_fixed(yb, sizeof(yb), ms.y, 2);
  ftoa_fixed(txb, sizeof(txb), ms.targetX, 2);
  ftoa_fixed(tyb, sizeof(tyb), ms.targetY, 2);

  char l1[32];
  snprintf(l1, sizeof(l1), "POS %sE %sN", xb, yb);

  char l2[32];
  snprintf(l2, sizeof(l2), "TGT %sE %sN", txb, tyb);

  char accb[16], latb[16];
  ftoa_fixed(accb, sizeof(accb), ms.acc, 2);
  ftoa_fixed(latb, sizeof(latb), ms.latErr, 2);

  char l3[32];
  if (ms.acc >= 800.0f) 
    snprintf(l3, sizeof(l3), "ACC NO GPS lat %s", latb);
  else 
    snprintf(l3, sizeof(l3), "ACC %sm lat %s", accb, latb);

  drawStr(0, 12, l1);
  drawStr(0, 28, l2);
  drawStr(0, 44, l3);

  // GPS satellites and solution count
  char l4[32];
  snprintf(l4, sizeof(l4), "SAT %d/%d Sol:%s", ms.sv, ms.svDgps, gpsNameLower(ms.gpsSol));
  drawStr(0, 60, l4);

  u8g2.sendBuffer();
}

static void drawControls() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);

  const char* opTxt = opNameLower(ms.op);
  char opBuf[10];
  if (opTxt) snprintf(opBuf, sizeof(opBuf), "%s", opTxt);
  else snprintf(opBuf, sizeof(opBuf), "op%d", ms.op);

  const char* ev = sensorEventLong(ms.sensor);
  char evBuf[20];
  snprintf(evBuf, sizeof(evBuf), "%s", ev ? ev : "unk");

  char l1[32];
  snprintf(l1, sizeof(l1), "OP %s %s", opBuf, evBuf);
  drawStr(0, 12, l1);

  char l2[32];
  if (ms.sensor == 0) 
    snprintf(l2, sizeof(l2), "OK CRC %04X", (unsigned)(ms.mapCRC & 0xFFFF));
  else 
    snprintf(l2, sizeof(l2), "CODE %d CRC %04X", ms.sensor, (unsigned)(ms.mapCRC & 0xFFFF));
  drawStr(0, 24, l2);

  char txb[16], tyb[16];
  ftoa_fixed(txb, sizeof(txb), ms.targetX, 2);
  ftoa_fixed(tyb, sizeof(tyb), ms.targetY, 2);

  char l3[32];
  snprintf(l3, sizeof(l3), "TGT %sE %sN", txb, tyb);
  drawStr(0, 36, l3);

  const float d = distMeters(ms.x, ms.y, ms.targetX, ms.targetY);
  const int brg = bearingDeg(ms.x, ms.y, ms.targetX, ms.targetY);
  const int hdg = (int)lroundf(ms.delta);
  const int err = angDiffDeg(brg, hdg);

  char dbuf[16];
  ftoa_fixed(dbuf, sizeof(dbuf), d, 1);

  char l4[32];
  snprintf(l4, sizeof(l4), "DST %sm BRG %03d", dbuf, brg);
  drawStr(0, 48, l4);

  char l5[32];
  snprintf(l5, sizeof(l5), "HDG %03d ERR %+d", (hdg%360+360)%360, err);
  drawStr(0, 60, l5);

  u8g2.sendBuffer();
}

static void drawDebug() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tr);

  char l1[32];
  snprintf(l1, sizeof(l1), "RX %luB %lu lines",
           (unsigned long)rxBytes, (unsigned long)rxLines);

  char l2[32];
  snprintf(l2, sizeof(l2), "AT %lu S %lu ok %lu",
           (unsigned long)seenAT, (unsigned long)seenS, (unsigned long)parsedS);

  char l3[33];
  if (strlen(lastLine) > 32) {
    strncpy(l3, lastLine, 29);
    l3[29] = '.';
    l3[30] = '.';
    l3[31] = '.';
    l3[32] = 0;
  } else {
    safe_strncpy(l3, lastLine, 33);
  }

  char l4[33];
  if (strlen(lastSLine) > 0) {
    if (strlen(lastSLine) > 32) {
      strncpy(l4, lastSLine, 29);
      l4[29] = '.';
      l4[30] = '.';
      l4[31] = '.';
      l4[32] = 0;
    } else {
      safe_strncpy(l4, lastSLine, 33);
    }
  } else {
    strcpy(l4, "No S line");
  }

  char vb[16], cb[16];
  ftoa_fixed(vb, sizeof(vb), ms.batV, 2);
  ftoa_fixed(cb, sizeof(cb), ms.current, 2);

  char l5[32];
  snprintf(l5, sizeof(l5), "Bat:%sV Cur:%sA", vb, cb);

  char l6[32];
  uint32_t age = (ms.lastRxMs == 0) ? 9999 : (millis() - ms.lastRxMs);
  snprintf(l6, sizeof(l6), "Age:%lums Pg:%d OF:%lu",
           (unsigned long)age, (int)page, (unsigned long)rxOverflowCount);

  drawStr(0, 8, l1);
  drawStr(0, 16, l2);
  drawStr(0, 24, l3);
  drawStr(0, 32, l4);
  drawStr(0, 40, l5);
  drawStr(0, 48, l6);

  // Show GPS solution value for debugging
  char l7[32];
  snprintf(l7, sizeof(l7), "GPS Sol:%d (%s)", ms.gpsSol, gpsNameLower(ms.gpsSol));
  drawStr(0, 56, l7);

  u8g2.sendBuffer();
}

static void drawRaw() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tr);

  for (uint8_t i = 0; i < RAW_ROWS; i++) {
    uint8_t idx = (rawHead + i) % RAW_ROWS;
    drawStr(0, 8 + i * 8, rawBuf[idx]);
  }

  u8g2.sendBuffer();
}

static bool needsRedraw() {
  uint32_t now = millis();
  
  // Force redraw occasionally for data updates
  static uint32_t lastForceRedraw = 0;
  if (now - lastForceRedraw > 1000) {
    lastForceRedraw = now;
    return true;
  }
  
  // Check if critical data changed
  int current_bat = batteryPercent(ms.batV);
  if (current_bat != last_bat_percent) {
    last_bat_percent = current_bat;
    return true;
  }
  if (ms.gpsSol != last_gps_sol) {
    last_gps_sol = ms.gpsSol;
    return true;
  }
  if (fabs(ms.acc - last_acc) > 0.05f) {
    last_acc = ms.acc;
    return true;
  }
  
  // Page switch always redraws
  static uint8_t lastPage = 255;
  if (page != lastPage) {
    lastPage = page;
    return true;
  }
  
  return false;
}

static void redrawIfNeeded() {
  uint32_t now = millis();
  if (now - lastDrawMs < 250) return; // ~4 FPS max
  
  if (needsRedraw()) {
    lastDrawMs = now;
    
    switch (page) {
      case 0: drawStatus(); break;
      case 1: drawGps(); break;
      case 2: drawControls(); break;
      case 3: drawDebug(); break;
      case 4: drawRaw(); break;
      case 5: drawTemperaturePage(); break;
      case 6: drawSettingsPage(); break;
      default: drawStatus(); break;
    }
  }
}

// ================= Key actions =================
static void onKeyPress(const char* name) {
  noteActivity();
  
  // Debug output
  Serial.print("Key: ");
  Serial.println(name);

  if (strcmp(name, "left") == 0) {
    page = (page == 0) ? 6 : (page - 1);  // Now 7 pages (0-6)
  } else if (strcmp(name, "right") == 0) {
    page = (page + 1) % 7;  // Now 7 pages (0-6)
  } else if (strcmp(name, "ok") == 0) {
    page = 0;              // back to STATUS page
  } else if (strcmp(name, "8") == 0) {
    if (LCD_CONTRAST < 250) LCD_CONTRAST += 5;
    u8g2.setContrast(LCD_CONTRAST);
    Serial.print("Contrast: ");
    Serial.println(LCD_CONTRAST);
  } else if (strcmp(name, "2") == 0) {
    if (LCD_CONTRAST > 5) LCD_CONTRAST -= 5;
    u8g2.setContrast(LCD_CONTRAST);
    Serial.print("Contrast: ");
    Serial.println(LCD_CONTRAST);
  } else if (strcmp(name, "4") == 0) {
    // Reserved for future use
  } else if (strcmp(name, "6") == 0) {
    // Reserved for future use
  } else if (strcmp(name, "onoff") == 0) {
    if (!pwrLatched) {
      pwrLatched = true;
      pwr_apply();
      Serial.println("Power latched ON");
    }
  } else if (strcmp(name, "start") == 0) {
    // Could add mower start command here
    Serial.println("Start pressed");
  } else if (strcmp(name, "home") == 0) {
    // Could add go home command here
    Serial.println("Home pressed");
  }
}

static void onKeyRelease(const char* name, uint32_t heldMs) {
  if (strcmp(name, "onoff") == 0 && heldMs > 1500) {
    pwrLatched = false;
    pwr_apply();
    Serial.println("Power latched OFF (long press)");
  } else if (strcmp(name, "onoff") == 0) {
    Serial.println("ON/OFF short press");
  }
}

static void scanKeys() {
  uint32_t t = millis();

  for (uint8_t i = 0; i < NKEYS; i++) {
    bool now = KEYS[i].readfn();

    if (now == ks[i].last) {
      ks[i].mismatch = 0;
      continue;
    }
    if (ks[i].mismatch < 255) ks[i].mismatch++;

    if (ks[i].mismatch >= HOLD_SCANS) {
      if (t - ks[i].tms >= DEBOUNCE_MS) {
        ks[i].tms = t;
        bool prev = ks[i].last;
        ks[i].last = now;
        ks[i].mismatch = 0;

        if (!prev && now) {
          ks[i].pressStart = t;
          onKeyPress(KEYS[i].name);
        } else if (prev && !now) {
          uint32_t held = (ks[i].pressStart == 0) ? 0 : (t - ks[i].pressStart);
          onKeyRelease(KEYS[i].name, held);
          ks[i].pressStart = 0;
        }
      }
    }
  }
}

static void safeInitKeyPins() {
  const uint32_t used[] = { P20, P21, P22, P23, P24, P25, P26, P27, P28, P29 };
  for (uint8_t i = 0; i < sizeof(used)/sizeof(used[0]); i++) {
    pinMode(used[i], INPUT_PULLUP);
  }
}

// ================= setup/loop =================
void setup() {
  Serial.begin(115200);
  Serial.println("Mower LCD Controller");
  Serial.println("7S Battery, Correct GPS Values, DS18B20 Support");


  // Setup LCD
  pinMode(LCD_RW, OUTPUT);
  digitalWrite(LCD_RW, LOW);

  u8g2.begin();
  u8g2.getU8x8()->x_offset = LCD_HW_XOFF;
  u8g2.setContrast(LCD_CONTRAST);

  u8g2.clearBuffer();
  u8g2.sendBuffer();

  // DS18B20 temperature sensor initialization
  initTemperatureSensor();

  // Backlight
  bl_set(true);
  noteActivity();

  // Power latch
  pwrLatched = false;
  pwr_apply();

  // Keypad
  safeInitKeyPins();
  memset(ks, 0, sizeof(ks));

  // Clear raw buffer
  for (uint8_t i = 0; i < RAW_ROWS; i++) rawBuf[i][0] = 0;
  rawHead = 0;

  // UART (RX only)
  MowerUart.begin(MOWER_BAUD);
#if (MOWER_UART_TX_PIN != -1)
  pinMode(MOWER_UART_TX_PIN, INPUT_PULLUP);
#endif

  
  
  Serial.println("Setup complete");
}

void loop() {

  // Main tasks
  pollUart();
  scanKeys();
  updateTemperature();  // Update temperature
  
  // Check for parser issues
  static uint32_t lastParserCheck = 0;
  uint32_t now = millis();
  if (now - lastParserCheck > 5000) {
    lastParserCheck = now;
    if (seenS > 0 && parsedS == 0) {
      Serial.print("Parser issue: seenS=");
      Serial.print(seenS);
      Serial.print(" parsedS=");
      Serial.print(parsedS);
      Serial.print(" lastSLine=");
      Serial.println(lastSLine);
    }
  }
  
  // Debug GPS values occasionally
  static uint32_t lastGpsDebug = 0;
  if (now - lastGpsDebug > 10000) {
    lastGpsDebug = now;
    if (ms.gpsSol >= 0) {
      Serial.print("GPS Solution: ");
      Serial.print(ms.gpsSol);
      Serial.print(" (");
      Serial.print(gpsNameLower(ms.gpsSol));
      Serial.println(")");
    }
  }
  
  // Backlight timeout
  if (blIsOn && (now - lastActivityMs >= BL_TIMEOUT_MS)) {
    bl_set(false);
  }
  
  // Redraw display if needed
  redrawIfNeeded();
  
  // Small delay to prevent watchdog timeout
  delay(1);
}