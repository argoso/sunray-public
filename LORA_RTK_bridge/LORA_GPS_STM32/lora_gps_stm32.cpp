#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "config.h"

// ---------------------------------------------------------
// Serialid
// ---------------------------------------------------------
HardwareSerial SerialGPS(1);   // LC29H GPS
HardwareSerial SerialSTM(2);   // STM32

// ---------------------------------------------------------
// OLED
// ---------------------------------------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET   16

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static bool oledOk          = false;
static bool oledOn          = true;
static unsigned long oledStartMillis = 0;

// ---------------------------------------------------------
// LED / FIX olek
// ---------------------------------------------------------
String ggaForLED = "";
static unsigned long lastGGATime     = 0;
static unsigned long lastLEDUpdate   = 0;
static unsigned long lastBlinkToggle = 0;
static bool blinkState = false;

enum FixState {
  FIX_NONE = 0,
  FIX_FLOAT,
  FIX_FIXED
};

static FixState currentFixState = FIX_NONE;

// ---------------------------------------------------------
// Reboot ajastus
// ---------------------------------------------------------
static unsigned long pendingRebootTime = 0;

// ---------------------------------------------------------
// LoRa statistika
// ---------------------------------------------------------
unsigned long packetCount      = 0;
unsigned long lastPacketMillis = 0;
int           lastRSSI         = 0;
static unsigned long lastLoRaCheck = 0;
static bool loraHasActivity = false;

// ---------------------------------------------------------
// RTCM statistika
// ---------------------------------------------------------
static unsigned long lastGoodRtcmTime = 0;
static unsigned long rtcmPacketCount = 0;
static unsigned long rtcmCrcErrors = 0;

// ---------------------------------------------------------
// RTCM RX stream buffer (LoRa -> GPS)
// ---------------------------------------------------------
static const int RX_STREAM_BUF_SIZE = 2048;
static uint8_t  rxStreamBuf[RX_STREAM_BUF_SIZE];
static int      rxStreamLen = 0;

// =========================================================
// Reboot
// =========================================================
static void scheduleESP32Reboot() {
  pendingRebootTime = millis() + 2000UL;
}

static void checkScheduledReboot() {
  if (pendingRebootTime != 0 && millis() >= pendingRebootTime) {
    ESP.restart();
  }
}

static void rebootGPS() {
  Serial.println("Rebooting GPS module.");
  SerialGPS.println("$PAIR023*3B");
  SerialGPS.flush();
  Serial.println("GPS reboot command sent to LC29H");
}

static void rebootAll() {
  Serial.println("Rebooting entire system.");
  rebootGPS();
  scheduleESP32Reboot();
}

// =========================================================
// LED loogika (GGA fix quality)
// =========================================================
static void checkRTKLED(const String &ggaLine) {
  int lastPos = 0;
  int nextPos = 0;
  int fixQualityVal = 0;

  // Väljad: 0=$G?GGA, 1=aeg, 2=lat, 3=N/S, 4=lon, 5=E/W, 6=fix quality
  for (int fieldIdx = 0; fieldIdx <= 6; fieldIdx++) {
    nextPos = ggaLine.indexOf(',', lastPos);
    String field;
    if (nextPos == -1) {
      field = ggaLine.substring(lastPos);
    } else {
      field = ggaLine.substring(lastPos, nextPos);
    }

    if (fieldIdx == 6) {
      fixQualityVal = field.toInt();
      break;
    }

    if (nextPos == -1) break;
    lastPos = nextPos + 1;
  }

  FixState newState = FIX_NONE;
  if (fixQualityVal == 4)      newState = FIX_FIXED;
  else if (fixQualityVal == 5) newState = FIX_FLOAT;
  else                         newState = FIX_NONE;

  currentFixState = newState;
}

static void updateLED() {
  unsigned long now = millis();

  // FIXED -> LED pidevalt põleb
  if (currentFixState == FIX_FIXED) {
    digitalWrite(ESP_LED_PIN, HIGH);
    return;
  }

  // FLOAT -> kiire vilkumine, muu -> aeglane vilkumine
  unsigned long period = (currentFixState == FIX_FLOAT) ? 250UL : 1000UL;

  if (now - lastBlinkToggle >= period) {
    lastBlinkToggle = now;
    blinkState = !blinkState;
    digitalWrite(ESP_LED_PIN, blinkState ? HIGH : LOW);
  }
}

// =========================================================
// OLED
// =========================================================
static void updateOLED() {
  if (!oledOk || !oledOn) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("LoRa RX RAW");

  display.setCursor(0, 10);
  display.print("RSSI: ");
  display.print(lastRSSI);
  display.println(" dBm");

  display.setCursor(0, 20);
  display.print("Pkts: ");
  display.println(packetCount);

  display.setCursor(0, 30);
  display.print("FIX : ");
  if (currentFixState == FIX_FIXED)      display.print("FIXED");
  else if (currentFixState == FIX_FLOAT) display.print("FLOAT");
  else                                   display.print("NONE");

  unsigned long age;
  if (rtcmPacketCount == 0 || lastGoodRtcmTime == 0) {
  age = 9999;  // pole veel ühtegi kehtivat RTCM-i saanud
  } else {
  age = (millis() - lastGoodRtcmTime) / 1000;
 }

display.setCursor(0, 40);
display.print("AGE : ");
display.print(age);
display.print("s");


  display.display();
}

// =========================================================
// GPS -> STM32: KÕIK baitid edasi + GGA LED jaoks
// =========================================================
static void pumpGPSToSTM() {
  static String ledLine;

  while (SerialGPS.available() > 0) {
    char c = (char)SerialGPS.read();

    // 1) KÕIK GPSilt tulev info otse STM-i.
    SerialSTM.write((uint8_t)c);

    // 2) Kõrval "sniff", et GGA põhjal LED-i olekut uuendada.
    if (c == '\n') {
      if (ledLine.startsWith("$GNGGA") || ledLine.startsWith("$GPGGA")) {
        ggaForLED   = ledLine;
        lastGGATime = millis();
        checkRTKLED(ggaForLED);
      }
      ledLine = "";
    } else if (c != '\r') {
      if (ledLine.length() < 200) {
        ledLine += c;
      } else {
        // kui midagi läheb täiesti lappesse, ärme lase Stringul lõpmatusse kasvada
        ledLine = "";
      }
    }
  }
}

// =========================================================
// STM32 -> ESP32 käsud (REBOOT_...)
// =========================================================
static void pumpSTMToESP() {
  static String stmCommandBuffer = "";
  static unsigned long lastCommandTime = 0;

  while (SerialSTM.available()) {
    char c = SerialSTM.read();

    if (c == '\n' || c == '\r' || (c >= 32 && c <= 126)) {
      stmCommandBuffer += c;
      lastCommandTime = millis();
    }

    if (c == '\n') {
      stmCommandBuffer.trim();

      if (stmCommandBuffer == "REBOOT_GPS") {
        Serial.println("Received REBOOT_GPS command from STM32");
        rebootAll();
      } else if (stmCommandBuffer == "REBOOT_ESP32") {
        Serial.println("Received REBOOT_ESP32 command from STM32");
        scheduleESP32Reboot();
      }
      stmCommandBuffer = "";
    }
  }

  // Kaitse: kui buffer jääb ripakile liiga kauaks või liiga pikaks, nulli.
  if (stmCommandBuffer.length() > 100 ||
      (stmCommandBuffer.length() > 0 && millis() - lastCommandTime > 1000)) {
    stmCommandBuffer = "";
  }
}

// =========================================================
// LoRa init
// =========================================================
static void setupLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ_HZ)) {
    Serial.println("LoRa init FAIL");
    while (true) delay(1000);
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW_HZ);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_TX_POWER);

  Serial.println("LoRa radio OK (RX, LoRa.h)");
}

// =========================================================
// RTCM CRC24Q (RTCM3 standardne kontrollsumma)
// =========================================================
static uint32_t rtcm3_crc24(const uint8_t *data, int len) {
  // len = header+payload (ilma CRC 3 baitita)
  uint32_t crc = 0;
  for (int i = 0; i < len; ++i) {
    crc ^= (uint32_t)data[i] << 16;
    for (int bit = 0; bit < 8; ++bit) {
      crc <<= 1;
      if (crc & 0x1000000) {
        crc ^= 0x1864CFB;  // polü 0x1864CFB
      }
    }
  }
  return crc & 0xFFFFFF;
}

// =========================================================
// RTCM3 frame parser LoRa RX-le
// =========================================================
static void processRtcmRxStream() {
  const int MAX_FRAMES_PER_CALL = 8;
  int framesProcessed = 0;

  while (true) {
    if (framesProcessed >= MAX_FRAMES_PER_CALL) return;

    // vaja vähemalt D3 + 2 pikkuse baiti
    if (rxStreamLen < 3) return;

    // otsi 0xD3 algust
    int startIndex = 0;
    while (startIndex < rxStreamLen && rxStreamBuf[startIndex] != 0xD3) {
      startIndex++;
    }

    if (startIndex > 0) {
      // viska D3-eelsed prahibaitid minema
      int remaining = rxStreamLen - startIndex;
      if (remaining > 0) {
        memmove(rxStreamBuf, rxStreamBuf + startIndex, remaining);
      }
      rxStreamLen = remaining;
      if (rxStreamLen < 3) return;
    }

    // peab algama D3-ga
    if (rxStreamBuf[0] != 0xD3) {
      // midagi on nihkes -> liigume ühe baidi võrra edasi
      memmove(rxStreamBuf, rxStreamBuf + 1, rxStreamLen - 1);
      rxStreamLen -= 1;
      continue;
    }

    // RTCM3 pikkus (10 bitti payloadile)
    uint16_t payloadLen = ((rxStreamBuf[1] & 0x03) << 8) | rxStreamBuf[2];
    int totalLen = 3 + payloadLen + 3;  // header (3) + payload + CRC(3)

    // sanity check
    if (totalLen <= 0 || totalLen > RX_STREAM_BUF_SIZE) {
      // midagi väga valesti -> liigume ühe baidi võrra edasi ja proovime uuesti
      memmove(rxStreamBuf, rxStreamBuf + 1, rxStreamLen - 1);
      rxStreamLen -= 1;
      continue;
    }

    if (rxStreamLen < totalLen) {
      // frame pole veel täielik, ootame uusi LoRa pakette
      return;
    }

    // TÄIS RTCM frame olemas -> CRC kontroll
    uint32_t crcCalc = rtcm3_crc24(rxStreamBuf, 3 + payloadLen);
    uint32_t crcMsg =
      ((uint32_t)rxStreamBuf[3 + payloadLen] << 16) |
      ((uint32_t)rxStreamBuf[3 + payloadLen + 1] << 8) |
      (uint32_t)rxStreamBuf[3 + payloadLen + 2];

    if (crcCalc == crcMsg) {
      // CRC OK -> SAADA GPS-ile
      SerialGPS.write(rxStreamBuf, totalLen);
      lastGoodRtcmTime = millis();
      rtcmPacketCount++;
      
      // Debug: Prindi mõned frame'id
      if (rtcmPacketCount % 10 == 0) {
        Serial.print("RTCM frame sent: type=");
        int msgType = (rxStreamBuf[3] << 4) | (rxStreamBuf[4] >> 4);
        Serial.print(msgType);
        Serial.print(", len=");
        Serial.print(totalLen);
        Serial.print(", age=");
        Serial.print((millis() - lastPacketMillis)/1000);
        Serial.println("s");
      }
    } else {
      rtcmCrcErrors++;
      if (rtcmCrcErrors % 5 == 0) {
        Serial.print("RTCM CRC errors: ");
        Serial.println(rtcmCrcErrors);
      }
      // CRC vale -> jätame selle frame'i vahele
    }

    // nihuta allesjäänud voog ette
    int remaining = rxStreamLen - totalLen;
    if (remaining > 0) {
      memmove(rxStreamBuf, rxStreamBuf + totalLen, remaining);
    }
    rxStreamLen = remaining;

    framesProcessed++;
  }
}

// =========================================================
// LoRa RX
// =========================================================
static void pollLoRa() {
  // Väldi liiga tihedat LoRa kontrolli, aga tee seda sagedamini kui varem
  if (millis() - lastLoRaCheck < 5) return;  // ~200 Hz
  lastLoRaCheck = millis();
  
  int packetSize = LoRa.parsePacket();
  if (!packetSize) {
    // Kui pikka aega pole pakette, võib vajada seadeid
    if (loraHasActivity && (millis() - lastPacketMillis > 10000)) {
      Serial.println("WARN: No LoRa packets for 10s");
      loraHasActivity = false;
    }
    return;
  }

  if (packetSize > 255) packetSize = 255;
  uint8_t buf[255];
  int idx = 0;

  while (LoRa.available() && idx < packetSize) {
    buf[idx++] = (uint8_t)LoRa.read();
  }

  if (idx > 0) {
    packetCount++;
    lastPacketMillis = millis();
    lastRSSI = LoRa.packetRssi();
    loraHasActivity = true;

    // RTCM voog -> buffer (rahulikum overflow käsitlus)
    if (idx > RX_STREAM_BUF_SIZE) {
      // üks LoRa pakett suurem kui buffer - ebanormaalne, viskame minema
      return;
    }

    if (rxStreamLen + idx > RX_STREAM_BUF_SIZE) {
      // bufferis pole piisavalt ruumi, lõika eest ära nii palju kui vaja
      int overflow = (rxStreamLen + idx) - RX_STREAM_BUF_SIZE;
      if (overflow < rxStreamLen) {
        memmove(rxStreamBuf, rxStreamBuf + overflow, rxStreamLen - overflow);
        rxStreamLen -= overflow;
      } else {
        rxStreamLen = 0;
      }
    }

    memcpy(rxStreamBuf + rxStreamLen, buf, idx);
    rxStreamLen += idx;

    // Lisa debug info aeg-ajalt
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 5000) {
      lastDebugPrint = millis();
      Serial.print("LoRa RX: ");
      Serial.print(packetCount);
      Serial.print(" pkts, RSSI: ");
      Serial.print(lastRSSI);
      Serial.print(" dBm, Buffer: ");
      Serial.print(rxStreamLen);
      Serial.println(" bytes");
    }
  }
}

// =========================================================
// setup & loop
// =========================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== TTGO ESP32 LoRa RTCM RX RAW (LoRa.h) ===");
  Serial.println("Optimized for faster RTK fix");

  pinMode(ESP_LED_PIN, OUTPUT);
  digitalWrite(ESP_LED_PIN, LOW);

  // SUURENDA bufferid veelgi
  SerialGPS.setRxBufferSize(2048);  // Enamus NMEA jaoks
  SerialGPS.setTxBufferSize(1024);  // RTCM jaoks
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  SerialSTM.setRxBufferSize(512);
  SerialSTM.setTxBufferSize(512);
  SerialSTM.begin(STM_BAUD, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);

  Wire.begin(4, 15);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledOk = true;
    oledStartMillis = millis();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Booting LoRa RX RAW.");
    display.display();
  } else {
    Serial.println("OLED init FAIL");
    oledOk = false;
  }

  setupLoRa();
  lastPacketMillis = millis();
  
  // Oota GPS-initsialiseerimist
  delay(2000);
  
  // GPS on juba valmis RTCM-i vastu võtma (kui eelnevalt seadistatud)
  Serial.println("GPS ready for RTCM");
}

void loop() {
  static unsigned long loopCounter = 0;
  loopCounter++;
  
  // PRIORITEET 1: GPS -> STM (NMEA)
  pumpGPSToSTM();
  
  // PRIORITEET 2: STM käsud
  pumpSTMToESP();
  
  // PRIORITEET 3: LoRa -> RTCM (kutsume igal loopil, sees on oma limiter)
  pollLoRa();
  
  // PRIORITEET 4: RTCM bufferi töötlemine (kui on andmeid)
  if (rxStreamLen > 0) {
    processRtcmRxStream();
  }
  
  // LED uuendus (igal loopil)
  if (millis() - lastLEDUpdate > 50) {
    lastLEDUpdate = millis();
    updateLED();
  }
  
  // OLED uuendus (aeglasemalt)
  static unsigned long lastOLEDUpdate = 0;
  if (oledOk && oledOn && millis() - lastOLEDUpdate > 500) {
    lastOLEDUpdate = millis();
    updateOLED();
  }
  
  // Lisa debug info iga 10 sekundi järel
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 10000) {
    lastStatusPrint = millis();
    Serial.println("\n=== STATUS ===");
    Serial.print("Fix state: ");
    Serial.println(currentFixState);
    Serial.print("LoRa packets: ");
    Serial.println(packetCount);
    Serial.print("RTCM frames sent: ");
    Serial.println(rtcmPacketCount);
    Serial.print("RTCM CRC errors: ");
    Serial.println(rtcmCrcErrors);
    Serial.print("Buffer size: ");
    Serial.println(rxStreamLen);
    Serial.print("Last RTCM: ");
    Serial.print((millis() - lastGoodRtcmTime)/1000);
    Serial.println("s ago");
    Serial.println("=============");
  }
  
  // OLED auto-off
  if (oledOk && oledOn && (millis() - oledStartMillis > OLED_TIMEOUT_MS)) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    oledOn = false;
  }
  
  // Planeeritud reboot
  checkScheduledReboot();
  
  // Väike yield
  delay(1);
}
