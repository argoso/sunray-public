// main.cpp - PARANDATUD VERSIOON (kergelt optimeeritud)
#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "config.h"

// =========== Globaalsed ===========

// WiFi / TCP
WiFiClient client;

bool wifiOk   = false;
bool serverOk = false;

unsigned long lastWifiAttemptMs   = 0;
unsigned long lastServerAttemptMs = 0;
unsigned long lastTcpRxMillis     = 0;
unsigned long lastTcpSendMillis   = 0;  // Viimane LoRa saatmine

// LoRa saatmise vahe kontroll (et RX ei upuks)
unsigned long lastLoRaSendMillis  = 0;

// RTCM voog (TCP → see buffer)
static const int STREAM_BUF_SIZE = 2048;  // SUUREM buffer
uint8_t  streamBuf[STREAM_BUF_SIZE];
int      streamLen = 0;

// statistika
unsigned long framesSent     = 0;   // LoRa pakid (chunkid)
unsigned long bytesSentLoRa  = 0;
unsigned long reconnectCount = 0;
unsigned long rtcmFrameCount = 0;   // Terveid RTCM frame'e

// OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledOn = true;
unsigned long oledStartMillis = 0;

// ---------- OLED uuendus ----------
void updateDisplay() {
  if (!oledOn) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("TX: TCP -> LoRa RTCM");

  display.setCursor(0, 10);
  display.print("WiFi: ");
  display.print(wifiOk ? "OK " : "LOST");
  display.print(" ");
  display.print(WiFi.RSSI());
  display.print("dBm");

  display.setCursor(0, 20);
  display.print("Srv : ");
  if (serverOk) {
    unsigned long age = (millis() - lastTcpRxMillis) / 1000;
    display.print(age);
    display.print("s");
  } else {
    display.print("DISC");
  }

  display.setCursor(0, 30);
  display.print("Frames: ");
  display.print(rtcmFrameCount);

  display.setCursor(0, 40);
  display.print("Pkts: ");
  display.print(framesSent);

  display.setCursor(0, 50);
  display.print("Bytes: ");
  display.println(bytesSentLoRa);

  display.display();
}

// ---------- WiFi ----------
void startWifiConnect() {
  Serial.println("WiFi: alustame ühendust.");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // WiFi võimsus maksimumi
  lastWifiAttemptMs = millis();
}

void pollWifi() {
  wl_status_t st = WiFi.status();
  if (st == WL_CONNECTED) {
    if (!wifiOk) {
      wifiOk = true;
      serverOk = false;
      Serial.print("WiFi OK, IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("RSSI: ");
      Serial.println(WiFi.RSSI());
    }
    return;
  }

  if (wifiOk && st != WL_CONNECTED) {
    wifiOk = false;
    serverOk = false;
    client.stop();
    Serial.println("WiFi kadus, serveri ühendus katkestatud.");
  }

  unsigned long now = millis();
  if (now - lastWifiAttemptMs >= WIFI_RETRY_MS) {
    Serial.println("WiFi retry...");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiAttemptMs = now;
  }
}

// ---------- TCP ----------
void pollServer() {
  if (!wifiOk) {
    serverOk = false;
    client.stop();
    return;
  }

  if (client.connected()) {
    if (!serverOk) {
      serverOk = true;
      lastTcpRxMillis = millis();  // Nulli timeout
      lastTcpSendMillis = millis();
      Serial.println("RTCM/TCP serveriga ühendus olemas.");
    }
    return;
  }

  unsigned long now = millis();
  if (now - lastServerAttemptMs >= SERVER_RETRY_MS) {
    Serial.print("Üritan RTCM serveriga ühendada ");
    Serial.print(RTCM_SERVER_IP);
    Serial.print(":");
    Serial.println(RTCM_SERVER_PORT);

    lastServerAttemptMs = now;

    if (client.connect(RTCM_SERVER_IP, RTCM_SERVER_PORT)) {
      serverOk = true;
      reconnectCount++;
      lastTcpRxMillis = millis();
      lastTcpSendMillis = millis();
      streamLen = 0;  // puhasta RTCM buffer
      Serial.println("RTCM serveriga ühendatud!");
    } else {
      serverOk = false;
      Serial.println("RTCM serveriga ühendus ebaõnnestus.");
      client.stop();
    }
  }
}

// ---------- LoRa init ----------
void setupLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ_HZ)) {
    Serial.println("LoRa init FAIL");
    while (true) delay(1000);
  }

  LoRa.setSpreadingFactor(LORA_SF);         // SF7
  LoRa.setSignalBandwidth(LORA_BW_HZ);      // 250 kHz (SAMA MIS RX-IL)
  LoRa.setCodingRate4(LORA_CR);            // 4/5
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_TX_POWER);
  
  Serial.println("LoRa radio OK (TX, LoRa.h)");
  Serial.print("Freq: "); Serial.print(LORA_FREQ_HZ);
  Serial.print(" BW: "); Serial.print(LORA_BW_HZ);
  Serial.print(" SF: "); Serial.println(LORA_SF);
}

// ---------- RTCM CRC kontroll ----------
static uint32_t rtcm3_crc24(const uint8_t *data, int len) {
  uint32_t crc = 0;
  for (int i = 0; i < len; ++i) {
    crc ^= (uint32_t)data[i] << 16;
    for (int bit = 0; bit < 8; ++bit) {
      crc <<= 1;
      if (crc & 0x1000000) {
        crc ^= 0x1864CFB;
      }
    }
  }
  return crc & 0xFFFFFF;
}

// ---------- RTCM frame -> LoRa paketid ----------
void sendRtcmFrame(const uint8_t* frame, int length) {
  if (length <= 0) return;
  
  // Kontrolli CRC enne saatmist
  uint16_t payloadLen = ((frame[1] & 0x03) << 8) | frame[2];
  uint32_t crcCalc = rtcm3_crc24(frame, 3 + payloadLen);
  uint32_t crcMsg = ((uint32_t)frame[3 + payloadLen] << 16) |
                    ((uint32_t)frame[3 + payloadLen + 1] << 8) |
                    (uint32_t)frame[3 + payloadLen + 2];
  
  if (crcCalc != crcMsg) {
    Serial.println("RTCM CRC ERROR - not sending");
    return;
  }
  
  // Saada LoRa kaudu (paki kaupa)
  int offset = 0;
  while (offset < length) {
    int chunkLen = length - offset;
    if (chunkLen > LORA_PACKET_MAX) {
      chunkLen = LORA_PACKET_MAX;
    }

    // väike vahe pakettide vahel, et RX ei upuks (3 ms, mitte 10 ms)
    const unsigned long LORA_MIN_GAP_MS = 3;
    unsigned long now = millis();
    if (now - lastLoRaSendMillis < LORA_MIN_GAP_MS) {
      unsigned long waitMs = LORA_MIN_GAP_MS - (now - lastLoRaSendMillis);
      if (waitMs > 0 && waitMs < 50) {
        delay(waitMs);
      }
    }

    LoRa.beginPacket();
    LoRa.write(frame + offset, chunkLen);
    if (LoRa.endPacket() == 1) {  // Success
      framesSent++;
      bytesSentLoRa += chunkLen;
      lastTcpSendMillis = millis();
      lastLoRaSendMillis = lastTcpSendMillis;
    } else {
      Serial.println("LoRa send failed");
    }
    
    offset += chunkLen;
  }
  
  rtcmFrameCount++;
  
  // Prindi iga 10. frame info
  if (rtcmFrameCount % 10 == 0) {
    int msgType = (frame[3] << 4) | (frame[4] >> 4);
    Serial.print("Sent RTCM frame #");
    Serial.print(rtcmFrameCount);
    Serial.print(", type=");
    Serial.print(msgType);
    Serial.print(", len=");
    Serial.print(length);
    Serial.println(" bytes");
  }
}

// võtab STREAM bufferist järjest RTCM3 frame'id
void processRtcmStream() {
  static unsigned long lastProcessTime = 0;
  
  // natuke reaktiivsem (3 ms, varem 5 ms)
  if (millis() - lastProcessTime < 3) return;
  lastProcessTime = millis();

  int framesProcessed = 0;
  const int MAX_FRAMES_PER_LOOP = 10;
  
  while (framesProcessed < MAX_FRAMES_PER_LOOP) {
    if (streamLen < 3) return;

    // otsi 0xD3
    int startIndex = 0;
    while (startIndex < streamLen && streamBuf[startIndex] != 0xD3) {
      startIndex++;
    }

    if (startIndex > 0) {
      // viskame müra eest ära
      memmove(streamBuf, streamBuf + startIndex, streamLen - startIndex);
      streamLen -= startIndex;
      if (streamLen < 3) return;
    }

    if (streamBuf[0] != 0xD3) {
      // Ei leia D3 - puhasta buffer aeglaselt
      if (streamLen > 100) {
        memmove(streamBuf, streamBuf + 1, streamLen - 1);
        streamLen -= 1;
        continue;
      }
      return;
    }

    // RTCM3 pikkus (10 bitti)
    uint16_t payloadLen = ((streamBuf[1] & 0x03) << 8) | streamBuf[2];
    int totalLen = 3 + payloadLen + 3;

    // Sanity check
    if (payloadLen == 0 || totalLen <= 3 || totalLen > STREAM_BUF_SIZE) {
      // Vigane - liiguta 1 bait edasi
      memmove(streamBuf, streamBuf + 1, streamLen - 1);
      streamLen -= 1;
      continue;
    }

    if (streamLen < totalLen) {
      // frame pole veel täielik
      return;
    }

    // terve RTCM frame olemas
    sendRtcmFrame(streamBuf, totalLen);

    // tõsta järelejäänud ette
    int remaining = streamLen - totalLen;
    if (remaining > 0) {
      memmove(streamBuf, streamBuf + totalLen, remaining);
    }
    streamLen = remaining;
    
    framesProcessed++;
  }
}

// ---------- TCP -> LoRa ----------
void handleTcpToLoRa() {
  if (!client.connected()) return;

  // TCP → streamBuf (pri 1)
  while (client.available() && streamLen < STREAM_BUF_SIZE) {
    streamBuf[streamLen++] = (uint8_t)client.read();
    lastTcpRxMillis = millis();
  }

  // Streami töötlemine (pri 2)
  if (streamLen > 0) {
    processRtcmStream();
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[BOOT] TX: TCP -> LoRa RTCM");
  Serial.println("Optimized version with CRC check");

  Wire.begin(4, 15);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Start TX LoRa.");
    display.println("TCP->LoRa RTCM");
    display.display();
    oledOn = true;
    oledStartMillis = millis();
  } else {
    Serial.println("OLED init FAIL");
  }

  setupLoRa();
  startWifiConnect();
  lastServerAttemptMs = millis() - SERVER_RETRY_MS;

  updateDisplay();
}

void loop() {
  static unsigned long loopCounter = 0;
  static unsigned long lastStatusPrint = 0;
  loopCounter++;
  
  // PRIORITEET 1: WiFi
  if (loopCounter % 2 == 0) {
    pollWifi();
  }
  
  // PRIORITEET 2: TCP server
  if (loopCounter % 3 == 0) {
    pollServer();
  }
  
  // PRIORITEET 3: TCP -> LoRa (kui ühendus olemas)
  if (wifiOk && serverOk && client.connected()) {
    handleTcpToLoRa();
  }
  
  // PRIORITEET 4: OLED update (aeglasemalt)
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 1000) {
    lastDisplayUpdate = millis();
    updateDisplay();
  }
  
  // TCP timeout - mõistlikum ootamine
  if (serverOk && client.connected()) {
    unsigned long tcpIdle = millis() - lastTcpRxMillis;
    unsigned long loraIdle = millis() - lastTcpSendMillis;
    
    // Kui on pikka aega andmeid, aga LoRa on saanud, siis pole hullu
    if (tcpIdle > TCP_TIMEOUT_MS && loraIdle > TCP_TIMEOUT_MS) {
      Serial.println("TCP & LoRa idle timeout -> reconnect");
      client.stop();
      serverOk = false;
      streamLen = 0;
    }
  }
  
  // OLED auto-off
  if (oledOn && (millis() - oledStartMillis > OLED_TIMEOUT_MS)) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    oledOn = false;
  }
  
  // Print status iga 30 sek
  if (millis() - lastStatusPrint > 30000) {
    lastStatusPrint = millis();
    Serial.println("\n=== TX STATUS ===");
    Serial.print("WiFi: "); Serial.println(wifiOk ? "OK" : "OFF");
    Serial.print("Server: "); Serial.println(serverOk ? "OK" : "OFF");
    Serial.print("RTCM frames: "); Serial.println(rtcmFrameCount);
    Serial.print("LoRa packets: "); Serial.println(framesSent);
    Serial.print("Bytes sent: "); Serial.println(bytesSentLoRa);
    Serial.print("Reconnects: "); Serial.println(reconnectCount);
    Serial.print("Buffer: "); Serial.print(streamLen); Serial.println(" bytes");
    Serial.println("=================");
  }
  
  // Väike yield
  delay(1);
}
