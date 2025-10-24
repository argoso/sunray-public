#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "config.h"

// -----------------------------------------------
HardwareSerial SerialGPS(1);  
HardwareSerial SerialSTM(2);  

WiFiClient ntripClient;
bool wifiConnected = false;
bool ntripConnected = false;
bool apModeActive = false;
unsigned long lastReconnectAttempt = 0;
unsigned long lastNTRIPData = 0;

// LED staatuse muutujad
unsigned long lastBlink = 0;
bool ledState = LOW;
int currentFixStatus = 0; // 0=no fix, 1=float, 2=fix
int connectionMode = 0; // 0=connecting, 1=wifi, 2=ap

// Uued muutujad LED taski jaoks
String ggaForLED = "";
unsigned long lastGGATime = 0;
unsigned long lastLEDUpdate = 0;

void rebootGPS();
void scheduleESP32Reboot();
void rebootAll();
void checkScheduledReboot();

// Base64 kodeerimise funktsioon
String base64_encode(String data) {
  const char* base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String encoded = "";
  int i = 0;
  int j = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];

  int len = data.length();
  while (len--) {
    char_array_3[i++] = data.charAt(j++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for(i = 0; i < 4; i++)
        encoded += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i) {
    for(j = i; j < 3; j++)
      char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;

    for (j = 0; j < i + 1; j++)
      encoded += base64_chars[char_array_4[j]];

    while(i++ < 3)
      encoded += '=';
  }

  return encoded;
}

// NTRIP ühenduse loomine
bool connectNTRIP() {
  if (apModeActive) return false;
  
  Serial.println("Connecting to NTRIP caster...");
  
  if (!ntripClient.connect(NTRIP_HOST, NTRIP_PORT)) {
    Serial.println("Failed to connect to NTRIP server");
    return false;
  }

  // Koosta NTRIP päring
  String auth = "";
  if (strlen(NTRIP_USER) > 0) {
    String credentials = String(NTRIP_USER) + ":" + String(NTRIP_PASS);
    auth = "Authorization: Basic " + base64_encode(credentials) + "\r\n";
  }

  String request = String("GET /") + NTRIP_MOUNT + " HTTP/1.1\r\n" +
                   "Host: " + NTRIP_HOST + "\r\n" +
                   "User-Agent: ESP32NTRIP/1.0\r\n" +
                   auth +
                   "Connection: close\r\n\r\n";

  ntripClient.print(request);

  // Oota vastust
  unsigned long timeout = millis() + 5000;
  while (millis() < timeout && ntripClient.connected() && !ntripClient.available()) {
    delay(10);
  }

  // Kontrolli vastust
  if (ntripClient.available()) {
    String response = ntripClient.readStringUntil('\n');
    if (response.indexOf("200") > 0 || response.indexOf("ICY 200") >= 0) {
      Serial.println("NTRIP connected successfully");
      // Tühjenda puhver
      while (ntripClient.available()) {
        ntripClient.read();
      }
      lastNTRIPData = millis();
      return true;
    } else {
      Serial.println("NTRIP response: " + response);
    }
  }
  
  ntripClient.stop();
  return false;
}

// WiFi ühenduse loomine
bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    apModeActive = false;
    connectionMode = 1;
    Serial.println("\nWiFi connected!");
    Serial.println("IP address: " + WiFi.localIP().toString());
    return true;
  } else {
    wifiConnected = false;
    Serial.println("\nWiFi connection failed!");
    return false;
  }
}

// AP režiimi käivitamine
void startAPMode() {
  Serial.println("Starting AP mode...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  
  apModeActive = true;
  connectionMode = 2;
  wifiConnected = false;
  ntripConnected = false;
  
  Serial.println("AP mode started");
  Serial.println("AP SSID: " + String(AP_SSID));
  Serial.println("AP IP address: " + WiFi.softAPIP().toString());
}

// RTK fix kontroll GGA sõnumist - kiire versioon
void checkRTKLED(const String &nmea) {
  // Kiire kontroll - otsi 6. välja (fix tüüp)
  int commaCount = 0;
  for (int i = 0; i < nmea.length(); i++) {
    if (nmea[i] == ',') {
      commaCount++;
      if (commaCount == 6 && (i + 1) < nmea.length()) {
        char fixType = nmea[i + 1];
        if (fixType == '4') {        // RTK Fixed
          currentFixStatus = 2;
        } else if (fixType == '5') { // RTK Float  
          currentFixStatus = 1;
        } else {                     // No RTK
          currentFixStatus = 0;
        }
        break;
      }
    }
  }
}

// LED haldamine
void updateLED() {
  unsigned long currentTime = millis();
  
  // Ühenduse oleku indikatsioon
  if (connectionMode == 0) { // Ühendumine
    if (currentTime - lastBlink >= 100) { // Kiire vilkumine
      ledState = !ledState;
      digitalWrite(ESP_LED_PIN, ledState);
      lastBlink = currentTime;
    }
  }
  else if (connectionMode == 2) { // AP režiim
    if (currentTime - lastBlink >= 1000) { // Aeglane vilkumine
      ledState = !ledState;
      digitalWrite(ESP_LED_PIN, ledState);
      lastBlink = currentTime;
    }
  }
  else { // WiFi ühendus - RTK olek
    switch (currentFixStatus) {
      case 0: // No fix - LED väljas
        digitalWrite(ESP_LED_PIN, LOW);
        break;
        
      case 1: // Float - LED vilgub (500ms tsükkel)
        if (currentTime - lastBlink >= 500) {
          ledState = !ledState;
          digitalWrite(ESP_LED_PIN, ledState);
          lastBlink = currentTime;
        }
        break;
        
      case 2: // Fixed - LED põleb
        digitalWrite(ESP_LED_PIN, HIGH);
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n=== ESP32 LC29H NTRIP Forwarder ===");

  // Serial seadistus
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  SerialSTM.begin(STM_BAUD, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);

  // LED seadistus
  pinMode(ESP_LED_PIN, OUTPUT);
  digitalWrite(ESP_LED_PIN, LOW);

  // Ühenduse loomine
  connectionMode = 0; // Ühendumise režiim
  if (!connectWiFi()) {
    startAPMode();
  }

  lastReconnectAttempt = millis();
}

// Kuula STM32 käske (mitte-blokeeriv)
void handleSTMCommands() {
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
      
      // Kontrolli erinevaid käske STM32-lt
      if (stmCommandBuffer == "REBOOT_GPS") {
        Serial.println("Received REBOOT_GPS command from STM32");
        rebootGPS();
      } else if (stmCommandBuffer == "REBOOT_ESP32") {
        Serial.println("Received REBOOT_ESP32 command from STM32");
        scheduleESP32Reboot();
      } else if (stmCommandBuffer == "REBOOT_ALL") {
        Serial.println("Received REBOOT_ALL command from STM32");
        rebootAll();
      }
      
      stmCommandBuffer = "";
    }
  }
  
  // Puhasta puhver kui liiga pikk või aegunud
  if (stmCommandBuffer.length() > 100 || 
      (stmCommandBuffer.length() > 0 && millis() - lastCommandTime > 1000)) {
    stmCommandBuffer = "";
  }
}

// Rebooti GPS moodul
void rebootGPS() {
  Serial.println("Rebooting GPS module...");
  SerialGPS.println("$PAIR023*3B");  // Saada LC29H reboot käsk
  SerialGPS.flush();
  Serial.println("GPS reboot command sent to LC29H");
}

// Plaanista ESP32 reboot (2 sekundi pärast)
void scheduleESP32Reboot() {
  Serial.println("Scheduling ESP32 reboot in 2 seconds...");
  static unsigned long rebootTime = 0;
  rebootTime = millis() + 2000;
  
  // Kontrolli rebooti aega põhitsüklis
  // Rebooti tegelikult execute'itakse loop() funktsioonis
}

// Rebooti kogu süsteem (GPS ja ESP32)
void rebootAll() {
  Serial.println("Rebooting entire system...");
  rebootGPS();  // Rebooti GPS esmalt
  scheduleESP32Reboot();  // Seejärel ESP32
}

// Rebooti kontroll põhitsüklis
void checkScheduledReboot() {
  static unsigned long rebootTime = 0;
  
  if (rebootTime > 0 && millis() >= rebootTime) {
    Serial.println("Executing ESP32 restart...");
    delay(100);
    ESP.restart();
  }
}

void loop() {
  // Halda WiFi ühendust
  if (!apModeActive && WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    ntripConnected = false;
    if (millis() - lastReconnectAttempt > RECONNECT_INTERVAL) {
      if (!connectWiFi()) {
        startAPMode();
      }
      lastReconnectAttempt = millis();
    }
  } else if (!wifiConnected && !apModeActive) {
    wifiConnected = true;
    connectionMode = 1;
    Serial.println("WiFi reconnected");
  }

  // Halda NTRIP ühendust (ainult WiFi režiimis)
  if (USE_NTRIP && wifiConnected && !apModeActive) {
    if (!ntripConnected) {
      if (millis() - lastReconnectAttempt > RECONNECT_INTERVAL) {
        ntripConnected = connectNTRIP();
        lastReconnectAttempt = millis();
      }
    } else {
      // Kontrolli NTRIP ühendust
      if (!ntripClient.connected()) {
        ntripConnected = false;
        Serial.println("NTRIP connection lost");
      } else {
        // Loe RTCM andmeid NTRIP-st ja saada GPS-le
        while (ntripClient.available()) {
          uint8_t rtcmData = ntripClient.read();
          SerialGPS.write(rtcmData);
          lastNTRIPData = millis();
        }
        
        // Kontrolli, kas NTRIP andmevoog on katkenud
        if (millis() - lastNTRIPData > NTRIP_TIMEOUT) {
          Serial.println("NTRIP data timeout");
          ntripClient.stop();
          ntripConnected = false;
        }
      }
    }
  }

  // Loe GPS andmeid ja saada STM-le (BLOKKEERIMATA)
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    SerialSTM.write(c);  // SAADA KOHE STM-LE
    
    // LED jaoks kogu GGA sõnumeid (ERALDI puhvrisse)
    static String ledLine = "";
    if (c == '\n') {
      if (ledLine.startsWith("$GNGGA") || ledLine.startsWith("$GPGGA")) {
        ggaForLED = ledLine;  // Salvesta GGA sõnum LED kontrolliks
        lastGGATime = millis();
      }
      ledLine = "";
    } else if (c != '\r') {
      if (ledLine.length() < 100) {  // Piirata pikkust, et vältida mälutäitust
        ledLine += c;
      }
    }
  }

    // Kuula STM32 käske (mitte-blokeeriv)
  handleSTMCommands();

  // LED uuendamine (iga 100ms - ERALDI task)
  if (millis() - lastLEDUpdate > 100) {
    lastLEDUpdate = millis();
    
    if (ggaForLED != "" && millis() - lastGGATime < 2000) {
      checkRTKLED(ggaForLED);
    }
    updateLED();
  }
  
  // Kontrolli plaanitud reboote
  checkScheduledReboot();

  // Väike viivitus tsükli jaoks
  delay(1);
}