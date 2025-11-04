#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "config.h"

// ---------------------------------------------------------
// UARTid
// ---------------------------------------------------------
HardwareSerial SerialGPS(1);   // GPS (LC29H) külge
HardwareSerial SerialSTM(2);   // robot/STM32 külge

// ---------------------------------------------------------
// NTRIP / WiFi olek
// ---------------------------------------------------------
WiFiClient ntripClient;

static bool wifiConnected      = false;   // STA ühendus casteriga samas võrgus olemas
static bool apModeActive       = false;   // kas kukkusime softAP fallbacki
static bool ntripConnected     = false;   // kas NTRIP on aktiivselt lahti ja RTCM tuleb
static unsigned long lastReconnectAttemptWiFi  = 0;
static unsigned long lastReconnectAttemptNTRIP = 0;
static unsigned long lastNTRIPData             = 0;   // millis() viimane RTCM bait

// ---------------------------------------------------------
// LED oleku jaoks
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


// =========================================================
// Abi: Base64 kooder Basic Auth headeri jaoks
// =========================================================
static String base64Encode(const uint8_t *data, size_t len) {
  static const char table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

  String out;
  out.reserve(((len + 2) / 3) * 4);

  for (size_t i = 0; i < len; i += 3) {
    uint32_t n = ((uint32_t)data[i]) << 16;
    if (i + 1 < len) n |= ((uint32_t)data[i + 1]) << 8;
    if (i + 2 < len) n |= ((uint32_t)data[i + 2]);

    out += table[(n >> 18) & 0x3F];
    out += table[(n >> 12) & 0x3F];
    if (i + 1 < len) {
      out += table[(n >> 6) & 0x3F];
    } else {
      out += '=';
    }
    if (i + 2 < len) {
      out += table[n & 0x3F];
    } else {
      out += '=';
    }
  }
  return out;
}


// =========================================================
// Ajasta ESP32 restart
// =========================================================
static void scheduleESP32Reboot() {
  pendingRebootTime = millis() + 2000UL; // reboot ~2s pärast
}

static void checkScheduledReboot() {
  if (pendingRebootTime != 0 && millis() >= pendingRebootTime) {
    ESP.restart();
  }
}

// Rebooti GPS moodul
static void rebootGPS() {
  Serial.println("Rebooting GPS module...");
  SerialGPS.println("$PAIR023*3B");  // Saada LC29H reboot käsk
  SerialGPS.flush();
  Serial.println("GPS reboot command sent to LC29H");
}

// Rebooti kogu süsteem (GPS ja ESP32)
static void rebootAll() {
  Serial.println("Rebooting entire system...");
  rebootGPS();  // Rebooti GPS esmalt
  scheduleESP32Reboot();  // Seejärel ESP32
}


// =========================================================
// LED oleku funktsioonid
// =========================================================

// GGA fixQuality väli (index 6):
// 0 = no fix, 1 = autonomous, 2 = DGPS, 4 = RTK FIXED, 5 = RTK FLOAT
static void checkRTKLED(const String &ggaLine) {
  int lastPos = 0;
  int nextPos = 0;
  int fixQualityVal = 0;

  // käime komade kaupa, tahame välja #6
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
  if (fixQualityVal == 4) {
    newState = FIX_FIXED;   // RTK Fixed
  } else if (fixQualityVal == 5) {
    newState = FIX_FLOAT;   // RTK Float
  } else {
    newState = FIX_NONE;    // muu / no fix / DGPS / autonomous
  }

  currentFixState = newState;
}

// LED kuvamine:
//  - FIX_FIXED  -> põleb pidevalt HIGH
//  - FIX_FLOAT  -> vilgub kiiresti (ca 4 Hz, ~250 ms periood)
//  - FIX_NONE   -> vilgub aeglaselt (1 Hz, ~1000 ms periood)
static void updateLED() {
  unsigned long now = millis();

  if (currentFixState == FIX_FIXED) {
    digitalWrite(ESP_LED_PIN, HIGH);
    return;
  }

  unsigned long period = 1000; // vaikimisi aeglane vilkumine
  if (currentFixState == FIX_FLOAT) {
    period = 250;
  }

  if (now - lastBlinkToggle >= period) {
    lastBlinkToggle = now;
    blinkState = !blinkState;
    digitalWrite(ESP_LED_PIN, blinkState ? HIGH : LOW);
  }
}


// =========================================================
// WiFi käsitlemine
// =========================================================
static void startAPFallback() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  apModeActive   = true;
  wifiConnected  = false;
  ntripConnected = false;
  Serial.println("Starting AP mode...");
}

static void connectWiFiSTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0 < WIFI_TIMEOUT)) {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected  = true;
    apModeActive   = false;    
    Serial.println("\nWiFi connected!");
    Serial.println("IP address: " + WiFi.localIP().toString());
  } else {
    // kui STA ei saanud ühendust -> softAP fallback
    startAPFallback();
  }
}

// hoiab WiFi elus / üritab STA uuesti
static void ensureWiFi() {
  if (apModeActive) {
    // oleme juba AP-s, ära spämmi iga tsükkel STA retry'd
    return;
  }

  if (wifiConnected && (WiFi.status() == WL_CONNECTED)) {
    // kõik korras
    return;
  }

  // kas on aeg uuesti proovida STA ühendust?
  if (millis() - lastReconnectAttemptWiFi >= RECONNECT_INTERVAL) {
    lastReconnectAttemptWiFi = millis();
    connectWiFiSTA();
  }
}


// =========================================================
// NTRIP käsitlemine
// =========================================================

// Basic Auth header NTRIP casterile
static String makeNTRIPAuthHeader() {
  String creds = String(NTRIP_USER) + ":" + String(NTRIP_PASS);
  String enc   = base64Encode((const uint8_t*)creds.c_str(), creds.length());
  return "Authorization: Basic " + enc + "\r\n";
}

// Loo ühendus casteriga ja saada NTRIP GET päring
static void connectNTRIP() {
  if (!USE_NTRIP) {
    ntripConnected = false;
    return;
  }
  if (!wifiConnected) {
    ntripConnected = false;
    return;
  }

  // sulge vana igaks juhuks
  if (ntripClient.connected()) {
    ntripClient.stop();
  }

  // proovi casteriga ühendada
  if (!ntripClient.connect(NTRIP_HOST, NTRIP_PORT)) {
    ntripConnected = false;
    return;
  }

  // Klassikaline NTRIP v1 päring
  String req  = "GET /";
  req += NTRIP_MOUNT;
  req += " HTTP/1.0\r\n";
  req += "User-Agent: NTRIP ESP32\r\n";
  req += "Accept: */*\r\n";
  req += "Connection: close\r\n";
  req += makeNTRIPAuthHeader();
  req += "\r\n";

  ntripClient.print(req);

  ntripConnected = true;
  Serial.println("NTRIP connected successfully");
  lastNTRIPData  = millis();
}

// Hoia NTRIP elus, reconnecti kui vajab
static void ensureNTRIP() {
  if (!USE_NTRIP) {
    ntripConnected = false;
    return;
  }
  if (!wifiConnected || apModeActive) {
    ntripConnected = false;
    return;
  }

  // ühendus kukkus
  if (!ntripClient.connected()) {
    ntripConnected = false;
    Serial.println("NTRIP connection lost");
  }

  // vajadusel proovi uuesti
  if (!ntripConnected) {
    if (millis() - lastReconnectAttemptNTRIP >= RECONNECT_INTERVAL) {
      lastReconnectAttemptNTRIP = millis();
      connectNTRIP();
    }
    return;
  }

  // kui RTCM voog on vaikinud liiga kaua -> katkesta ja lase uuesti luua
  if (millis() - lastNTRIPData > NTRIP_TIMEOUT) {
    ntripClient.stop();
    ntripConnected = false;
  }
}


// =========================================================
// pumpCasterToGPS()
//   - kui caster (ntripClient) annab RTCM baiti,
//     saadame need otse GPS moodulile (LC29H).
//   - iga loetud bait uuendab lastNTRIPData.
// =========================================================
static void pumpCasterToGPS() {
  if (!ntripConnected) return;

  while (ntripClient.available() > 0) {
    uint8_t b = ntripClient.read();
    SerialGPS.write(b);       // RTCM -> GPS
    lastNTRIPData = millis(); // märgime, et voog elus
  }
}


// =========================================================
// GNSS → STM32 filtreeritud ja LED-i jaoks
//
// Me EI saada STM32-le enam RTCM ega "W-> HEX" spämmi.
// Kogume GPS-ilt ridu, lubame ainult ehtsad NMEA/PQT... laused.
// Kontrollime checksum'i enne saatmist.
//
// Samal ajal nopime GGA-st fixQuality ja uuendame LED seisu.
// =========================================================

// heksmärgi väärtuseks int
static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return -1;
}

// kas rida on kehtiv NMEA/PQT ja checksum klapib?
static bool isValidNMEALine(const String &line) {
  if (line.length() < 6) return false;
  if (line[0] != '$') return false;

  int starIdx = line.indexOf('*');
  if (starIdx < 0 || starIdx + 2 >= line.length()) return false;

  uint8_t calc = 0;
  for (int i = 1; i < starIdx; i++) {
    calc ^= (uint8_t) line[i];
  }

  int hi = hexVal(line[starIdx + 1]);
  int lo = hexVal(line[starIdx + 2]);
  if (hi < 0 || lo < 0) return false;
  uint8_t sent = (uint8_t)((hi << 4) | lo);

  return (sent == calc);
}

// kas see on GNSS lause, mida STM32 vajab?
// lubame $GP.. $GN.. $GL.. $GA.. $GB.. $GQ.. ja Quecteli $PQT..
static bool isGnssSentenceWeCareAbout(const String &line) {
  if (!line.startsWith("$")) return false;

  if (line.startsWith("$GP")) return true;
  if (line.startsWith("$GN")) return true;
  if (line.startsWith("$GL")) return true;
  if (line.startsWith("$GA")) return true;
  if (line.startsWith("$GB")) return true;
  if (line.startsWith("$GQ")) return true;

  if (line.startsWith("$PQT")) return true; // Quectel proprietary (PQTMEPE, PQTMPL)

  return false;
}

// saada üks puhas GNSS lause STM32-le
static void forwardNMEAtoSTM(const String &line) {
  if (!isGnssSentenceWeCareAbout(line)) return;
  if (!isValidNMEALine(line)) return;
  SerialSTM.println(line);
}

// loe GPSilt, uuenda LED infot ja saada STM32-le ainult puhtad laused
static void pumpGPSToSTM() {
  static String lineBuf;
  static bool   binaryFlag = false;

  while (SerialGPS.available() > 0) {
    char c = (char)SerialGPS.read();

    // --- LED/fix jälgimine GGA põhjal ---
    {
      static String ledLine;
      if (c == '\n') {
        if (ledLine.startsWith("$GNGGA") || ledLine.startsWith("$GPGGA")) {
          ggaForLED   = ledLine;
          lastGGATime = millis();
          checkRTKLED(ggaForLED); // uuenda currentFixState
        }
        ledLine = "";
      } else if (c != '\r') {
        if (ledLine.length() < 200) {
          ledLine += c;
        }
      }
    }

    // --- STM32 forward puhvrisse ---
    if (c == '\n') {
      lineBuf.trim(); // eemalda CR saba

      if (!binaryFlag && lineBuf.startsWith("$")) {
        forwardNMEAtoSTM(lineBuf);
      }

      // järgmise rea jaoks nulli
      lineBuf = "";
      binaryFlag = false;
      continue;
    }

    if (c == '\r') {
      // ignore CR, ootame '\n'
      continue;
    }

    // kui näeme mitteprintitavat (alla 32 või üle 126),
    // märgime selle rea binaarseks -> STM32-le ei saadeta
    if (((uint8_t)c < 32) || ((uint8_t)c > 126)) {
      binaryFlag = true;
    } else {
      if (lineBuf.length() < 240) {
        lineBuf += c;
      }
    }
  }
}


// =========================================================
// STM32 → GPS
// Kui STM32 tahab GPSile käske (UBX/NMEA config), luba see liiklus.
// =========================================================
//static void pumpSTMToGPS() {
//  while (SerialSTM.available() > 0) {
//    char c = (char)SerialSTM.read();
//    SerialGPS.write(c);
//  }
//}

void pumpSTMToGPS() {
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
    //    rebootGPS();
        rebootAll();
      } else if (stmCommandBuffer == "REBOOT_ESP32") {
        Serial.println("Received REBOOT_ESP32 command from STM32");
        scheduleESP32Reboot();
      } //else if (stmCommandBuffer == "REBOOT_ALL") {
        //Serial.println("Received REBOOT_ALL command from STM32");
        //rebootAll();
      //}
      
      stmCommandBuffer = "";
    }
  }
  // Puhasta puhver kui liiga pikk või aegunud
  if (stmCommandBuffer.length() > 100 || 
      (stmCommandBuffer.length() > 0 && millis() - lastCommandTime > 1000)) {
    stmCommandBuffer = "";
  }
}

// =========================================================
// setup()
// =========================================================
void setup() {
  // USB debug
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== ESP32 LC29H NTRIP Forwarder ===");

  // LED pin init
  pinMode(ESP_LED_PIN, OUTPUT);
  digitalWrite(ESP_LED_PIN, LOW);

  // UART init
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  SerialSTM.begin(STM_BAUD, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);

  // proovi WiFi STA, kui ei saa -> AP fallback
  connectWiFiSTA();

  // kui STA toimib ja NTRIP lubatud -> ühenda casteriga
  if (wifiConnected && USE_NTRIP) {
    connectNTRIP();
  }

  lastReconnectAttemptWiFi  = millis();
  lastReconnectAttemptNTRIP = millis();
  lastNTRIPData             = millis();
}


// =========================================================
// loop()
// =========================================================
void loop() {

  // 1. Hoia WiFi elus või lase vajadusel STA uuesti proovida
  ensureWiFi();

  // 2. Hoia NTRIP elus (reconnecti kui vaja)
  ensureNTRIP();

  // 3. Kui NTRIPist jookseb RTCM, pumpa see GPSi
  pumpCasterToGPS();

  // 4. Loe GPS -> filtreeri -> STM32 (ainult puhas NMEA/PQTMEPE/PQTMPL)
  pumpGPSToSTM();

  // 5. STM32 -> GPS käsud (vajadusel)
  pumpSTMToGPS();

  // 6. Uuenda LED (RTK FIX/FLOAT/no fix vilkumisloogika)
  if (millis() - lastLEDUpdate > 50) {
    lastLEDUpdate = millis();
    updateLED();
  }

  // 7. Kontrolli, kas on plaanitud ESP32 restart
  checkScheduledReboot();

  // 8. Väike hingetõmme
  delay(1);
}
