// Ardumower Sunray
// LC29H GPS driver for Quectel LC29H module (driver + eraldi parser)

#include "../../config.h"
#include "Arduino.h"
#include <math.h>
#include "lc29h.h"
#include "../../gps.h"

LC29H::LC29H() {
  useTCP = false;
  _bus = nullptr;
  _client = nullptr;

  relPosN = relPosE = relPosD = 0;
  iTOW = 0;
  dgpsAge = 0;
  numSVdgps = 0;
  chksumErrorCounter = 0;
  dgpsChecksumErrorCounter = 0;
  dgpsPacketCounter = 0;
  accuracy = NAN;
  protectionLevel = NAN;
  // ühenduse tuvastus
  modulePresent = false;
  hwState = GPS_HW_DISCONNECTED;
  startupDetectDeadline = 0;
  lastByteAt = 0;
  lastSentenceAt = 0;

}

void LC29H::beginInternal() {
  CONSOLE.println("using gps driver: LC29H");
  this->dgpsAge = 0;
  this->numSVdgps = 0;
  this->accuracy = NAN;
  this->protectionLevel = NAN;

  
  // anna moodulile pisut aega ennast väljareklaamida
  startupDetectDeadline = millis() + 2000UL; // 2 s aken
  modulePresent = false;
  hwState = GPS_HW_NO_DATA;
  lastByteAt = 0;
  lastSentenceAt = 0;
if (GPS_CONFIG) {
    configure();
  }

  // Kui siia jõudsime ja baiti tuli, siis voog töötab
  hwState = GPS_HW_STREAMING;
}

void LC29H::begin(HardwareSerial& bus,uint32_t baud) {
  CONSOLE.println("LC29H begin serial:");
  CONSOLE.print(bus);
  CONSOLE.println(baud);
  _bus = &bus;
  _baud = baud;
  _bus->begin(_baud);
  useTCP = false;
  beginInternal();
}

void LC29H::begin(Client &client, char *host, uint16_t port) {
  CONSOLE.println("LC29H::begin tcp");
  useTCP = true;
  _client = &client;
  if (!client.connect(host, port)) {
    CONSOLE.print("Cannot connect to ");
    CONSOLE.print(host);
    CONSOLE.print(":");
    CONSOLE.println(port);
  }
  beginInternal();
}

bool LC29H::configure() {
  CONSOLE.println("LC29H already configured - skipping configuration");
  return true;
}

void LC29H::reboot() {
  CONSOLE.println("Rebooting LC29H GPS module via ESP32...");
  if (_bus) {
    _bus->println("REBOOT_GPS");
    _bus->flush();
    CONSOLE.println("Reboot command sent to ESP32");
  } else if (useTCP && _client) {
    CONSOLE.println("TCP mode - cannot send reboot command");
  }

  // Kui siia jõudsime ja baiti tuli, siis voog töötab
  hwState = GPS_HW_STREAMING;
}

void LC29H::send(const uint8_t *buffer, size_t size) {
  if (_bus) _bus->write(buffer, size);
}

void LC29H::sendRTCM(const uint8_t *buffer, size_t size) {
  if (_bus) _bus->write(buffer, size);
}

unsigned long LC29H::getGpsAge() {
  const unsigned long last = parser.getLastValidDataTime();
  if (last == 0) return (unsigned long)-1;
  return millis() - last;
}

void LC29H::syncFromParser() {
  // --- olemasolev sünk parser -> driver ---
  relPosN = parser.getRelPosN();
  relPosE = parser.getRelPosE();
  relPosD = parser.getRelPosD();
  iTOW    = parser.getITOW();
  dgpsAge = parser.getDgpsAge();
  numSVdgps = parser.getNumSVDgps();
  dgpsPacketCounter = parser.getDgpsPacketCounter();
  accuracy = parser.getAccuracy();
  protectionLevel = parser.getProtectionLevel();

  // peegelda ka driveri avalikesse GNSS väljadesse (comm.cpp loeb siit)
  extern double lat, lon, height;
  extern int    numSV, solution;
  this->lat      = lat;
  this->lon      = lon;
  this->height   = height;
  this->numSV    = numSV;
  this->solution = static_cast<SolType>(solution);

  // --------------------------------------------------------------------
  // 🔑 ARVUTA JA KIRJUTA stateX/stateY ISE (nii et UI "pos x/y" liiguks)
  // --------------------------------------------------------------------
  // kasutame absoluutset referentsi kui see on antud (AT+P),
  // või vastasel juhul seame referentsi alles RTK FIX korral
  extern bool   absolutePosSource;
  extern double absolutePosSourceLon, absolutePosSourceLat;
  extern float  stateX, stateY;

  static bool   localRefInit = false;
  static double localRefLon = 0.0, localRefLat = 0.0;

  // lahenduse peab pidama "kasutatavaks"
  if (this->solution != SOL_INVALID) {
    double refLon = 0.0, refLat = 0.0;
    bool haveRef = false;

    if (absolutePosSource) {
      refLon = absolutePosSourceLon;
      refLat = absolutePosSourceLat;
      haveRef = true;
    } else {
      // ⚠️ MUUDETUD: nullpunkt seatakse alles siis, kui RTK on FIX
      if (!localRefInit &&
          this->solution == SOL_FIXED &&
          !isnan(this->lon) && !isnan(this->lat)) {
        localRefLon = this->lon;
        localRefLat = this->lat;
        localRefInit = true;
      }
      if (localRefInit) {
        refLon = localRefLon;
        refLat = localRefLat;
        haveRef = true;
      }
    }

    if (haveRef) {
      // lihtne ENU-eeldus: Maa raadius ja equirectangular
      const double R = 6378137.0;          // WGS84
      const double deg2rad = 3.14159265358979323846 / 180.0;
      const double lat0 = refLat * deg2rad;

      const double dLon = (this->lon - refLon) * deg2rad;
      const double dLat = (this->lat - refLat) * deg2rad;

      const double east  = R * dLon * cos(lat0);
      const double north = R * dLat;

      // UI loeb neid kahte muutujat
      stateX = (float)east;
      stateY = (float)north;
    }
  }

  // Kui siia jõudsime ja baiti tuli, siis voog töötab
  hwState = GPS_HW_STREAMING;
}


void LC29H::run() {
  static String nmeaBuffer = "";
  static unsigned long lastCrcErrorReport = 0;
  static int crcErrorCount = 0;

  // aegumise kontroll parseri aja järgi
  if (solutionAvail) {
    unsigned long last = parser.getLastValidDataTime();
    if (last > 0 && (millis() - last) > 5000UL) {
      solution = SOL_INVALID;
      solutionAvail = false;
    }
  }

  Stream *stream = useTCP ? (Stream*)_client : (Stream*)_bus;
  if (!stream) {
    hwState = GPS_HW_DISCONNECTED;
    return;
  }

  // kui 2 s jooksul ei näe ühtki baiti, märgi DISCONNECTED
  if (!modulePresent && startupDetectDeadline && (millis() > startupDetectDeadline) && (lastByteAt == 0)) {
    hwState = GPS_HW_DISCONNECTED;
  }

  if (!stream->available()) {
    // pole hetkel baite — uuenda riistvaraseisu vanuse põhjal
    if (modulePresent) {
      const unsigned long age = millis() - lastByteAt;
      if (age > 3000UL) hwState = GPS_HW_NO_DATA; // >3s vaikust = NO_DATA
    }
    return;
  }

  auto shouldParse = [](const String& s)->bool {
    return (
      s.startsWith("$GNGGA") || s.startsWith("$GPGGA") ||
      s.startsWith("$GNRMC") || s.startsWith("$GPRMC") ||
      s.startsWith("$GPGSV") || s.startsWith("$GNGSV") ||
      s.startsWith("$GLGSV") || s.startsWith("$GAGSV") ||
      s.startsWith("$GBGSV") || s.startsWith("$GQGSV") ||
      s.startsWith("$PQTMEPE") || s.startsWith("$PQTMPL")
    );
  };

  auto hexVal = [](char ch)->int {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return 10 + (ch - 'A');
    if (ch >= 'a' && ch <= 'f') return 10 + (ch - 'a');
    return -1;
  };

  while (stream->available()) {
    char c = stream->read();
    lastByteAt = millis();
    if (!modulePresent) {
      modulePresent = true;
      CONSOLE.println("LC29H GPS module found");
    }

    if ((c >= 32 && c <= 126) || c == '\r' || c == '\n') {
      nmeaBuffer += c;
    }

    if (c == '\r' || c == '\n') {
      while (nmeaBuffer.endsWith("\r") || nmeaBuffer.endsWith("\n"))
        nmeaBuffer.remove(nmeaBuffer.length() - 1);

      if (nmeaBuffer.length() > 6 && nmeaBuffer[0] == '$') {
        const int starIndex = nmeaBuffer.lastIndexOf('*');
        lastSentenceAt = millis();

        // ---- LENIENT kontrollsumma käsitlus ----
        if (starIndex > 0 && nmeaBuffer.length() >= starIndex + 3) {
          const int hi = hexVal(nmeaBuffer[starIndex + 1]);
          const int lo = hexVal(nmeaBuffer[starIndex + 2]);
          String payload = nmeaBuffer.substring(0, starIndex);

          if (shouldParse(payload)) {
            bool crcOK = false;
            if (hi >= 0 && lo >= 0) {
              uint8_t calc = 0;
              for (int i = 1; i < starIndex; i++) calc ^= (uint8_t)nmeaBuffer[i];
              const uint8_t rx = (uint8_t)((hi << 4) | lo);
              crcOK = (calc == rx);
            }

            if (!crcOK) {
              chksumErrorCounter++;
              // vajadusel logi: CONSOLE.printf("NMEA CRC fail (lenient): %s\n", payload.c_str());
            }

            parser.parseNMEA(payload);
            syncFromParser();
            crcErrorCount = 0;
          }
        }
        else {
          // *HH puudub → ikka parseeri
          String payload = nmeaBuffer;
          if (shouldParse(payload)) {
            parser.parseNMEA(payload);
            syncFromParser();
            crcErrorCount = 0;
          }
        }
      }

      nmeaBuffer = "";
    }

    if (nmeaBuffer.length() > 600) nmeaBuffer = "";
  }

  hwState = GPS_HW_STREAMING;
}
