// LC29H parser: ainult ridade lahtiparsimine ja globaalse GNSS seisu uuendamine.

#include "Arduino.h"
#include <math.h>
#include "lc29h_parser.h"
#include "../../gps.h"

extern bool   absolutePosSource;
extern double absolutePosSourceLon, absolutePosSourceLat;

// ---- Globaalsed GNSS muutujad (teised koodiosad eeldavad neid) ----
double lat = 0;
double lon = 0;
double height = 0;
unsigned long solutionTimeout = 0;  // jään alles, aga ei kasuta absolute-deadline mustrit

double hAccuracy = 0;   // HDOP (dimensionless)
double vAccuracy = 0;   // hetkel ei täideta – säilitame ühilduvuse

double groundSpeed = 0;
int    numSV = 0;
int    solution = SOL_INVALID;
bool   solutionAvail = false;

double LC29HParser::getLatitude()  const { extern double lat;      return lat; }
double LC29HParser::getLongitude() const { extern double lon;      return lon; }
double LC29HParser::getHeight()    const { extern double height;   return height; }
int    LC29HParser::getNumSV()     const { extern int numSV;       return numSV; }
int    LC29HParser::getSolution()  const { extern int solution;    return solution; }

// -------------------------------------------------------------------

LC29HParser::LC29HParser()
: activeSats(0),
  lastValidDataTime(0),
  relPosN(0), relPosE(0), relPosD(0),
  iTOW(0),
  dgpsAge(0.0),                 // 0 -> Age kasvab alates käivitusest
  numSVdgps(0),
  dgpsPacketCounter(0),
  accuracy(NAN),
  protectionLevel(NAN)
{}

int LC29HParser::splitFields(const String& s, String *outFields, int maxFields) {
  int fieldIndex = 0;
  int startIndex = 0;
  const int len = s.length();
  for (int i = 0; i < len; i++) {
    if (s[i] == ',') {
      if (fieldIndex < maxFields) outFields[fieldIndex++] = s.substring(startIndex, i);
      startIndex = i + 1;
      if (fieldIndex >= maxFields) break;
    }
  }
  if (fieldIndex < maxFields && startIndex <= len) {
    outFields[fieldIndex++] = s.substring(startIndex);
  }
  return fieldIndex;
}

double LC29HParser::convertNmeaToDecimal(const String& nmeaCoord, const String& dir) {
  if (nmeaCoord.length() < 4) return 0.0;
  const int degLength = (dir == "N" || dir == "S") ? 2 : 3;
  String degStr = nmeaCoord.substring(0, degLength);
  String minStr = nmeaCoord.substring(degLength);
  double deg = degStr.toDouble();
  double minutes = minStr.toDouble();
  double decimal = deg + minutes / 60.0;
  if (dir == "S" || dir == "W") decimal = -decimal;
  return decimal;
}

void LC29HParser::updateSolution(int quality) {
  // 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float
  switch (quality) {
    case 4: solution = SOL_FIXED;  break;
    case 5: solution = SOL_FLOAT;  break;
    case 2: solution = SOL_FLOAT;  break;
    case 1: solution = SOL_FLOAT;  break;
    default: solution = SOL_INVALID;
  }
}

void LC29HParser::updateDGPSSatellites() {
  if (activeSats > 0) {
    switch (solution) {
      case SOL_FIXED:  numSVdgps = min(activeSats, numSV); break;
      case SOL_FLOAT:  numSVdgps = min(activeSats, numSV); break;
      case SOL_INVALID:
      default:         numSVdgps = 0; break;
    }
    numSVdgps = min(numSVdgps, activeSats);
  } else {
    numSVdgps = 0;
  }
}

double LC29HParser::getHdop() const {
  // HDOP talletub globaalsesse hAccuracy-sse, et säilitada ühilduvus
  return hAccuracy;
}

double LC29HParser::getRtkRatio() const {
  if (numSV == 0) return 0.0;
  return (double)numSVdgps / (double)numSV;
}

// Efektiivne RTK AGE: aeg viimasest RTK FIX-ist (sekundites).
double LC29HParser::getDgpsAgeEffective() const {
  // Kui pole kunagi FIX-i olnud (dgpsAge = 0), tagasta NAN
  if (dgpsAge <= 0.0) return NAN;

  unsigned long nowMs = millis();
  unsigned long fixMs = (unsigned long)dgpsAge;
  unsigned long elapsedMs;

  if (nowMs >= fixMs) {
    elapsedMs = nowMs - fixMs;
  } else {
    // millis() overflow või muu jama – ära lase negatiivseks minna
    elapsedMs = 0;
  }

  return (double)elapsedMs / 1000.0;
}

// ---- NMEA checksum & rea puhastamine (faili-sisesed helperid) ----
static uint8_t _calcNmeaChecksum(const String& s) {
  int start = s.indexOf('$');
  int star  = s.indexOf('*');
  if (start < 0) start = 0;
  if (star  < 0) star  = s.length();
  uint8_t cs = 0;
  for (int i = start + 1; i < star; i++) cs ^= (uint8_t)s[i];
  return cs;
}

static String _stripChecksumAndTrim(const String& line, bool& checksumOk) {
  String s = line;
  while (s.endsWith("\r") || s.endsWith("\n")) s.remove(s.length() - 1);
  int star = s.indexOf('*');
  checksumOk = true;
  if (star >= 0) {
    if (star + 2 < s.length()) {
      String hex = s.substring(star + 1, star + 3);
      auto hexVal = [](char c)->int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
        if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
        return -1;
      };
      int hi = hexVal(hex[0]), lo = hexVal(hex[1]);
      if (hi >= 0 && lo >= 0) {
        uint8_t want = (uint8_t)((hi << 4) | lo);
        uint8_t got  = _calcNmeaChecksum(s);
        checksumOk = (want == got);
      }
    }
    s = s.substring(0, star); // eemalda *CS parsingu jaoks
  }
  return s;
}

void LC29HParser::parseNMEA(const String& nmea) {
  // normaliseeri rida ja kontrolli checksum
  bool csOK = true;
  String s = _stripChecksumAndTrim(nmea, csOK);
  if (!csOK) return;

  // --- GGA: positsioon, fixQuality, HDOP, sat arv ---
  if (s.startsWith("$GNGGA") || s.startsWith("$GPGGA")) {

    String fields[16];
    int fieldIndex = splitFields(s, fields, 16);

    if (fieldIndex >= 14) {
      String timeStr        = fields[1];
      String latStr         = fields[2];
      String latDir         = fields[3];
      String lonStr         = fields[4];
      String lonDir         = fields[5];
      String qualityStr     = fields[6];
      String satellitesStr  = fields[7];
      String hdopStr        = fields[8];
      String altStr         = fields[9];
      // String dgpsAgeStr  = fields[13]; // GGA AGE – EI KASUTA ENAM

      int   quality    = qualityStr.toInt();
      int   satellites = satellitesStr.toInt();
      float hdopVal    = hdopStr.toFloat();
      float alt        = altStr.toFloat();

      // kasutuses olevad satelliidid (diagnostika)
      activeSats = satellites;
      if (numSV < activeSats) numSV = activeSats;

      // --- RTK FIX AGE LOOGIKA ---
      // Enne esimest FIX-i on dgpsAge = 0 -> Age = millis() - 0 (kasvab alates käivitusest).
      // Kui quality == 4 (RTK FIX):
      //   - salvestame ajamärgi millis(), millal FIX-GGA saabus
      // Kui quality != 4, siis dgpsAge ei muutu -> Age kasvab edasi.
      unsigned long nowMs = millis();
      if (quality == 4) {
        dgpsAge = (double)nowMs;
        dgpsPacketCounter++;
      }

      // koordinaadid kraadides
      double latitude  = convertNmeaToDecimal(latStr, latDir);
      double longitude = convertNmeaToDecimal(lonStr, lonDir);

      // uuenda globaale
      lat       = latitude;
      lon       = longitude;
      height    = alt;
      hAccuracy = hdopVal;

      // accuracy tuleb PQTMEPE lausest (EPE_2D); ära kirjuta HDOP-iga üle

      // --- MÄÄRA LAHENDUSE TÜÜP (DGPS/AUTON ka kasutatavad UI jaoks) ---
      updateSolution(quality);

      // --- RELATIIVPOSITSIOON MEETRITES (mitte kraadides!) ---
      // vali referents: AT+P (absolutePosSource) või FIX-i järel esimene kehtiv fix
      static bool   lc29h_haveRef = false;
      static double lc29h_refLon = 0.0;
      static double lc29h_refLat = 0.0;
      static double lc29h_refAlt = NAN;

      double refLon = 0.0, refLat = 0.0;
      if (absolutePosSource) {
        refLon = absolutePosSourceLon;
        refLat = absolutePosSourceLat;
        lc29h_haveRef = true;
        lc29h_refLon = refLon;
        lc29h_refLat = refLat;
        lc29h_refAlt = alt; // kui baasjaama kõrgus pole teada, kasuta hetke alt referentsiks
      } else {
        // Pane nullpunkt alles FIX-i saavutamisel
        if (!lc29h_haveRef && (solution == SOL_FIXED)) {
          lc29h_refLon = longitude;
          lc29h_refLat = latitude;
          lc29h_refAlt = alt;
          lc29h_haveRef = true;
        }
        refLon = lc29h_refLon;
        refLat = lc29h_refLat;
      }

      if (lc29h_haveRef) {
        const double R = 6378137.0; // WGS84
        const double deg2rad = 3.14159265358979323846 / 180.0;
        const double lat0 = refLat * deg2rad;
        const double dLon = (longitude - refLon) * deg2rad;
        const double dLat = (latitude  - refLat) * deg2rad;

        const double east  = R * dLon * cos(lat0);
        const double north = R * dLat;

        relPosN = north;   // meetrites
        relPosE = east;    // meetrites
        relPosD = (isnan(lc29h_refAlt) ? 0.0 : (alt - lc29h_refAlt)); // relatiivne D
      } else {
        // kuni referents puudub
        relPosN = 0.0;
        relPosE = 0.0;
        relPosD = (isnan(lc29h_refAlt) ? 0.0 : (alt - lc29h_refAlt));
      }

      lastValidDataTime = millis();
      updateDGPSSatellites();
      solutionAvail = true;
    }
  }

  // --- RMC: aeg ja kiirus ---
  else if (s.startsWith("$GNRMC") || s.startsWith("$GPRMC")) {

    String fields[16];
    int fieldIndex = splitFields(s, fields, 16);

    if (fieldIndex >= 10) {
      String status = fields[2];     // A=valid, V=void
      if (status == "A") {
        String timeStr   = fields[1];
        String speedStr  = fields[7];

        if (timeStr.length() >= 6) {
          int hours   = timeStr.substring(0,2).toInt();
          int minutes = timeStr.substring(2,4).toInt();
          double secs = timeStr.substring(4).toFloat();
          iTOW = (unsigned long)((hours * 3600 + minutes * 60 + secs) * 1000);
        }

        // sõlmed -> m/s
        groundSpeed = speedStr.toFloat() * 0.514444;
      }
    }
  }

  // --- GSV: nähtavad satelliidid ---
  else if (s.startsWith("$GPGSV") || s.startsWith("$GNGSV") ||
           s.startsWith("$GLGSV") || s.startsWith("$GQGSV") ||
           s.startsWith("$GBGSV") || s.startsWith("$GAGSV")) {

    static int visGPS = 0, visGLO = 0, visGAL = 0, visBDS = 0, visQZSS = 0;

    String fields[32];
    int fieldIndex = splitFields(s, fields, 32);
    if (fieldIndex >= 4) {
      int satsInView = fields[3].toInt();

      if      (s.startsWith("$GPGSV")) visGPS  = satsInView;   // GPS
      else if (s.startsWith("$GLGSV")) visGLO  = satsInView;   // GLONASS
      else if (s.startsWith("$GAGSV")) visGAL  = satsInView;   // Galileo
      else if (s.startsWith("$GBGSV")) visBDS  = satsInView;   // BeiDou
      else if (s.startsWith("$GQGSV")) visQZSS = satsInView;   // QZSS
      else if (s.startsWith("$GNGSV")) {
        // kombineeritud “kõik nähtavad”
        visGPS = visGLO = visGAL = visBDS = visQZSS = 0;
        numSV = satsInView; // “nähtavad kokku”
        return;
      }

      // per-konstellatsioonide summa:
      numSV = visGPS + visGLO + visGAL + visBDS + visQZSS;  // “visible total”
    }
  }

  // --- PQTMEPE: EPE_2D (m) ---
  else if (s.startsWith("$PQTMEPE")) {

    String fields[10];
    int fieldIndex = splitFields(s, fields, 10);

    // 0:$PQTMEPE 1:ver 2:EPE_N 3:EPE_E 4:EPE_D 5:EPE_2D 6:EPE_3D
    if (fieldIndex >= 6) {
      String epe2DStr = fields[5];
      if (epe2DStr.length() > 0) {
        accuracy = epe2DStr.toFloat(); // meetrites
      }
    }
  }

  // --- PQTMPL: Protection Level (m) ---
  else if (s.startsWith("$PQTMPL")) {

    String fields[10];
    int fieldIndex = splitFields(s, fields, 10);

    // Kui teine väli on "ERROR", siis lihtsalt ei uuenda (pole saadaval)
    if (fieldIndex >= 2 && fields[1] == "ERROR") {
      // leave protectionLevel as-is
    } else {
      // 0:$PQTMPL 1:ver 2:PL_PosN_mm 3:PL_PosE_mm ...
      if (fieldIndex >= 4) {
        String plNmmStr = fields[2];
        String plEmmStr = fields[3];
        if (plNmmStr.length() > 0 && plEmmStr.length() > 0) {
          float plN = plNmmStr.toFloat() / 1000.0f; // m
          float plE = plEmmStr.toFloat() / 1000.0f; // m
          float worst = (plN > plE) ? plN : plE;
          protectionLevel = worst;
        }
      }
    }
  }

  // muu: ignore
}
