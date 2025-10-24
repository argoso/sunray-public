// Ardumower Sunray 
// LC29H GPS driver for Quectel LC29H module

#include "../../config.h"
#include "Arduino.h"
#include "lc29h.h"
#include "../../gps.h"

// Defineeri GPS muutujad siin, kui gps.cpp puudub
double lat = 0;
double lon = 0;
double height = 0;
double hAccuracy = 0;
double vAccuracy = 0;
double groundSpeed = 0;
int numSV = 0;
int solution = SOL_INVALID;
bool solutionAvail = false;
unsigned long solutionTimeout = 0;

LC29H::LC29H()
{
  useTCP = false;
  lastValidDataTime = 0;
  relPosN = 0;
  relPosE = 0;
  relPosD = 0;
  iTOW = 0;
  dgpsAge = 0;
  numSVdgps = 0;
  chksumErrorCounter = 0;
  dgpsChecksumErrorCounter = 0;
  dgpsPacketCounter = 0;
  accuracy = 0;
  activeSats = 0;
}

void LC29H::begin(){
  CONSOLE.println("using gps driver: LC29H");    
  this->dgpsAge = 0;
  this->solutionAvail = false;
  this->numSV = 0;
  this->numSVdgps = 0;
  this->accuracy = 0;
  this->activeSats = 0;
  
  if (GPS_CONFIG){
    configure();
  }
}

void LC29H::begin(HardwareSerial& bus,uint32_t baud)
{	
  CONSOLE.println("LC29H begin serial:");
  CONSOLE.print(bus);
  CONSOLE.println(baud);
  _bus = &bus;
  _baud = baud;  
  _bus->begin(_baud);  
  begin();
}

void LC29H::begin(Client &client, char *host, uint16_t port)
{
  CONSOLE.println("LC29H::begin tcp");
  useTCP = true;
  _client = &client;
  if(!client.connect(host,port)){
    CONSOLE.print("Cannot connect to ");
    CONSOLE.print(host);
    CONSOLE.print(":");
    CONSOLE.println(port);
  }
  begin();
}

bool LC29H::configure(){  
  CONSOLE.println("LC29H already configured - skipping configuration"); 
  return true;
}

void LC29H::reboot(){
  CONSOLE.println("Rebooting LC29H GPS module via ESP32...");
  
  if (_bus) {
    // Saada käsk ESP32-le GPS rebootimiseks
    _bus->println("REBOOT_GPS");
    _bus->flush(); // Oota kuni kõik andmed on saadetud
    CONSOLE.println("Reboot command sent to ESP32");
    
    // Ära oota siin - ESP32 tegeleb GPS rebootiga
  } else if (useTCP && _client) {
    CONSOLE.println("TCP mode - cannot send reboot command");
  }
}

void LC29H::send(const uint8_t *buffer, size_t size){
  if (_bus) _bus->write(buffer, size);  
}

void LC29H::sendRTCM(const uint8_t *buffer, size_t size){
  if (_bus) _bus->write(buffer, size);  
}

double LC29H::convertNmeaToDecimal(const String& nmeaCoord, const String& dir) {
  if (nmeaCoord.length() < 4) return 0.0;
  
  int degLength = (dir == "N" || dir == "S") ? 2 : 3;
  
  // Kui on pikkuskraad ja algab 0-ga, siis eemalda esimene 0
  if ((dir == "E" || dir == "W") && nmeaCoord[0] == '0') {
    degLength = 2;
    String degStr = nmeaCoord.substring(1, 3);
    String minStr = nmeaCoord.substring(3);
    
    int deg = degStr.toInt();
    double min = minStr.toFloat();
    double decimal = deg + min / 60.0;
    
    if (dir == "S" || dir == "W") decimal = -decimal;
    
    return decimal;
  }
  else {
    String degStr = nmeaCoord.substring(0, degLength);
    String minStr = nmeaCoord.substring(degLength);
    
    int deg = degStr.toInt();
    double min = minStr.toFloat();
    double decimal = deg + min / 60.0;
    
    if (dir == "S" || dir == "W") decimal = -decimal;
    
    return decimal;
  }
}

void LC29H::updateSolution(int quality) {
  switch (quality) {
    case 4: solution = SOL_FIXED; break;    // RTK Fixed
    case 5: solution = SOL_FLOAT; break;    // RTK Float  
    case 2: solution = SOL_INVALID; break;  // DGPS
    case 1: solution = SOL_INVALID; break;  // Autonomous
    default: solution = SOL_INVALID;
  }
}

void LC29H::updateDGPSSatellites() {
  // Arvuta DGPS satelliitide arv aktiivsete satelliitide põhjal
  // See on indikatiivne väärtus statistika jaoks
  if (activeSats > 0) {
    // Lihtne loogika: eelda, et DGPS kasutab umbes 70% aktiivsetest satelliitidest
    // Või kasuta kvaliteedikoodi põhjal erinevaid loogikaid
    switch (solution) {
      case SOL_FIXED:    // RTK Fixed - kõrge kvaliteet
        numSVdgps = min(activeSats, numSV); // Kasuta kõiki aktiivseid satelliite
        break;
      case SOL_FLOAT:    // RTK Float - keskmine kvaliteet  
        numSVdgps = (int)(activeSats * 0.8); // ~80% aktiivsetest
        break;
      case SOL_INVALID:  // DGPS või muud
        if (solutionAvail) {
          numSVdgps = (int)(activeSats * 0.6); // ~60% aktiivsetest DGPS puhul
        } else {
          numSVdgps = 0; // Lahendust pole
        }
        break;
      default:
        numSVdgps = 0;
    }
    
    // Veendu, et numSVdgps ei ületa kunagi numSV või activeSats
    numSVdgps = min(numSVdgps, numSV);
    numSVdgps = min(numSVdgps, activeSats);
  } else {
    numSVdgps = 0;
  }
}

void LC29H::parseNMEA(const String& nmea) {
  if (nmea.startsWith("$GNGGA") || nmea.startsWith("$GPGGA")) {
    int startIndex = 0;
    int fieldIndex = 0;
    String fields[15];
    
    for (int i = 0; i < nmea.length(); i++) {
      if (nmea[i] == ',' || nmea[i] == '*') {
        fields[fieldIndex++] = nmea.substring(startIndex, i);
        startIndex = i + 1;
        if (fieldIndex >= 14) break;
      }
    }
    
    if (startIndex < nmea.length()) {
      fields[fieldIndex] = nmea.substring(startIndex);
    }
    
    if (fieldIndex >= 13) {
      String timeStr = fields[1];
      String latStr = fields[2];
      String latDir = fields[3];
      String lonStr = fields[4];
      String lonDir = fields[5];
      String qualityStr = fields[6];
      String satellitesStr = fields[7];
      String hdopStr = fields[8];
      String altStr = fields[9];
      
      // LISA: DGPS väljad
      String dgpsAgeStr = (fieldIndex >= 13) ? fields[13] : "";
      String dgpsStationStr = (fieldIndex >= 14) ? fields[14] : "";
      
      int quality = qualityStr.toInt();
      int satellites = satellitesStr.toInt();
      float hdop = hdopStr.toFloat();
      float alt = altStr.toFloat();
      
      // LISA: DGPS andmete töötlemine - COMM.CPP OOTAB dgpsAge millisekundites
      if (dgpsAgeStr != "" && dgpsAgeStr != " " && dgpsAgeStr.length() > 0) {
        float dgpsAgeSeconds = dgpsAgeStr.toFloat();
        dgpsAge = millis() - (unsigned long)(dgpsAgeSeconds * 1000);
        dgpsPacketCounter++;
      } else {
        dgpsAge = millis();
      }
      
      double latitude = convertNmeaToDecimal(latStr, latDir);
      double longitude = convertNmeaToDecimal(lonStr, lonDir);
      
      lat = latitude;
      lon = longitude;
      height = alt;
      hAccuracy = hdop;
      accuracy = hdop;
      numSV = satellites;
      
      relPosN = lat;
      relPosE = lon;
      relPosD = height;
      
      lastValidDataTime = millis();
      
      updateSolution(quality);
      
      // Uuenda DGPS satelliitide arvu
      updateDGPSSatellites();
      
      solutionAvail = true;
      solutionTimeout = millis() + 5000;
    }
  }
  else if (nmea.startsWith("$GNRMC") || nmea.startsWith("$GPRMC")) {
    int startIndex = 0;
    int fieldIndex = 0;
    String fields[15];
    
    for (int i = 0; i < nmea.length(); i++) {
      if (nmea[i] == ',' || nmea[i] == '*') {
        fields[fieldIndex++] = nmea.substring(startIndex, i);
        startIndex = i + 1;
        if (fieldIndex >= 12) break;
      }
    }
    
    if (fieldIndex >= 9) {
      String timeStr = fields[1];
      String status = fields[2];
      String latStr = fields[3];
      String latDir = fields[4];
      String lonStr = fields[5];
      String lonDir = fields[6];
      String speedStr = fields[7];
      String courseStr = fields[8];
      String dateStr = fields[9];
      
      if (timeStr.length() >= 6) {
        int hours = timeStr.substring(0,2).toInt();
        int minutes = timeStr.substring(2,4).toInt();
        double seconds = timeStr.substring(4).toFloat();
        iTOW = (unsigned long)((hours * 3600 + minutes * 60 + seconds) * 1000);
      }
      
      groundSpeed = speedStr.toFloat() * 0.514444;
    }
  }
  else if (nmea.startsWith("$GNGSA") || nmea.startsWith("$GPGSA")) {
    int startIndex = 0;
    int fieldIndex = 0;
    String fields[20];
    
    for (int i = 0; i < nmea.length(); i++) {
      if (nmea[i] == ',' || nmea[i] == '*') {
        fields[fieldIndex++] = nmea.substring(startIndex, i);
        startIndex = i + 1;
        if (fieldIndex >= 18) break;
      }
    }
    
    // TÄIENDA: GSA sõnumi DOP väärtused ja aktiivsed satelliidid
    if (fieldIndex >= 17) {
      String pdopStr = fields[15];
      String hdopStr = fields[16];
      String vdopStr = fields[17];
      
      hAccuracy = hdopStr.toFloat();
      vAccuracy = vdopStr.toFloat();
      accuracy = hAccuracy;
      
      // LISA: Loenda aktiivsed satelliidid GSA sõnumist (väljad 3-14)
      activeSats = 0;
      for (int i = 3; i <= 14; i++) {
        if (i < fieldIndex && fields[i] != "" && fields[i] != " ") {
          activeSats++;
        }
      }
      
      // Uuenda DGPS satelliitide arvu uute andmete põhjal
      updateDGPSSatellites();
    }
  }
  else if (nmea.startsWith("$GNVTG") || nmea.startsWith("$GPVTG")) {
    int startIndex = 0;
    int fieldIndex = 0;
    String fields[12];
    
    for (int i = 0; i < nmea.length(); i++) {
      if (nmea[i] == ',' || nmea[i] == '*') {
        fields[fieldIndex++] = nmea.substring(startIndex, i);
        startIndex = i + 1;
        if (fieldIndex >= 10) break;
      }
    }
    
    if (fieldIndex >= 8) {
      String speedKmhStr = fields[7];
      groundSpeed = speedKmhStr.toFloat() / 3.6;
    }
  }
  else if (nmea.startsWith("$GNGLL") || nmea.startsWith("$GPGLL")) {
    // GLL message - ignoreeri CRC vigu sellel sõnumil
  }
  else if (nmea.startsWith("$GPGSV") || nmea.startsWith("$GNGSV") || 
           nmea.startsWith("$GLGSV") || nmea.startsWith("$GQGSV")) {
    // GSV message - satelliitide info
    int commaCount = 0;
    for (int i = 0; i < nmea.length(); i++) {
      if (nmea[i] == ',') commaCount++;
    }
    
    if (commaCount >= 4) {
      int startIndex = nmea.indexOf(',') + 1;
      int endIndex = nmea.indexOf(',', startIndex);
      if (endIndex != -1) {
        startIndex = endIndex + 1;
        endIndex = nmea.indexOf(',', startIndex);
        if (endIndex != -1) {
          startIndex = endIndex + 1;
          endIndex = nmea.indexOf(',', startIndex);
          if (endIndex != -1) {
            String satsInViewStr = nmea.substring(startIndex, endIndex);
            int satsInView = satsInViewStr.toInt();
            if (satsInView > numSV) {
              numSV = satsInView;
            }
          }
        }
      }
    }
  }
}

void LC29H::run()
{
  static String nmeaBuffer = "";
  static unsigned long lastCrcErrorReport = 0;
  static int crcErrorCount = 0;
  
  // Kontrolli GPS andmete aegumist
  if (solutionAvail && millis() > solutionTimeout) {
    solution = SOL_INVALID;
    solutionAvail = false;
    solutionTimeout = millis() + 1000;
  }
  
  Stream *stream; 
  if (useTCP) stream = _client;
  else stream = _bus;

  if (!stream->available()) return;
  
  while (stream->available()) {
    char c = stream->read();
    
    // Lisa märk puhvrisse, kui see on printable või rea lõpu märk

    if (c == '\n' || c == '\r' || (c >= 32 && c <= 126)) {
      nmeaBuffer += c;
    }
    
    // Kui saime terve rea, parseeri
    if (c == '\n') {
      nmeaBuffer.trim();
      
      if (nmeaBuffer.length() > 6 && nmeaBuffer[0] == '$') {
        int starIndex = nmeaBuffer.indexOf('*');
        
        if (starIndex != -1 && nmeaBuffer.length() > starIndex + 2) {
          // CRC on olemas - kontrolli seda
          String withoutCrc = nmeaBuffer.substring(0, starIndex);
          String crcReceived = nmeaBuffer.substring(starIndex + 1, starIndex + 3);
          
          // CRC arvutus otse siin
          uint8_t calculatedCrc = 0;
          for (int i = 1; i < withoutCrc.length(); i++) {
            if (withoutCrc[i] == '*') break;
            calculatedCrc ^= withoutCrc[i];
          }
          
          String calculatedCrcStr = String(calculatedCrc, HEX);
          if (calculatedCrcStr.length() == 1) {
            calculatedCrcStr = "0" + calculatedCrcStr;
          }
          calculatedCrcStr.toUpperCase();
          
          crcReceived.toUpperCase();
          
          if (calculatedCrcStr == crcReceived) {
            // CRC OK - parseeri sõnum
            parseNMEA(withoutCrc);
            crcErrorCount = 0; // Reseti vealoendur edukal sõnumil

          
          } else {
            // CRC viga - logi ainult kui see ei ole GLL sõnum

            if (!nmeaBuffer.startsWith("$GNGLL") && !nmeaBuffer.startsWith("$GPGLL")) {
              chksumErrorCounter++;
              crcErrorCount++;
              
              // Logi CRC vead ainult iga 10 sekundi järel või kui on esimesed 5 viga
              if (crcErrorCount <= 5 || millis() - lastCrcErrorReport > 10000) {
                lastCrcErrorReport = millis();
                
                if (crcErrorCount > 5) {
                  // Võib logida kui on palju vigu
                }
              }
            }
          }
        } else {
          // Pole CRC-d või CRC osa on puudulik - parseeri otse
          parseNMEA(nmeaBuffer);
        }
      }
      
      nmeaBuffer = "";
    }
  }
  
  // Puhasta puhver kui liiga pikk
  if (nmeaBuffer.length() > 200) {
    nmeaBuffer = "";
  }
}

unsigned long LC29H::getGpsAge() {
  if (lastValidDataTime == 0) {
    return 999999;
  }
  
  unsigned long currentAge = (millis() - lastValidDataTime) / 1000;
  return currentAge;
}