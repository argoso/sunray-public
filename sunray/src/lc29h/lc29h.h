#ifndef LC29H_H
#define LC29H_H

#include "Arduino.h"
#include <math.h>
#include "../../gps.h"
#include "../driver/RobotDriver.h"
#include "lc29h_parser.h"

class LC29H : public GpsDriver {
  public:
    LC29H();
    // Riistvaraseisu jälgimine
    enum GpsHwState { GPS_HW_DISCONNECTED, GPS_HW_NO_DATA, GPS_HW_STREAMING };



    // Ühendused
    void begin(Client &client, char *host, uint16_t port) override;
    void begin(HardwareSerial& bus,uint32_t baud) override;
    void run() override;
    bool configure() override;
    void reboot() override;
    void send(const uint8_t *buffer, size_t size) override;
    void sendRTCM(const uint8_t *buffer, size_t size) override;

    unsigned long getGpsAge();

    // UUS: GPS riistvaraseis ja tervise getterid
    bool isPresent() const { return modulePresent; }
    GpsHwState getHwState() const { return hwState; }
    unsigned long lastByteAge() const { return lastByteAt ? (millis() - lastByteAt) : (unsigned long)0; }
    unsigned long lastSentenceAge() const { return lastSentenceAt ? (millis() - lastSentenceAt) : (unsigned long)0; }

    // --- Getterid (säilitame olemasoleva API) ---
    double getRelPosN()            { return parser.getRelPosN(); }
    double getRelPosE()            { return parser.getRelPosE(); }
    double getRelPosD()            { return parser.getRelPosD(); }
    unsigned long getITOW()        { return parser.getITOW(); }
    double getDgpsAge()            { return parser.getDgpsAge(); }
    int getNumSVDgps()             { return parser.getNumSVDgps(); }
    int getChksumErrorCounter()    { return chksumErrorCounter; }          // checksum loogika on driveris
    int getDgpsChecksumErrorCounter() { return dgpsChecksumErrorCounter; } // ei kasutata siin – säilitame API
    int getDgpsPacketCounter()     { return parser.getDgpsPacketCounter(); }

    double getAccuracy()           { return parser.getAccuracy(); }        // EPE_2D (m)
    double getProtectionLevel()    { return parser.getProtectionLevel(); } // PL (m)
    double getHdop()               { return parser.getHdop(); }
    double getRtkRatio()           { return parser.getRtkRatio(); }

    // --- Avalikud väljad ühilduvuse nimel (täidetakse run() käigus parserist) ---
    // NB: need dubleerivad parseri seisu; jäetud alles, sest muu kood võib neid otseselt kasutada.
    double relPosN = 0;
    double relPosE = 0;
    double relPosD = 0;
    unsigned long iTOW = 0;
    double dgpsAge = 0;
    int numSVdgps = 0;
    int chksumErrorCounter = 0;
    int dgpsChecksumErrorCounter = 0; // jätame nulli (checksum DGPS paketile kui kunagi vaja)
    int dgpsPacketCounter = 0;
    double accuracy = NAN;
    double protectionLevel = NAN;
    double lat = 0;
    double lon = 0;
    double height = 0;
    int    numSV = 0;
    SolType solution = SOL_INVALID;


  private:
    uint32_t _baud = 0;
    HardwareSerial* _bus = nullptr;
    Client* _client = nullptr;
    bool useTCP = false;

    LC29HParser parser;

    void beginInternal();
    void syncFromParser(); // peegeldab parseri seisud avalikesse väljadesse

    // --- UUS: ühenduse tuvastuse raamistik ---
    volatile bool modulePresent = false;
    GpsHwState hwState = GPS_HW_DISCONNECTED;
    unsigned long startupDetectDeadline = 0;
    unsigned long lastByteAt = 0;
    unsigned long lastSentenceAt = 0;
};

#endif
