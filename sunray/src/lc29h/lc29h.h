#ifndef LC29H_H
#define LC29H_H

#include "Arduino.h"			
#include "../../gps.h"
#include "../driver/RobotDriver.h"

class LC29H : public GpsDriver {
  public:
    LC29H();    
    void begin(Client &client, char *host, uint16_t port) override;
    void begin(HardwareSerial& bus,uint32_t baud) override;
    void run() override;
    bool configure() override;  
    void reboot() override;
    void send(const uint8_t *buffer, size_t size) override;  
    void sendRTCM(const uint8_t *buffer, size_t size) override;
    unsigned long getGpsAge();

    // Lisa get-funktsioonid comm.cpp jaoks (ILMA OVERRIDETA)
    double getRelPosN() { return relPosN; }
    double getRelPosE() { return relPosE; }
    double getRelPosD() { return relPosD; }
    unsigned long getITOW() { return iTOW; }
    double getDgpsAge() { return dgpsAge; }
    int getNumSVDgps() { return numSVdgps; }
    int getChksumErrorCounter() { return chksumErrorCounter; }
    int getDgpsChecksumErrorCounter() { return dgpsChecksumErrorCounter; }
    int getDgpsPacketCounter() { return dgpsPacketCounter; }
    double getAccuracy() { return accuracy; }
    double getRtkRatio() { 
      if (numSV == 0) return 0.0;
      return (double)numSVdgps / (double)numSV; 
    }

    // Lisa need muutujad
    double relPosN = 0;
    double relPosE = 0;
    double relPosD = 0;
    unsigned long iTOW = 0;
    double dgpsAge = 0;
    int numSVdgps = 0;
    int chksumErrorCounter = 0;
    int dgpsChecksumErrorCounter = 0;
    int dgpsPacketCounter = 0;
    double accuracy = 0;

  private:
    uint32_t _baud;  	
    HardwareSerial* _bus;
    Client* _client;
    bool useTCP;
    unsigned long lastValidDataTime;
    int activeSats = 0; // Aktiivsete satelliitide arv GSA sõnumist
    
    void begin();
    void parseNMEA(const String& nmea);
    double convertNmeaToDecimal(const String& nmeaCoord, const String& dir);
    void updateSolution(int quality);
    void updateDGPSSatellites();
};

#endif