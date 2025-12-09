#ifndef LC29H_PARSER_H
#define LC29H_PARSER_H

#include "Arduino.h"
#include <math.h>
#include "../../gps.h"

// LC29HParser vastutab NMEA/PQTM* ridade lahtiparsimise eest
// ning uuendab nii globaale (lat, lon, jms) kui ka oma sisemisi diagnostika välju.
class LC29HParser {
public:
  LC29HParser();

  // Parsi üks NMEA/PQT* rida.
  void parseNMEA(const String& nmea);

  // --- Getterid driverile/ülejäänud koodile ---

  // Aeg (RMC-st)
  unsigned long getITOW() const            { return iTOW; }

  // dgpsAge hoiab ajamärki (millis), millal viimane RTK FIX GGA saabus.
  // Enne esimest FIX-i on see 0 -> Age kasvab alates käivitusest.
  // UI arvutab: Age = (millis() - getDgpsAge()) / 1000 (või /60000).
  double        getDgpsAge() const         { return dgpsAge; }

  // Efektiivne RTK AGE sekundites (aeg viimasest RTK FIX-ist).
  // Kui FIX-i pole veel olnud (dgpsAge == 0), tagastab NAN.
  double        getDgpsAgeEffective() const;

  int           getNumSVDgps() const       { return numSVdgps; }
  int           getDgpsPacketCounter()const{ return dgpsPacketCounter; }

  // Täpsus
  double        getAccuracy() const        { return accuracy; }             // EPE_2D (m) PQTMEPE #5
  double        getProtectionLevel() const { return protectionLevel; }      // max(N,E) (m) PQTMPL

  // HDOP (diagnostika) – tuleb globaalsest hAccuracy’st
  double        getHdop() const; // defineeritud .cpp-s (loeb globaali)

  // RTK suhe: numSVdgps / numSV (numSV globaalsest)
  double        getRtkRatio() const;

  // Relatiivne positsioon
  double        getRelPosN() const         { return relPosN; }
  double        getRelPosE() const         { return relPosE; }
  double        getRelPosD() const         { return relPosD; }

  double getLatitude()  const;
  double getLongitude() const;
  double getHeight()    const;
  int    getNumSV()     const;
  int    getSolution()  const;

  // Ajaabi driverile: millal saime GGA’st kehtivat infot
  unsigned long getLastValidDataTime() const { return lastValidDataTime; }

  // Ühilduvuse pärast – tagastab sama mis dgpsAge castituna.
  unsigned long getLastDgpsAgeUpdateMillis() const {
    return (unsigned long)dgpsAge;
  }

private:
  // sisemine seis
  int           activeSats;               // kasutuses olevate satelliitide arv (GGA väli 7)
  unsigned long lastValidDataTime;        // millis() kui positsioon viimati uuendati (GGA)
  double        relPosN, relPosE, relPosD;
  unsigned long iTOW;

  // millis(), millal viimane RTK FIX GGA saabus (0 enne esimest FIX-i)
  double        dgpsAge;

  int           numSVdgps;
  int           dgpsPacketCounter;

  double        accuracy;                 // EPE_2D (m)
  double        protectionLevel;          // PL (m)

  // Abifunktsioonid
  static int    splitFields(const String& s, String *outFields, int maxFields);
  static double convertNmeaToDecimal(const String& nmeaCoord, const String& dir);
  void          updateSolution(int quality);
  void          updateDGPSSatellites();
};

#endif // LC29H_PARSER_H
