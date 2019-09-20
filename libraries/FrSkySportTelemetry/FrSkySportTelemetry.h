/*
  FrSky telemetry class for Teensy 3.x/LC, ESP8266 and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20190824
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_TELEMETRY_H_
#define _FRSKY_SPORT_TELEMETRY_H_

#include "Arduino.h"
#include "FrSkySportPolling.h"
#include "FrSkySportSensor.h"
#include "FrSkySportSingleWireSerial.h"

#define FRSKY_TELEMETRY_MAX_SENSORS 28

class FrSkySportTelemetry
{
  public:
    FrSkySportTelemetry(bool polling = false);
    void begin(FrSkySportSingleWireSerial::SerialId id,
                FrSkySportSensor* sensor1,         FrSkySportSensor* sensor2 =  NULL, FrSkySportSensor* sensor3 =  NULL, 
                FrSkySportSensor* sensor4  = NULL, FrSkySportSensor* sensor5 =  NULL, FrSkySportSensor* sensor6 =  NULL,
                FrSkySportSensor* sensor7  = NULL, FrSkySportSensor* sensor8 =  NULL, FrSkySportSensor* sensor9 =  NULL, 
                FrSkySportSensor* sensor10 = NULL, FrSkySportSensor* sensor11 = NULL, FrSkySportSensor* sensor12 = NULL,
                FrSkySportSensor* sensor13 = NULL, FrSkySportSensor* sensor14 = NULL, FrSkySportSensor* sensor15 = NULL,
                FrSkySportSensor* sensor16 = NULL, FrSkySportSensor* sensor17 = NULL, FrSkySportSensor* sensor18 = NULL,
                FrSkySportSensor* sensor19 = NULL, FrSkySportSensor* sensor20 = NULL, FrSkySportSensor* sensor21 = NULL,
                FrSkySportSensor* sensor22 = NULL, FrSkySportSensor* sensor23 = NULL, FrSkySportSensor* sensor24 = NULL,
                FrSkySportSensor* sensor25 = NULL, FrSkySportSensor* sensor26 = NULL, FrSkySportSensor* sensor27 = NULL,
                FrSkySportSensor* sensor28 = NULL);
    void send();
    void setData(uint8_t rssi, float rxBatt);

  private:
    FrSkySportSensor* sensors[FRSKY_TELEMETRY_MAX_SENSORS];
    FrSkySportSingleWireSerial serial;
    uint8_t  sensorCount;
    uint8_t  prevData;
    FrSkySportPolling *pollingClass;
};

#endif // _FRSKY_SPORT_TELEMETRY_H_
