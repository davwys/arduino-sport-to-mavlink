/*
  FrSky ASS-70/ASS-100 airspeed sensor class for Teensy 3.x/LC, ESP8266 and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20190824
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_ASS_H_
#define _FRSKY_SPORT_SENSOR_ASS_H_

#include "FrSkySportSensor.h"

#define ASS_DEFAULT_ID ID10
#define ASS_SPEED_DATA_ID 0x0A00

#define ASS_SPEED_DATA_PERIOD 500

class FrSkySportSensorAss : public FrSkySportSensor
{
  public:
    FrSkySportSensorAss(SensorId id = ASS_DEFAULT_ID);
    void setData(float speed);
    virtual void send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now);
    virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);
    float getSpeed();

  private:
    uint32_t speedData;
    uint32_t speedTime;
    float speed;
};

#endif // _FRSKY_SPORT_SENSOR_ASS_H_
