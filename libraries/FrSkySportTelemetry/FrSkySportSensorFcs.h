/*
  FrSky FCS-40A/FCS-150A current sensor class for Teensy 3.x/LC, ESP8266 and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20190824
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_FCS_H_
#define _FRSKY_SPORT_SENSOR_FCS_H_

#include "FrSkySportSensor.h"

#define FCS_DEFAULT_ID ID3 // ID3 is the default for FCS-40A sensor, for FCS-150A use ID8.
#define FCS_DATA_COUNT 2
#define FCS_CURR_DATA_ID 0x0200
#define FCS_VOLT_DATA_ID 0x0210

#define FCS_CURR_DATA_PERIOD 500
#define FCS_VOLT_DATA_PERIOD 500

class FrSkySportSensorFcs : public FrSkySportSensor
{
  public:
    FrSkySportSensorFcs(SensorId id = FCS_DEFAULT_ID);
    void setData(float current, float voltage);
    virtual void send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now);
    virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);
    float getCurrent();
    float getVoltage();

  private:
    uint32_t currentData;
    uint32_t voltageData;
    uint32_t currentTime;
    uint32_t voltageTime;
    float current;
    float voltage;
};

#endif // _FRSKY_SPORT_SENSOR_FCS_H_
