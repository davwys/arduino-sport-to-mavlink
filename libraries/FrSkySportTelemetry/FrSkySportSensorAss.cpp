/*
  FrSky ASS-70/ASS-100 airspeed sensor class for Teensy 3.x/LC, ESP8266 and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20190824
  Not for commercial use
*/

#include "FrSkySportSensorAss.h" 

FrSkySportSensorAss::FrSkySportSensorAss(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorAss::setData(float speed)
{
  speedData = (uint32_t)(speed * 10.0);
}

void FrSkySportSensorAss::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  if(sensorId == id)
  {
    if(now > speedTime)
    {
      speedTime = now + ASS_SPEED_DATA_PERIOD;
      serial.sendData(ASS_SPEED_DATA_ID, speedData);
    }
    else
    {
      serial.sendEmpty(ASS_SPEED_DATA_ID);
    }
  }
}

uint16_t FrSkySportSensorAss::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  if((sensorId == id) || (sensorId == FrSkySportSensor::ID_IGNORE))
  {
    if(appId == ASS_SPEED_DATA_ID)
    {
      speed = data / 10.0;
      return appId;
    }
  }
  return SENSOR_NO_DATA_ID;
}

float FrSkySportSensorAss::getSpeed() { return speed; }
