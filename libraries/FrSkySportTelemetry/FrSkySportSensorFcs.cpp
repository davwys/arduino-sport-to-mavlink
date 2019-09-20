/*
  FrSky FCS-40A/FCS-150A current sensor class for Teensy 3.x/LC, ESP8266 and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20190824
  Not for commercial use
*/

#include "FrSkySportSensorFcs.h" 

FrSkySportSensorFcs::FrSkySportSensorFcs(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorFcs::setData(float current, float voltage)
{
  currentData = (uint32_t)(current * 10);
  voltageData = (uint32_t)(voltage * 100);
}

void FrSkySportSensorFcs::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
      case 0:
        if(now > currentTime)
        {
          currentTime = now + FCS_CURR_DATA_PERIOD;
          serial.sendData(FCS_CURR_DATA_ID, currentData);
        }
        else
        {
          serial.sendEmpty(FCS_CURR_DATA_ID);
        }
        break;
      case 1:
        if(now > voltageTime)
        {
          voltageTime = now + FCS_VOLT_DATA_PERIOD;
          serial.sendData(FCS_VOLT_DATA_ID, voltageData);
        }
        else
        {
          serial.sendEmpty(FCS_VOLT_DATA_ID);
        }
        break;
    }
    sensorDataIdx++;
    if(sensorDataIdx >= FCS_DATA_COUNT) sensorDataIdx = 0;
  }
}

uint16_t FrSkySportSensorFcs::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  if((sensorId == id) || (sensorId == FrSkySportSensor::ID_IGNORE))
  {
    switch(appId)
    {
      case FCS_CURR_DATA_ID:
        current = data / 10.0;
        return appId;
      case FCS_VOLT_DATA_ID:
        voltage = data / 100.0;
        return appId;
    }
  }
  return SENSOR_NO_DATA_ID;
}

float FrSkySportSensorFcs::getCurrent() { return current; }
float FrSkySportSensorFcs::getVoltage() { return voltage; }
