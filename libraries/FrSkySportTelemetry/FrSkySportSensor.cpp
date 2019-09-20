/*
  FrSky sensor base class for Teensy 3.x/LC, ESP8266 and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20190824
  Not for commercial use
*/

#include "FrSkySportSensor.h" 

FrSkySportSensor::FrSkySportSensor(SensorId id) : sensorId(id), sensorDataIdx(0) {}
void FrSkySportSensor::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now) { }
uint16_t FrSkySportSensor::decodeData(uint8_t id, uint16_t appId, uint32_t data) { return SENSOR_NO_DATA_ID; }
