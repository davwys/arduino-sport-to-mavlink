/*
  FrSky sensor data polling class for Teensy 3.x/LC, ESP8266 and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20190824
  Not for commercial use
*/

#include "FrSkySportPolling.h"

FrSkySportPolling::FrSkySportPolling()
{
  nextPollIdIdx = 0;
  nextPollTime = 0;
  receiverDataIdx = 0;
  receiverDataPollTime = 0;
}

FrSkySportSensor::SensorId FrSkySportPolling::getNextId()
{
  if(nextPollIdIdx >= FRSKY_POLLIED_ID_COUNT) nextPollIdIdx = 0;
  return POLLED_ID_TABLE[nextPollIdIdx++];
}

FrSkySportSensor::SensorId FrSkySportPolling::pollData(FrSkySportSingleWireSerial& serial, uint32_t now)
{
  FrSkySportSensor::SensorId id = FrSkySportSensor::ID_IGNORE;
  if(serial.port != NULL)
  {
    // Send receiver data (RSSI followed by RxBatt) every 444ms independent of other IDs
    if(now >= receiverDataPollTime)
    {
      serial.sendHeader(FrSkySportSensor::ID25);
      switch(receiverDataIdx)
      {
        case 0 :
          serial.sendData(POLLING_RSSI_DATA_ID, rssiData);
          receiverDataPollTime = now + POLLING_ID_POLL_TIME;
          break;
        case 1 :
          serial.sendData(POLLING_RXBATT_DATA_ID, rxBattData);
          receiverDataPollTime = now + POLLING_RECEIVER_DATA_POLL_TIME;
          break;
      }
      receiverDataIdx++;
      if(receiverDataIdx >= POLLING_RECEIVER_DATA_COUNT) receiverDataIdx = 0;
      nextPollTime = now + POLLING_ID_POLL_TIME;
    }
    // Poll next ID every 12ms
    else if(now >= nextPollTime)
    {
      id = getNextId();
      serial.sendHeader(id);
      nextPollTime = now + POLLING_ID_POLL_TIME;
    }
  }

  return id;
}

void FrSkySportPolling::setData(uint8_t rssi, float rxBatt)
{
  if(rssi > 100) rssi = 100;
  rssiData = (uint32_t)rssi;
  if(rxBatt > 13.2) rxBatt = 13.2; else if (rxBatt < 0.0) rxBatt = 0.0;
  rxBattData = (uint32_t)(rxBatt * 255.0 / 3.3 / 4.0);
}
