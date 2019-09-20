/*
  FrSky telemetry decoder class for Teensy 3.x/LC, ESP8266 and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20190824
  Not for commercial use
*/

#include "FrSkySportDecoder.h"

FrSkySportDecoder::FrSkySportDecoder(bool polling)
{
  if(polling == true) pollingClass = new FrSkySportPolling();
  else pollingClass = NULL; 
}

void FrSkySportDecoder::begin(FrSkySportSingleWireSerial::SerialId id,
                                FrSkySportSensor* sensor1,  FrSkySportSensor* sensor2,  FrSkySportSensor* sensor3,
                                FrSkySportSensor* sensor4,  FrSkySportSensor* sensor5,  FrSkySportSensor* sensor6,
                                FrSkySportSensor* sensor7,  FrSkySportSensor* sensor8,  FrSkySportSensor* sensor9,
                                FrSkySportSensor* sensor10, FrSkySportSensor* sensor11, FrSkySportSensor* sensor12,
                                FrSkySportSensor* sensor13, FrSkySportSensor* sensor14, FrSkySportSensor* sensor15,
                                FrSkySportSensor* sensor16, FrSkySportSensor* sensor17, FrSkySportSensor* sensor18,
                                FrSkySportSensor* sensor19, FrSkySportSensor* sensor20, FrSkySportSensor* sensor21,
                                FrSkySportSensor* sensor22, FrSkySportSensor* sensor23, FrSkySportSensor* sensor24,
                                FrSkySportSensor* sensor25, FrSkySportSensor* sensor26, FrSkySportSensor* sensor27,
                                FrSkySportSensor* sensor28)
{
  // Store sensor references in array
  sensors[0] = sensor1;
  sensors[1] = sensor2;
  sensors[2] = sensor3;
  sensors[3] = sensor4;
  sensors[4] = sensor5;
  sensors[5] = sensor6;
  sensors[6] = sensor7;
  sensors[7] = sensor8;
  sensors[8] = sensor9;
  sensors[9] = sensor10;
  sensors[10] = sensor11;
  sensors[11] = sensor12;
  sensors[12] = sensor13;
  sensors[13] = sensor14;
  sensors[14] = sensor15;
  sensors[15] = sensor16;
  sensors[16] = sensor17;
  sensors[17] = sensor18;
  sensors[18] = sensor19;
  sensors[19] = sensor20;
  sensors[20] = sensor21;
  sensors[21] = sensor22;
  sensors[22] = sensor23;
  sensors[23] = sensor24;
  sensors[24] = sensor25;
  sensors[25] = sensor26;
  sensors[26] = sensor27;
  sensors[27] = sensor28;

  // Count sensors (stops at first NULL)
  for(sensorCount = 0; sensorCount < FRSKY_DECODER_MAX_SENSORS; sensorCount++)
  {
    if(sensors[sensorCount] == NULL) break;
  }

  state = START_FRAME;
  hasStuffing = false; 

  FrSkySportDecoder::serial.begin(id);
}

uint16_t FrSkySportDecoder::decode()
{
  uint16_t result = SENSOR_NO_DATA_ID;
  if(serial.port != NULL)
  {
    if(pollingClass != NULL)
    {
      uint32_t now = millis();
      FrSkySportSensor::SensorId polledId = pollingClass->pollData(serial, now);
      if(polledId != FrSkySportSensor::ID_IGNORE) { state = DATA_FRAME; id = polledId; }
    }

    if(serial.port->available())
    {
      uint8_t byte = serial.port->read();
      if(byte == FRSKY_TELEMETRY_START_FRAME) { state = SENSOR_ID; }                               // Regardles of the state restart state machine when start frame found
      else
      {
        if(hasStuffing == true) { byte ^= 0x20; hasStuffing = false; }                             // Xor next byte with 0x20 to remove stuffing
        if((byte == FRSKY_STUFFING) && (state > DATA_FRAME) && (state <= CRC)) hasStuffing = true; // Skip stuffing byte in data and mark to xor next byte with 0x20
        else if (state == SENSOR_ID) { id = byte; state = DATA_FRAME; }                            // Store the sensor ID, start sarching for data frame
        else if((state == DATA_FRAME) && (byte == FRSKY_SENSOR_DATA_FRAME)) 
        {
          crc = byte;                                                                              // Data frame found, initialize the CRC and start collecting APP ID
          state = APP_ID_BYTE_1;
        }
        else if(state == APP_ID_BYTE_1) { ((uint8_t*)&appId)[0] = byte; state = APP_ID_BYTE_2; }   // APP ID first byte collected, look for second byte
        else if(state == APP_ID_BYTE_2) { ((uint8_t*)&appId)[1] = byte; state = DATA_BYTE_1; }     // APP ID second byte collected, store APP ID and start looking for DATA
        else if(state == DATA_BYTE_1) { ((uint8_t*)&data)[0] = byte; state = DATA_BYTE_2; }        // DATA first byte collected, look for second byte
        else if(state == DATA_BYTE_2) { ((uint8_t*)&data)[1] = byte; state = DATA_BYTE_3; }        // DATA second byte collected, look for third byte
        else if(state == DATA_BYTE_3) { ((uint8_t*)&data)[2] = byte; state = DATA_BYTE_4; }        // DATA third byte collected, look for fourth byte
        else if(state == DATA_BYTE_4) { ((uint8_t*)&data)[3] = byte; state = CRC; }                // DATA fourth byte collected, store DATA and look for CRC
        else if((state == CRC) && (byte == (0xFF - crc)))                                          // read CRC and compare with calculated one. 
        {                                                                                          // If OK, send data to registered sensors for decoding and restart the state machine.
          hasStuffing = false; 
          state = START_FRAME;
          for(uint8_t i = 0; i < sensorCount; i++) { result = sensors[i]->decodeData(id, appId, data); if(result != SENSOR_NO_DATA_ID) break; }
        }
        else { hasStuffing = false; state = START_FRAME; }

        // Update CRC value
        if((state > APP_ID_BYTE_1) && (state <= CRC) && (hasStuffing == false)) { crc += byte; crc += crc >> 8; crc &= 0x00ff; }
      }
    }
  }
  return result;
}
 