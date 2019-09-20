/*
  FrSky single wire serial class for Teensy 3.x/LC, ESP8266 and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20190824
  Not for commercial use
*/

#include "FrSkySportSingleWireSerial.h"

FrSkySportSingleWireSerial::FrSkySportSingleWireSerial()
{
  uartC3 = NULL;
  port = NULL;
#if !defined(__MK20DX128__) && !defined(__MK20DX256__) && !defined(__MKL26Z64__) && !defined(__MK66FX1M0__) && !defined(__MK64FX512__)
  softSerial = NULL;
#endif
}

void FrSkySportSingleWireSerial::begin(SerialId id)
{
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MKL26Z64__) || defined(__MK66FX1M0__) || defined(__MK64FX512__)

  if(id == SERIAL_USB) // Not really single wire, but added for debug purposes via USB
  {
    port = &Serial;
    Serial.begin(57600);
  }
  else if(id == SERIAL_1)
  {
    port = &Serial1;
    Serial1.begin(57600);
    uartC3 = &UART0_C3;
    UART0_C1 = 0xA0;  // Put Serial1 into single wire mode
    UART0_C3 = 0x10;  // Invert Serial1 Tx levels
    UART0_S2 = 0x10;  // Invert Serial1 Rx levels;
  }
  else if(id == SERIAL_2)
  {
    port = &Serial2;
    Serial2.begin(57600);
    uartC3 = &UART1_C3;
    UART1_C1 = 0xA0;  // Put Serial2 into single wire mode
    UART1_C3 = 0x10;  // Invert Serial2 Tx levels
    UART1_S2 = 0x10;  // Invert Serial2 Rx levels;
  }
  else if(id == SERIAL_3)
  {
    port = &Serial3;
    Serial3.begin(57600);
    uartC3 = &UART2_C3;
    UART2_C1 = 0xA0;  // Put Serial3 into single wire mode
    UART2_C3 = 0x10;  // Invert Serial3 Tx levels
    UART2_S2 = 0x10;  // Invert Serial3 Rx levels;
  }
  #if defined(__MK66FX1M0__) || defined(__MK64FX512__)
  else if(id == SERIAL_4)
  {
    port = &Serial4;
    Serial4.begin(57600);
    uartC3 = &UART3_C3;
    UART3_C1 = 0xA0;  // Put Serial4 into single wire mode
    UART3_C3 = 0x10;  // Invert Serial4 Tx levels
    UART3_S2 = 0x10;  // Invert Serial4 Rx levels;
  }
  else if(id == SERIAL_5)
  {
    port = &Serial5;
    Serial5.begin(57600);
    uartC3 = &UART4_C3;
    UART4_C1 = 0xA0;  // Put Serial5 into single wire mode
    UART4_C3 = 0x10;  // Invert Serial5 Tx levels
    UART4_S2 = 0x10;  // Invert Serial5 Rx levels;
  }
  else if(id == SERIAL_6)
  {
    port = &Serial6;
    Serial6.begin(57600);
    uartC3 = &UART5_C3;
    UART5_C1 = 0xA0;  // Put Serial6 into single wire mode
    UART5_C3 = 0x10;  // Invert Serial6 Tx levels
    UART5_S2 = 0x10;  // Invert Serial6 Rx levels;
  }
  #endif
#elif defined(__AVR_ATmega328P__) || defined(ESP8266)
  if(softSerial != NULL)
  {
    delete softSerial;
    softSerial = NULL;
  }
  softSerialId = id;
  softSerial = new SoftwareSerial(softSerialId, softSerialId, true);
  port = softSerial;
  softSerial->begin(57600);
#else
  #error "Unsupported processor! Only Teesny 3.x/LC, ESP8266 and ATmega328P based boards supported.";
#endif
  crc = 0;
}

void FrSkySportSingleWireSerial::setMode(SerialMode mode)
{
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MKL26Z64__) || defined(__MK66FX1M0__) || defined(__MK64FX512__)
  if((port != NULL) && (uartC3 != NULL))
  {
    if(mode == TX)
    {
        *uartC3 |= 0x20;
    }
    else if(mode == RX)
    {
        *uartC3 ^= 0x20;
    }
  }
#elif defined(__AVR_ATmega328P__) || defined(ESP8266)
  if(port != NULL)
  {
    if(mode == TX)
    {
        pinMode(softSerialId, OUTPUT);
    }
    else if(mode == RX)
    {
        pinMode(softSerialId, INPUT);
    }
  }
#endif
}

void FrSkySportSingleWireSerial::sendHeader(uint8_t id)
{
  if(port != NULL)
  {
    setMode(TX);
    port->write(FRSKY_TELEMETRY_START_FRAME);
    port->write(id);
    port->flush();
    setMode(RX);
  }
}

void FrSkySportSingleWireSerial::sendByte(uint8_t byte)
{
  if(port != NULL)
  {
    if(byte == 0x7E)
    {
      port->write(FRSKY_STUFFING);
      port->write(0x5E); // 0x7E xor 0x20
    }
    else if(byte == 0x7D)
    {
      port->write(FRSKY_STUFFING);
      port->write(0x5D); // 0x7D xor 0x20
    }
    else
    {
      port->write(byte);
    }
    crc += byte;
    crc += crc >> 8; crc &= 0x00ff;
  }
}

void FrSkySportSingleWireSerial::sendCrc()
{
  // Send and reset CRC
  sendByte(0xFF - crc);
  crc = 0;
}

void FrSkySportSingleWireSerial::sendData(uint16_t dataTypeId, uint32_t data)
{
  if(port != NULL)
  {
    setMode(TX);
    sendByte(FRSKY_SENSOR_DATA_FRAME);
    uint8_t *bytes = (uint8_t*)&dataTypeId;
    sendByte(bytes[0]);
    sendByte(bytes[1]);
    bytes = (uint8_t*)&data;
    sendByte(bytes[0]);
    sendByte(bytes[1]);
    sendByte(bytes[2]);
    sendByte(bytes[3]);
    sendCrc();
    port->flush();
    setMode(RX);
  }
}

void FrSkySportSingleWireSerial::sendEmpty(uint16_t dataTypeId)
{
  if(port != NULL)
  {
    setMode(TX);
    sendByte(0x00);
    uint8_t *bytes = (uint8_t*)&dataTypeId;
    sendByte(bytes[0]);
    sendByte(bytes[1]);
    for(uint8_t i = 0; i < 4; i++) sendByte(0x00);
    sendCrc();
    port->flush();
    setMode(RX);
  }
}
