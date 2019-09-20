/*
  FrSky S-Port Telemetry library example
  (c) Pawelsky 20190824
  Not for commercial use
  
  Note that you need Teensy 3.x/LC, ESP8266 or ATmega328P based (e.g. Pro Mini, Nano, Uno) board and FrSkySportTelemetry library for this example to work
*/

// Uncomment the #define below to enable internal polling of data.
// Use only when there is no device in the S.Port chain (e.g. S.Port capable FrSky receiver) that normally polls the data.
//#define POLLING_ENABLED

#include "FrSkySportSensor.h"
#include "FrSkySportSensorAss.h"
#include "FrSkySportSensorFcs.h"
#include "FrSkySportSensorFlvss.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorRpm.h"
#include "FrSkySportSensorSp2uart.h"
#include "FrSkySportSensorVario.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportTelemetry.h"
#if !defined(__MK20DX128__) && !defined(__MK20DX256__) && !defined(__MKL26Z64__) && !defined(__MK66FX1M0__) && !defined(__MK64FX512__)
#include "SoftwareSerial.h"
#endif

FrSkySportSensorAss ass;                               // Create ASS sensor with default ID
FrSkySportSensorFcs fcs;                               // Create FCS-40A sensor with default ID (use ID8 for FCS-150A)
FrSkySportSensorFlvss flvss1;                          // Create FLVSS sensor with default ID
FrSkySportSensorFlvss flvss2(FrSkySportSensor::ID15);  // Create FLVSS sensor with given ID
FrSkySportSensorGps gps;                               // Create GPS sensor with default ID
FrSkySportSensorRpm rpm;                               // Create RPM sensor with default ID
FrSkySportSensorSp2uart sp2uart;                       // Create SP2UART Type B sensor with default ID
FrSkySportSensorVario vario;                           // Create Variometer sensor with default ID
#ifdef POLLING_ENABLED
  FrSkySportTelemetry telemetry(true);                 // Create telemetry object with polling
#else
  FrSkySportTelemetry telemetry;                       // Create telemetry object without polling
#endif

void setup()
{
  // Configure the telemetry serial port and sensors (remember to use & to specify a pointer to sensor)
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MKL26Z64__) || defined(__MK66FX1M0__) || defined(__MK64FX512__)
  telemetry.begin(FrSkySportSingleWireSerial::SERIAL_3, &ass, &fcs, &flvss1, &flvss2, &gps, &rpm, &sp2uart, &vario);
#else
  telemetry.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_12, &ass, &fcs, &flvss1, &flvss2, &gps, &rpm, &sp2uart, &vario);
#endif
}

void loop()
{
  // Set airspeed sensor (ASS) data
  ass.setData(76.5);  // Airspeed in km/h

  // Set current/voltage sensor (FCS) data
  // (set Voltage source to FAS in menu to use this data for battery voltage,
  //  set Current source to FAS in menu to use this data for current readins)
  fcs.setData(25.3,   // Current consumption in amps
              12.6);  // Battery voltage in volts

  // Set LiPo voltage sensor (FLVSS) data (we use two sensors to simulate 8S battery 
  // (set Voltage source to Cells in menu to use this data for battery voltage)
  flvss1.setData(4.07, 4.08, 4.09, 4.10, 4.11, 4.12);  // Cell voltages in volts (cells 1-6)
  flvss2.setData(4.13, 4.14);                          // Cell voltages in volts (cells 7-8)

  // Set GPS data
  gps.setData(48.858289, 2.294502,  // Latitude and longitude in degrees decimal (positive for N/E, negative for S/W)
              245.5,                // Altitude in m (can be negative)
              100.0,                // Speed in m/s
              90.23,                // Course over ground in degrees (0-359, 0 = north)
              14, 9, 14,            // Date (year - 2000, month, day)
              12, 00, 00);          // Time (hour, minute, second) - will be affected by timezone setings in your radio

  // Set RPM/temperature sensor data
  // (set number of blades to 2 in telemetry menu to get correct rpm value)
  rpm.setData(200,    // Rotations per minute
              25.6,   // Temperature #1 in degrees Celsuis (can be negative, will be rounded)
              -7.8);  // Temperature #2 in degrees Celsuis (can be negative, will be rounded)

  // Set SP2UART sensor data
  // (values from 0.0 to 3.3 are accepted)
  sp2uart.setData(1.5,   // ADC3 voltage in volts
                  3.3);  // ADC4 voltage in volts

  // Set variometer data
  // (set Variometer source to VSpd in menu to use the vertical speed data from this sensor for variometer).
  vario.setData(250.5,  // Altitude in meters (can be negative)
                -1.5);  // Vertical speed in m/s (positive - up, negative - down)

#ifdef POLLING_ENABLED
  // Set receiver data to be sent in case the polling is enabled (so no actual receiver is used)
  telemetry.setData(90,    // RSSI value (0-100, 0 = no telemetry, 100 = full signal)
                    4.9);  // RxBatt (voltage supplied to the receiver) value in volts (0.0-13.2)
#endif

  // Send the telemetry data, note that the data will only be sent for sensors
  // that are being polled at given moment
  telemetry.send();}