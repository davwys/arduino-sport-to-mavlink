/*
  FrSky S-Port Telemetry Decoder library example for the XJT decoder class (old hub telemetry and basic RSSI/ADC1/ADC2/RxBatt/SWR data)
  (c) Pawelsky 20190824
  Not for commercial use
  
  Note that you need Teensy 3.x/LC, ESP8266 or ATmega328P based (e.g. Pro Mini, Nano, Uno) board and FrSkySportDecoder library for this example to work
*/

// Uncomment the #define below to enable internal polling of data.
// Use only when there is no device in the S.Port chain (e.g. S.Port capable FrSky receiver) that normally polls the data.
//#define POLLING_ENABLED

#include "FrSkySportSensor.h"
#include "FrSkySportSensorXjt.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportDecoder.h"
#if !defined(__MK20DX128__) && !defined(__MK20DX256__) && !defined(__MKL26Z64__) && !defined(__MK66FX1M0__) && !defined(__MK64FX512__)
#include "SoftwareSerial.h"
#endif

FrSkySportSensorXjt xjt;            // Create XJT sensor with default ID
#ifdef POLLING_ENABLED
  FrSkySportDecoder decoder(true);  // Create decoder object with polling
#else
  FrSkySportDecoder decoder;        // Create decoder object without polling
#endif


uint32_t currentTime, displayTime;
uint16_t decodeResult;

void setup()
{
  // Configure the decoder serial port and sensors (remember to use & to specify a pointer to sensor)
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MKL26Z64__) || defined(__MK66FX1M0__) || defined(__MK64FX512__)
  decoder.begin(FrSkySportSingleWireSerial::SERIAL_3, &xjt);
#else
  decoder.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_12, &xjt);
#endif
  Serial.begin(115200);
}

void loop()
{
  // Read and decode the telemetry data, note that the data will only be decoded for sensors
  // that that have been passed to the begin method. Print the AppID of the decoded data.
  decodeResult = decoder.decode();
  if(decodeResult != SENSOR_NO_DATA_ID) { Serial.print("Decoded data with AppID 0x"); Serial.println(decodeResult, HEX); }
  
  // Display data once a second to not interfeere with data decoding
  currentTime = millis();
  if(currentTime > displayTime)
  {
    displayTime = currentTime + 1000;

    Serial.println("");

    // Get basic XJT data (RSSI/ADC1/ADC2/RxBatt/SWR data)
    Serial.print("Basic: RSSI = "); Serial.print(xjt.getRssi()); // RSSI
    Serial.print(", ADC1 = "); Serial.print(xjt.getAdc1());      // ADC1 voltage in volts
    Serial.print("V, ADC2 = "); Serial.print(xjt.getAdc2());     // ADC2 voltage in volts
    Serial.print("V, RxBatt = "); Serial.print(xjt.getRxBatt()); // RxBatt voltage in volts
    Serial.print("V, SWR = "); Serial.println(xjt.getSwr());     // SWR

    // Get fuel sensor (FGS) data
    Serial.print("FGS: fuel = "); Serial.print(xjt.getFuel()); Serial.println("%"); // Fuel level in percent

    // Get LiPo voltage sensor (FLVS) data (each cell1-cell12 voltage in volts)
    Serial.print("FLVS: cell1 = "); Serial.print(xjt.getCell1()); Serial.print("V, cell2 = "); Serial.print(xjt.getCell2());
    Serial.print("V, cell3 = "); Serial.print(xjt.getCell3()); Serial.print("V, cell4 = "); Serial.print(xjt.getCell4());
    Serial.print("V, cell5 = "); Serial.print(xjt.getCell5()); Serial.print("V, cell6 = "); Serial.print(xjt.getCell6()); 
    Serial.print("V, cell7 = "); Serial.print(xjt.getCell7()); Serial.print("V, cell8 = "); Serial.print(xjt.getCell8());
    Serial.print("V, cell9 = "); Serial.print(xjt.getCell9()); Serial.print("V, cell10 = "); Serial.print(xjt.getCell10());
    Serial.print("V, cell11 = "); Serial.print(xjt.getCell11()); Serial.print("V, cell12 = "); Serial.print(xjt.getCell12()); Serial.println("V"); 

    // Get current/voltage sensor (FAS) data
    Serial.print("FAS: current = "); Serial.print(xjt.getCurrent());                    // Current consumption in amps
    Serial.print("A, voltage = "); Serial.print(xjt.getVoltage()); Serial.println("V"); // Battery voltage in volts

    // Get variometer sensor (FVAS) data
    Serial.print("FVAS: altitude = "); Serial.print(xjt.getAltitude()); Serial.print("m"); // Altitude in m (can be nevative)
    Serial.print("m, VSI = "); Serial.print(xjt.getVsi()); Serial.println("m/s");          // Verticas speed in m/s (can be nevative)

    // Get GPS data
    Serial.print("GPS: lat = "); Serial.print(xjt.getLat(), 6); Serial.print(", lon = "); Serial.print(xjt.getLon(), 6); // Latitude and longitude in degrees decimal (positive for N/E, negative for S/W)
    Serial.print(", altitude = "); Serial.print(xjt.getAltitude()); // Altitude in m (can be negative)
    Serial.print("m, speed = "); Serial.print(xjt.getSpeed()); // Speed in m/s
    Serial.print("m/s, COG = "); Serial.print(xjt.getCog());   // Course over ground in degrees (0-359, 0 = north)
    char dateTimeStr[18]; 
    sprintf(dateTimeStr, "%02u-%02u-%04u %02u:%02u:%02u", xjt.getDay(), xjt.getMonth(), xjt.getYear() + 2000, xjt.getHour(), xjt.getMinute(), xjt.getSecond());
    Serial.print(", date/time = "); Serial.println(dateTimeStr); // Date (year - need to add 2000, month, day) and time (hour, minute, second)

    // Get accelerometer sensor (TAS) data
    Serial.print("TAS: ACCX = "); Serial.print(xjt.getAccX());                    // X axis acceleraton in Gs (can be negative)
    Serial.print("G, ACCY = "); Serial.print(xjt.getAccY());                      // Y axis acceleraton in Gs (can be negative)
    Serial.print("G, ACCZ = "); Serial.print(xjt.getAccZ()); Serial.println("G"); // Z axis acceleraton in Gs (can be negative)
    
    // Get temperature sensor (TEMS) data
    Serial.print("TEMS: T1 = "); Serial.print(xjt.getT1());                               // Temperature #1 in degrees Celsius (can be negative, will be rounded)
    Serial.print(" deg. C, T2 = "); Serial.print(xjt.getT1()); Serial.println(" deg. C"); // Temperature #2 in degrees Celsius (can be negative, will be rounded)

    // Get RPM sensor (RPMS) data
    // (set number of blades to 2 in telemetry menu to get correct rpm value)
    Serial.print("RPMS: RPM = "); Serial.println(xjt.getRpm()); // Rotations per minute 

    Serial.println("");
  }
}