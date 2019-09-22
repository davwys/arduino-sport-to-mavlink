# Arduino S.Port to MAVLink Converter

## Introduction

This is an Arduino Sketch for converting FrSky S.Port telemetry data into the more common MAVLink format to be used with various groundstation applications. This should provide full telemetry as well as a functional moving map.
S.Port data should be provided from a FrSky Transmitter such as the Taranis X9D.

Note that the sketch is currently **UNTESTED**, more info coming soon.

## Installation & Testing

Full wiring instructions coming soon. S.Port input is configured for D2 port, use a 4.7kOhm resistor on the S.Port line.

Simply flash the .ino file on any Arduino (tested on Arduino Nano), don't forget to include the provided libraries.
The Arduino will then provide MAVLink telemetry data through its USB port at 57600 bauds. The code can be adapted relatively easily to provide data through other outputs (such as a bluetooth module) by changing the output functions to output on another Serial interface.

From your groundstation application, simply choose the appropriate COM port, set baud rate to 57600 and hit "connect".
If you are using Mission Planner, you might have to skip parameter fetching, as the Arduino does not output any parameter data.

## Currently supported applications

The following groundstation applications have been tested and are known to be working with this sketch:

- [x] Mission Planner (Windows)
- [x] APM Planner 2.0 (macOS/Windows)
- [ ] QGroundcontrol (macOS/Windows) -> coming soon
- [ ] Tower (Android) -> coming soon

## Known issues

- Sketch is currently untested, use at your own risk

If you find any further problems, feel free to open an issue!

## Future plans

- Expanding sensor selection
- Including attitude data (roll/pitch/yaw) if possible to have functional AHI in groundstation applications
- Add support for QGroundcontrol


(c) 2019 David Wyss

## Credits
Base Arduino to MAVLink code from https://github.com/alaney/arduino-mavlink

S.Port conversion library and sample code by Pawelsky from https://www.rcgroups.com/forums/showpost.php?p=29439177&postcount=1

