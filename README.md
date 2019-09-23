# Arduino S.Port to MAVLink Converter

## Introduction

This is an Arduino Sketch for converting FrSky S.Port telemetry data into the more common MAVLink format to be used with various groundstation applications. This should provide full telemetry as well as a functional moving map.
S.Port data should be provided from a FrSky Transmitter such as the Taranis X9D.

Note that the sketch is currently in Beta state, as there is still more testing to be done.

## Wiring

Here's how to wire up an Arduino Nano for this project. Note that for other Arduinos, pinouts may vary.
This graph assumes the arduino is powered with 5V via USB - you can power it directly from the TX module as well, but make sure the voltage matches what your Arduino supports!
Full Bluetooth module support coming soon!

![Alt text](wiring.jpg?raw=true "Title")

You do not need to use a TX module for this to work, you can also connect the Arduino directly to the module bay pins if using the internal module. Check Google for the correct module bay pinout.

Tested with:
- [x] FrSky R9M
- [ ] FrSky XJT Module (coming soon)
- [ ] Taranis Module Bay

## Installation & Testing

Simply flash the .ino file on any Arduino (tested on Arduino Nano), don't forget to include the provided libraries.
The Arduino will then provide MAVLink telemetry data through its USB port at 115200 bauds. The code can be adapted relatively easily to provide data through other outputs (such as a bluetooth module) by changing the output functions to output on another Serial interface. Included Bluetooth module support coming soon!

From your groundstation application, simply choose the appropriate COM port, set baud rate to 57600 and hit "connect".
If you are using Mission Planner, you might have to skip parameter fetching, as the Arduino does not output any parameter data.



## Currently supported applications

The following groundstation applications have been tested and are known to be working with this sketch:

- [x] Mission Planner (Windows)
- [x] APM Planner 2.0 (macOS/Windows)
- [ ] QGroundcontrol (macOS/Windows) -> coming soon
- [ ] Tower (Android) -> coming soon

## Known issues

- Not all sensors currently supported

If you find any further problems, feel free to open an issue!

## Future plans

- Expanding sensor selection
- Including attitude data (roll/pitch/yaw) if possible to have functional AHI in groundstation applications
- Add support for QGroundcontrol


(c) 2019 David Wyss

## Credits
Base Arduino to MAVLink code from https://github.com/alaney/arduino-mavlink

S.Port conversion library and sample code by Pawelsky from https://www.rcgroups.com/forums/showpost.php?p=29439177&postcount=1

