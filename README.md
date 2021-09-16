# [Ultimate-Battery-Tester](https://github.com/ArminJo/Ultimate-Battery-Tester)
### Version 1.0.0
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://github.com/ArminJo/Ultimate-Battery-Tester/workflows/TestCompile/badge.svg)](https://github.com/ArminJo/Ultimate-Battery-Tester/actions)
[![Hit Counter](https://hitcounter.pythonanywhere.com/count/tag.svg?url=https%3A%2F%2Fgithub.com%2FArminJo%2FUltimate-Battery-Tester)](https://github.com/brentvollebregt/hit-counter)

## Features
- **Measures the ESR (equivalent series resistance) of the battery.** This is an idicator of the health of the battery.
- Stores voltage and ESR graphim EEPROM while discharging.
- Current measurement or EEPROM stored measurement graph can be displayed with Arduino Plotter.
- You can continue interrupted dichare easurements.
- Display of current esr, voltage, current and capacity on a 1602 LCD.

## Battery ESR
The internal resistance is an idicator of the health of the battery. E.g. if a NiMH battery has an ESR of **1 ohm**, it delivers **only 1 volt at a current of 200 mA**, which may be to low for the circuit to work properly.
ESR values for NiMH can go down to excellent 0.05 ohm.

## Compile with the Arduino IDE
Download and extract the repository. In the Arduino IDE open the sketch with File -> Open... and select the src/SBMInfo folder. 

# Revision History
### Version 1.0.0
- Initial version with EEPROM storage.

#### If you find this library useful, please give it a star.
