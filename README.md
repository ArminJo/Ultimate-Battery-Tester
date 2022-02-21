# [Ultimate-Battery-Tester](https://github.com/ArminJo/Ultimate-Battery-Tester)
### Version 2.0.0
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://github.com/ArminJo/Ultimate-Battery-Tester/workflows/TestCompile/badge.svg)](https://github.com/ArminJo/Ultimate-Battery-Tester/actions)
![Hit Counter](https://visitor-badge.laobi.icu/badge?page_id=ArminJo_Ultimate-Battery-Tester)


# Features
- **Measures the ESR (equivalent series resistance) of the battery.** This is an idicator of the health of the battery.
- Stores voltage and ESR graph for up to 16 hours in EEPROM while discharging.
- Current measurement or EEPROM stored measurement graph can be displayed with Arduino Plotter.
- You can continue interrupted dicharge measurements.
- Display of ESR, voltage, current and capacity on a 1602 LCD.

# Li-Ion battery capacity
For Li-Ion the capacity is specified for discharge from **4.2 V to 3.0 V** as in [CGR18650CG Datasheet](https://github.com/ArminJo/Ultimate-Battery-Tester/blob/master/CGR18650CG-Panasonic.pdf)
or to **2.75 V** as in [ICR18650-26A Datasheet](https://github.com/ArminJo/Ultimate-Battery-Tester/blob/master/ICR18650-26A_Samsung.pdf).
The UltimateBatteryTester has a **cut-off voltage of 3.5 V** for Li-Ion to treat the cells with care.<br/>
This results in a reduced capacity displayed by approximately the factor 0.85 (1.18), e.g. a Li-Ion cell with nominal capacity of 2150 mAh at 3 V EOD (End Of Discharge) is measured as 1830 mAh at 3.5 V EOD.

# Battery ESR
The internal resistance is an indicator of the health of the cell. E.g. if a NiMH cell has an ESR of **1 &ohm;**, it delivers **only 1 volt at a current of 200 mA**, which may be to low for the circuit to work properly.
ESR values for NiMH can go down to excellent 0.05 &ohm;.<br/>
Typical ESR value for a 18650 Li-Ion cell is 0.05 &ohm;.

**The ESR is not stored in EEPROM, but computed using voltage, current and the value of the constant load resistor. At low ESR values (e.g. 0.05 ohm for Li-Ion cells) a minor change in load resistor during measurement will lead to strange ESR values displayed.**

Arduino plot for a **Li-Ion cell** with nominal 2150 mAh at 3 volt. This plot is done in 2 measurements, modifying the cutoff voltage to 3.0 volt for the second measurement. The load resistor of nominal 12 &ohm; changed more than 50 m&ohm; between the measurements, resulting in strange ESR values for the second measurement.
![2155mAh_53mOhm](pictures/2155mAh_53mOhm.png)

Arduino plot for a **NiMH cell** with 120 m&ohm; ESR.
![870mAh_120mOhm](pictures/870mAh_120mOhm.png)

# Compile with the Arduino IDE
Download and extract the repository. In the Arduino IDE open the sketch with File -> Open... and select the UltimateBatteryTester folder. 

![Fritzing board](https://github.com/ArminJo/Ultimate-Battery-Tester/blob/master/extras/UltimateBatteryTester_Steckplatine.png)
![Fritzing schematics](https://github.com/ArminJo/Ultimate-Battery-Tester/blob/master/extras/UltimateBatteryTester_Schaltplan.png)

# Special pin usage
If pin 10 is connected to ground, verbose output for Arduino Serial Monitor is enabled. Verbose output is not suitable for Arduino Plotter.<br/>
If pin 11 is connected to ground, "cut off is low" is displayed and discharge ends at a lower voltage. E.g. Li-ion discharge ends at 3000 mV instead of 3500 mV.

# Modes of measurement
After reset the tester starts with mode Setup:

## Mode Setup
This mode lasts for 6 seconds before **mode InitialESRMeasurement** starts.<br/>
After boot, the tester displays its name and version and date for 2 seconds.

```
Battery Tester
2.0 MAR 1 2022
```
Then it prints the data read from EEPROM to serial monitor and displays ESR and capacity for 4 seconds.
The Arduino supply voltage (VCC) together with the message "Stored data" is displayed in the first row for 2 seconds.
Then the messages "dbl press = stop",  "cut off is high" or "low" and "Press button to append to EEPROM" are displayed for each 2 seconds.

<pre>
4.8V Stored data
0.222&ohm;   1212mAh
</pre>
<pre>
dbl press = stop
0.222&ohm;   1212mAh
</pre>

```
Press button to
append to EEPROM
```

## Mode DetectingBattery
If no battery is attached, you see:

<pre>
0.000V
0.222&ohm;  No batt.
</pre>

If a battery is attached, you see e.g.

```
1.247V
NiCd NiMH found
```
for 2 seconds and then mode is **switched to InitialESRMeasurement**.

## Mode InitialESRMeasurement
This mode lasts for 30 seconds before a new measurement is initiated and **mode is switched StoreToEEPROM**.<br/>
This 30 seconds can be used to quick check a battery, without overwriting the already stores values.<br/>
A button press during the 30 seconds **switches directly to mode StoreToEEPROM** but appends to already stored EEPROM data instead of initiating a new measurement.
This enables to connect the tester to the Arduino Serial Plotter at any time in the measurement without loosing data already acquired.
Because connecting to the Serial Plotter always resets the tester, we must be able to avoid to start a fresh measurement after reset.

<pre>
1.247V 12 331mA
0.688&ohm;   0.228V
</pre>

In the first row the **no load voltage** of the battery, the **30 second countdown** and the **load current** is displayed.
In the second row the **ESR** and the **difference between the load and no load voltage**, used to compute the ESR, is dispayed.

## Mode StoreToEEPROM
- Every second, a sample is taken and displayed.
- Every 60 seconds the sample is stored.
- Every 120 seconds 2 compressed samples are stored in EEPROM.
A button press displays `Capacity stored` for 2 seconds, writes the current capacity to EEPROM and **switches to mode Stopped**.

## Mode Stopped
The battery no load voltage is displayed in the first row.
A press of the start/stop button **switches to mode InitialESRMeasurement**.

A double press during 2 seconds always displays `Stop measurement` for 2 seconds and then **switches to mode Stopped**.


# Revision History
### Version 2.0.0
- Improved version.

### Version 1.0.0
- Initial version with EEPROM storage.

#### If you find this library useful, please give it a star.
