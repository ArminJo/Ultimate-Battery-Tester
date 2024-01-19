<div align = center>

# Ultimate-Battery-Tester
Program for measuring the ESR (equivalent series resistance) of a battery and printing a graph of the values at discharge.

[![Badge License: GPLv3](https://img.shields.io/badge/License-GPLv3-brightgreen.svg)](https://www.gnu.org/licenses/gpl-3.0)
 &nbsp; &nbsp;
[![Badge Version](https://img.shields.io/github/v/release/ArminJo/Ultimate-Battery-Tester?include_prereleases&color=yellow&logo=DocuSign&logoColor=white)](https://github.com/ArminJo/Ultimate-Battery-Tester/releases/latest)
 &nbsp; &nbsp;
[![Badge Commits since latest](https://img.shields.io/github/commits-since/ArminJo/Ultimate-Battery-Tester/latest?color=yellow)](https://github.com/ArminJo/Ultimate-Battery-Tester/commits/master)
 &nbsp; &nbsp;
[![Badge Build Status](https://github.com/ArminJo/Ultimate-Battery-Tester/workflows/TestCompile/badge.svg)](https://github.com/ArminJo/Ultimate-Battery-Tester/actions)
 &nbsp; &nbsp;
![Badge Hit Counter](https://visitor-badge.laobi.icu/badge?page_id=ArminJo_Ultimate-Battery-Tester)
<br/>
<br/>
[![Stand With Ukraine](https://raw.githubusercontent.com/vshymanskyy/StandWithUkraine/main/badges/StandWithUkraine.svg)](https://stand-with-ukraine.pp.ua)

</div>

#### If you find this library useful, please give it a star.

&#x1F30E; [Google Translate](https://translate.google.com/translate?sl=en&u=https://github.com/ArminJo/Ultimate-Battery-Tester)

<br/>

# Features
- **Measures the ESR (equivalent series resistance) of the battery.** This is an indicator of the health of the battery.
- **Stores voltage, current and ESR graph for up to 11 hours as well as capacity in EEPROM** while discharging.
- Current measurement or EEPROM stored measurement graph can be displayed with Arduino Plotter.
- Display of no load voltage to be independence from load (resistor).
- Easy **continuing of interrupted discharge measurements**.
- Display of ESR, voltage, current and capacity on a **1602 LCD**.
- Computes "Standard" capacity between NominalFullVoltageMillivolt and SwitchOffVoltageMillivoltHigh to enable better comparison.
- Supports **2 different load resistors** for different battery voltages to keep current below 600 mA.
- Supports battery voltages up to 20 volt (@5V Arduino VCC) and external load resistor e.g. for measuring of battery packs.

<br/>

# Li-Ion battery capacity
For Li-Ion the capacity is specified for discharge from **4.2 V to 3.0 V** as in [CGR18650CG Datasheet](https://github.com/ArminJo/Ultimate-Battery-Tester/blob/master/CGR18650CG-Panasonic.pdf)
or to **2.75 V** as in [ICR18650-26A Datasheet](https://github.com/ArminJo/Ultimate-Battery-Tester/blob/master/ICR18650-26A_Samsung.pdf).
The UltimateBatteryTester has a **cut-off voltage of 3.5 V** for Li-Ion to treat the cells with care.<br/>
This results in a reduced capacity displayed by approximately the factor 0.85 (1.18), e.g. a Li-Ion cell with nominal capacity of 2150 mAh at 3 V EOD (End Of Discharge) is measured as 1830 mAh at 3.5 V EOD.<br/>
The cut off voltage can be changed to low values by connecting pin 11 to ground. See [here](#special-pin-usage).

<br/>

# Battery ESR
The internal resistance is an indicator of the health of the cell. E.g. if a NiMH cell has an ESR of **1 &ohm;**, it delivers **only 1 volt at a current of 200 mA**, which may be to low for the circuit to work properly.
ESR values for NiMH can go down to excellent 0.05 &ohm;.<br/>
Typical ESR value for a 18650 Li-Ion cell is 0.05 &ohm;.

It seems that the **dynamic ESR** measured by devices like **YR1035+** is around half as much as the **?static? ESR** measured by this program.
This was suprising for me, since I expected only a fixed offset, because of connection imperfections.

# Sample plots
The plots are created with the Arduino 1.x Serial Plotter. The Arduino 2.x Serial Plotter is not as powerful and uses a different data format.

Plot for **2 parallel Li-Ion cells**.<br/>
The first capacity value is the "standard" capacity measured from the first peak at standard full voltage 4100 mV to the second peak, the cutoff "high" voltage at 3400 mV.<br/>
The peaks, which are 50 mV added to the measured value, are artificially generated for easy recognition of the capacity range.<br/>
The second peak is always suppressed for graphs with discharge to "high", because the end of this graph is by definition the end of the standard capacity range.<br/>
The second capacity value is the total capacity measured in this graph.<br/>
**The displayed voltage is the "no load" voltage**, to be independent of the current load resistor.
![2Li-ion_toLow](pictures/2Li-ion_toLow.png)

Plot a **worn out single Li-ion cell** with increased ESR and a more flat curve at 3.4 V.
![Li-ion_toLow](pictures/Li-ion_ToLow.png)

Plot for a **Li-Ion cell** with accidentally setting the cutoff voltage to zero (0.5 volt) (older graph version without peaks).
![Discharge to zero](pictures/Li-ion_ToZero_QUIXIN_21A21.png)

Plot for a **NiMH cell** with 55 m&ohm; ESR.
![870mAh_120mOhm](pictures/1275mAh_55mOhm.png)

Plot of a NiMh battery sold as 4/5AA 1800mAh Ni-Mh proving only 960 mAh.
 ![1800mAh](pictures/1800NiMhBattery.png)
 
<br/>

# Principle of operation
The battery type is detected by a fixed mapping of voltage to type in `BatteryTypeInfoArray[]`.
While the Mosfet is switched on, the voltage at the 2 &ohm; shunt resistor is measured to get the current. The voltage at the battery terminal is measured to get the voltage under load.<br/>
Every second, the Mosfet is deactivated for 10 ms or 100 ms (depending on battery type), the "no load" voltage at the battery terminal is measured and the Mosfet is switched on again.<br/>
The internal resistance can now be computed from the difference of the load and the no load voltage and the difference of the currents (measured mA and 0 mA).

Every minute, current data is stored/appended to EEPROM. **The complete data is printed in Arduino Plotter format at each reboot**.
So it is possible to interrupt the measurement or switch it off after the measurement is finished, without loosing data.
More details can be found [below](#modes-of-measurement).

<br/>

# Measurement of battery packs with external series load resistor
Battery packs up to 17.2 volt (4s) can be measured too. Voltages above 14.8 volt require a 5 volt supply for the arduino internal ADC.
Given the voltage measurement resistor network from the schematic, with Li-ion (3.7 V VCC) we can merely measure up to 14.8 V.<br/>
Since the build in load resistor is 12 &ohm;, **the current would go up to 1.4 ampere and the power to 24 watt**, leaving 2.8 watt at the 2 &ohm; shunt resistors.<br/>
This is too much for the resistors I used for shunt!<br/>
The solution is to **add an additional resistor of around 20 &ohm; in series to the 10 &ohm; already built in one**.
This reduces the current to around 500 mA and power to 9 watt leaving 1 watt at the 2 &ohm; shunt resistors.<br/>
The voltage must still be measured at the battery terminal, so I use a distinct cable for voltage measurement, normally connected to the built in load resistors / battery + cable.<br/>
No other adaption has to be made.

<br/>

# Compile with the Arduino IDE
Download and extract the repository. In the Arduino IDE open the sketch with File -> Open... and select the UltimateBatteryTester folder.

# Pictures

| Overview with distinct voltage measurement cable (thin red one) to enable additional series resistors for battery packs | Top View |
|-|-|
| ![Overview](pictures/Overview.jpg) | ![Top View](pictures/TopView.jpg) |
| MosFets | Reset and application sensor button |
| ![MosFets](pictures/MosFets.jpg) | ![Reset and application sensor button](pictures/Buttons.jpg) |
| Battery holder top view | Battery holder bottom view |
| ![LCD Battery holder top view](pictures/BatteryHolderTop.jpg) | ![Battery holder bottom view](pictures/BatteryHolderBottom.jpg) |

# Schematics
![Fritzing board](https://github.com/ArminJo/Ultimate-Battery-Tester/blob/master/extras/UltimateBatteryTester_Steckplatine.png)
![Fritzing schematics](https://github.com/ArminJo/Ultimate-Battery-Tester/blob/master/extras/UltimateBatteryTester_Schaltplan.png)

# Special pin usage
If pin 10 is connected to ground, verbose output for Arduino Serial Monitor is enabled. Verbose output is not suitable for Arduino Plotter.<br/>
If pin 11 is connected to ground, "cut off is low" is displayed and discharge ends at a lower voltage. E.g. Li-ion discharge ends at 3000 mV instead of 3500 mV.

# Examples on Wokwi
The screenshots below are taken from this [Wokwi example](https://wokwi.com/projects/381051341790948353).

![Wokwi](pictures/Wokwi.png)

# Modes of measurement
After reset the tester starts with mode Setup:

## Mode Setup
After boot, the tester displays its name and version and date for 2 seconds.

![BatteryTester](pictures/BatteryTester.png)

The message `No plotter out` is displayed in the second row for 2 seconds.
If pin `PIN_ONLY_PLOTTER_OUTPUT` (pin 10) is held low, then the message `Only plotter out` is displayed.

![NoPlotterOut](pictures/NoPlotterOut.png)
![OnlyPlotterOut](pictures/OnlyPlotterOut.png)

Then it prints the data read from EEPROM to serial monitor and displays ESR and capacity.
The Arduino supply voltage (VCC) together with the message `Stored data` is displayed in the first row for 2 seconds.<br/>
The character `h` or `l` or `z` shows the cutoff voltage of the stored data.

![StoredData](pictures/StoredData.png)

Then the message `cutoff high` or `low`  and the cutoff voltage is displayed in the first row for 2 seconds.<br/>
If battery is still inserted, it reflects the cutoff voltage of the stored data, to enable easy **continuing an interrupted measurement**.<br/>
If no battery is inserted, this cutoff message reflects the state of the pin `PIN_DISCHARGE_TO_LOW`.

![CutoffHigh](pictures/CutoffHigh.png)
![CutoffLow](pictures/CutoffLow.png)

After this, **mode is switched to DetectingBattery**.<br/>

## Stop and Start again
A double press during 0.4 seconds always displays `Stop measurement` for 2 seconds and then **mode is switched to Stopped**.<br/>
Another press will start again.
`Stopped B` means stopped by **button** press, `Stopped D` means stopped by button **double** press, `Stopped F` means stopped by EEPROM **full**,
`Stopped -` means stopped by reaching the cutoff voltage and `Stopped U` means stopped by VCC undervoltage.

![StopMeasurement](pictures/StopMeasurement.png)
![Stopped](pictures/Stopped_D.png)

![StartAgain](pictures/StartAgain.png)

## Mode DetectingBattery
If no battery is inserted, the Arduino supply voltage (VCC) together with the message `No batt.` is displayed in the first row until a battery is inserted.<br/>

![NoBatt](pictures/NoBatt.png)

By pressing the stop button, you can toggle cutoff voltage between high, low and zero (50 mV).

![NoBatt](pictures/CutoffIsHigh.png)
![NoBatt](pictures/CutoffIsLow.png)
![NoBatt](pictures/CutoffIsZero.png)

If a battery is inserted, you see e.g.

![LiIoFound](pictures/LiIoFound.png)

for 2 seconds.
Then the messages `dbl press = stop`,  and `Press button to append to EEPROM` are displayed for 2 seconds each, but only once after boot.

![DblPress](pictures/DblPress.png)
![PressButtonToAppend](pictures/PressButtonToAppend.png)

After this, the mode is **switched to InitialESRMeasurement**.

## Mode InitialESRMeasurement
This mode lasts for 30 seconds before a new measurement is initiated and **mode is switched StoreToEEPROM**.<br/>
This 30 seconds can be used to quick check a battery, without overwriting the already stores values.<br/>
A button press during the 30 seconds **switches directly to mode StoreToEEPROM and appends to already stored EEPROM data**, instead of starting a new measurement.
This enables it to connect the tester to the Arduino Serial Plotter at any time in the measurement without loosing data already acquired.
Because connecting to the Serial Plotter always resets the tester, we must be able to avoid to start a fresh measurement after reset.

![InitialESRMeasurement](pictures/InitialESRMeasurement.png)

In the first row the **no load voltage** of the battery, the **30 second countdown** and the **load current** is displayed.
In the second row the **ESR** and the **difference between the load and no load voltage**, used to compute the ESR, is displayed.<br/>
A value of **99.999 &ohm; indicates overflow** over the maximum value of 65.535 &ohm;.

## Mode StoreToEEPROM
- Every second, a sample is taken and displayed.
- Every 60 seconds the sample is stored.
- For the first 337 samples (5:37 hours) each 8 bit delta is stored to EEPROM.
- After the first 337 samples, all data are compressed, and every 120 seconds 2 compressed samples are stored to EEPROM.
- The number between the voltage and the current in the first row is the virtual EEPROM storage index and incremented at each storage.

![Storing](pictures/Storing.png)

If the **no load voltage drops below the cut off voltage** or the start/stop button is pressed, `Capacity stored` is displayed for 2 seconds,
the current capacity is written to EEPROM and **mode is switched to Stopped**.

![CapacityStored](pictures/CapacityStored.png)

## Mode Stopped
The battery no load voltage is displayed in the first row. A stop melody is played and `Finished` is displayed **after reaching the cutoff voltage**.<br/>
`Stopped B` is displayed after manual stopping by button press.<br/>

![Finished](pictures/Finished.png)
![Stopped by button single press](pictures/Stopped_B.png)

After another button press `Start again` is displayed and **mode is switched to DetectingBattery**.
![StartAgain](pictures/StartAgain.png)


## Cutoff display
If the state of the `PIN_DISCHARGE_TO_LOW` pin changes, the message `cutoff high` or `low`
and the according cutoff voltage is displayed in the first row for 2 seconds.

![CutoffHigh](pictures/CutoffHigh.png)
![CutoffLow](pictures/CutoffLow.png)

# Revision History
### Version 4.0.0
- Use capacity between NominalFullVoltageMillivolt and SwitchOffVoltageMillivoltHigh as standard capacity to enable better comparison.
- If powered by USB plotter pin logic is reversed, i.e. plotter output is enabled if NOT connected to ground.
- In state detecting battery, you can toggle cutoff voltage between high, low and zero (50 mV) with stop button.
- Fix bug for appending to compressed data.
- Support for storage period of 120 s.
- Synchronizing of LCD access for button handler, avoiding corrupted display content.
- Print improvements.
- Support for storage period of 120 s.
- Compression improved for rapidly descending voltage.
- Moving seldom used function of pin 10 to pin A5.
- New Logger mode with separate shunt enabled by pin 10.
- Store data in an array of structure instead in 3 arrays

### Version 3.2.1
- BUTTON_IS_ACTIVE_HIGH is not default any more

### Version 3.2.0
- Cutoff message improved.

### Version 3.1.0
- Fixed "conversion does not clear rest of EEPROM" bug.

### Version 3.0.0
- Improved compression.
- Attention by short beep each minute in STATE_DETECTING_BATTERY.

### Version 2.2.0
- ESR > 64 bug fixed.
- Display of changes on pin PIN_DISCHARGE_TO_LOW.

### Version 2.1.0
- ESR is stored and not computed any more.

### Version 2.0.0
- Improved version.

### Version 1.0.0
- Initial version with EEPROM storage.
