/*
 *  UltimateBatteryTester.ino
 *  Test internal resistance (ESR) of batteries and acquire and display the discharge graph
 *
 *  To suspend a measurement while in storage mode, press single for storage of current capacity
 *  If measurement is stopped, it can be started by another press
 *  If pin 11 connected to ground, verbose output for Arduino Serial Monitor is enabled. This is not suitable for Arduino Plotter.
 *
 *  Stored and displayed ESR is the average of the ESR's of the last storage period (1 min)
 *
 *  Data is stored to EEPROM in a compressed format.
 *  This allows to store 660 samples of voltage and milliampere and results in 11 h at 1 minute per sample (5 h 30 min at 30 seconds per sample).
 *  For a LIPO this is equivalent to around 3300 mAh.
 *  One EEPROM block contains the initial voltage and current values as well as the capacity, battery type and value of the used load resistor.
 *  These values are stored at the beginning of the measurement
 *  The plotter values are stored as 4 bit deltas and written to EEPROM every second sample.
 *  The upper 4 bit store the first value, lower 4 bit store the second value.
 *  The capacity is stored at end of measurement or on button press during the storage.
 *
 *
 *  Copyright (C) 2021-2022  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  https://github.com/ArminJo/Ultimate-Battery-Tester
 *
 *  UltimateBatteryTester is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

#include "ADCUtils.hpp"
#include "pitches.h"

/*
 * Version 2.2 - 8/2022
 *    ESR > 64 bug fixed.
 *    Display of changes on pin PIN_DISCHARGE_TO_LOW
 * Version 2.1 - 3/2022
 *    ESR is stored.
 * Version 2.0 - 3/2022
 *    Improved version.
 * Version 1.0 - 9/2021
 *    Tested version.
 * Version 0.0 - 9/2021
 *    Initial version.
 */

#define VERSION_EXAMPLE "2.2"
//#define DEBUG

#define LI_ION_CUT_OFF_VOLTAGE_MILLIVOLT        3500 // Switch off voltage for Li-ion capacity measurement
#define LI_ION_CUT_OFF_VOLTAGE_MILLIVOLT_LOW    3000 // Switch off voltage for Li-ion capacity measurement

#define NIMH_CUT_OFF_VOLTAGE_MILLIVOLT          1100 // Switch off voltage for NI-MH capacity measurement
#define NIMH_CUT_OFF_VOLTAGE_MILLIVOLT_LOW      1000 // Switch off voltage for NI-MH capacity measurement

/*
 * You should calibrate your ADC readout by replacing this value with the voltage you measured a the AREF pin after the program started.
 * For my Nanos I measured e.g. 1060 mV and 1093 mV.
 */
#if !defined(ADC_INTERNAL_REFERENCE_MILLIVOLT)
#define ADC_INTERNAL_REFERENCE_MILLIVOLT    1100L    // Value measured at the AREF pin
#endif

#define MILLIS_IN_ONE_SECOND 1000L
#define SECONDS_IN_ONE_MINUTE 60L

/*
 * Activate the type of LCD you use
 * Default is parallel LCD with 2 rows of 16 characters (1602).
 * Serial LCD uses A4/A5 - the hardware I2C pins on Arduino
 */
#if !defined(USE_SERIAL_LCD) && !defined(USE_PARALLEL_LCD) && !defined(USE_NO_LCD)
#define USE_PARALLEL_LCD
#endif
//#define USE_SERIAL_LCD

#define STATE_INITIAL_ESR_DURATION_SECONDS  32  // 32 seconds (-2 for initial display of append message) before starting discharge and storing, to have time to just test for ESR of battery.

#define MAX_VALUES_DISPLAYED_IN_PLOTTER     500 // The Arduino Plotter displays 500 values on before scrolling
#define NUMBER_OF_2_COMPRESSED_SAMPLES      329  // -1 since we always have the initial value. 11 h for 1 minute sample rate
#define STORAGE_PERIOD_SECONDS              SECONDS_IN_ONE_MINUTE // 1 minute
#define SAMPLE_PERIOD_MILLIS                MILLIS_IN_ONE_SECOND
#define BATTERY_DETECTION_PERIOD_MILLIS     (MILLIS_IN_ONE_SECOND / 2) // 500 ms
#define BATTERY_DETECTION_MINIMAL_MILLIVOLT 100
/*
 * Pin and ADC definitions
 * Start/Stop button is connected to INT0 pin 2
 * Pin 3 to 8 are used for parallel LCD connection
 */
#define ADC_CHANNEL_VOLTAGE          0 // ADC0 for voltage measurement
#define ADC_CHANNEL_CURRENT          1 // ADC1 for current measurement
#define PIN_VOLTAGE_RANGE_EXTENSION A2 // This pin is low to extend the voltage range from 2.2 volt to 4.4 volt
#define PIN_LOAD_HIGH               A3 // This pin is high to switch on the high load (3 ohm)
// A4 + A5, the hardware I2C pins on Arduino, are used for Serial LCD
#define PIN_LOAD_LOW                12 // This pin is high to switch on the low load (10 ohm). A4 is occupied by I2C for serial LCD display.
#define PIN_TONE                     9
// Mode pins
#define PIN_ONLY_PLOTTER_OUTPUT     10 // Verbose output to Arduino Serial Monitor is disabled, if connected to ground. This is intended for Arduino Plotter mode.
#define PIN_DISCHARGE_TO_LOW        11 // If connected to ground, "cut off is low" is displayed and discharge ends at a lower voltage. E.g. Li-ion discharge ends at 3000 mV instead of 3500 mV

#define USE_BUTTON_0            // Enable code for button 0 at INT0 / pin 2.
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!! I have an active high button attached !!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define BUTTON_IS_ACTIVE_HIGH

/*
 * External circuit definitions
 */
#define SHUNT_RESISTOR_MILLIOHM                 2000L  // 2 ohm
#define LOAD_LOW_MILLIOHM                       (1000 + SHUNT_RESISTOR_MILLIOHM) // Additional 1 ohm
#define LOAD_HIGH_MILLIOHM                      (10 * 1000 + SHUNT_RESISTOR_MILLIOHM) // Additional 10 ohm
#define ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE    2L       // Divider with 100 kOhm and 100 kOhm -> 2.2 V range
#define ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE   4L       // Divider with 100 kOhm and 33.333 kOhm -> 4.4 V range

#define NO_LOAD     0
#define LOW_LOAD    1 // 12 ohm
#define HIGH_LOAD   2 // 3 ohm

/*
 * Values for different battery types
 */
struct BatteryTypeInfoStruct {
    const char TypeName[11];
    uint16_t NominalVoltageMillivolt; // Not used yet
    uint16_t DetectionThresholdVoltageMillivolt; // type is detected if voltage is below this threshold
    uint16_t SwitchOffVoltageMillivolt;
    uint16_t SwitchOffVoltageMillivoltLow;
    uint8_t LoadType; // High or low
};
#define NO_BATTERY_INDEX    0
struct BatteryTypeInfoStruct BatteryTypeInfoArray[] = { { "No battery", 0, 1000, 0, 0, NO_LOAD }, /**/
{ "NiCd NiMH ", 1200, 1460, NIMH_CUT_OFF_VOLTAGE_MILLIVOLT, NIMH_CUT_OFF_VOLTAGE_MILLIVOLT_LOW, HIGH_LOAD }, /**/
{ "Alkali    ", 1500, 1550, 1300, 1000, HIGH_LOAD }, /**/
{ "NiZn batt.", 1650, 1800, 1500, 1300, HIGH_LOAD }, /**/
{ "LiFePO4   ", 3200, 3400, 3000, 2700, LOW_LOAD }, /**/
{ "LiIo batt.", 3700, 4300, LI_ION_CUT_OFF_VOLTAGE_MILLIVOLT, LI_ION_CUT_OFF_VOLTAGE_MILLIVOLT_LOW, LOW_LOAD }, /**/
{ "9 V Block ", 9000, 9200, 7 * NIMH_CUT_OFF_VOLTAGE_MILLIVOLT, 7 * NIMH_CUT_OFF_VOLTAGE_MILLIVOLT_LOW, LOW_LOAD }, /**/
{ "Voltage   ", 0, 0, 0, 0, NO_LOAD } };

/*
 * Current battery values set by getBatteryValues()
 */
struct BatteryInfoStruct {
    uint16_t VoltageNoLoadMillivolt;
    uint16_t VoltageLoadMillivolt;
    int16_t Milliampere;
    uint16_t Milliohm; // ESR - Equivalent Series Resistor | internal battery resistance
    uint16_t sESRDeltaMillivolt = 0; // only displayed at initial ESR testing
    uint32_t CapacityAccumulator;
    uint16_t CapacityMilliampereHour;

    uint8_t LoadState; // NO_LOAD | LOW_LOAD 12 ohm | HIGH_LOAD 3 ohm
    uint8_t TypeIndex;
} sBatteryInfo;
uint16_t sLastVoltageNoLoadMillivoltForBatteryCheck;
uint16_t sLastVoltageNoLoadMillivoltForPrint;
bool sBatteryWasInserted = false;
bool sBatteryWasDetectedAtLeastOnce = false;

/*
 * Tester state machine
 */
#define STATE_SETUP_AND_READ_EEPROM         0
#define STATE_DETECTING_BATTERY             1
#define STATE_INITIAL_ESR_MEASUREMENT       2 // only voltage and ESR measurement every n seconds for STATE_INITIAL_ESR_DURATION_SECONDS seconds
#define STATE_STORE_TO_EEPROM               3
#define STATE_STOPPED                       4 // Switch off voltage reached, until removal of battery
volatile uint8_t sMeasurementState = STATE_SETUP_AND_READ_EEPROM;

/*
 * Current value is in sCurrentLoadResistorHistory[0]. Used for computing and storing the average.
 */
#define HISTORY_SIZE_FOR_LOAD_RESISTOR_AVERAGE 16
uint16_t sCurrentLoadResistorHistory[HISTORY_SIZE_FOR_LOAD_RESISTOR_AVERAGE];
uint16_t sCurrentLoadResistorAverage;

/*
 * Current value is in sESRHistory[0]. The average is stored in sBatteryInfo.Milliohm.
 * For 9V and 60 mA we have a resolution of 0.3 ohm so we need an average.
 */
#define HISTORY_SIZE_FOR_ESR_AVERAGE ((STORAGE_PERIOD_SECONDS * SAMPLE_PERIOD_MILLIS) / MILLIS_IN_ONE_SECOND) // 60
uint16_t sESRHistory[HISTORY_SIZE_FOR_ESR_AVERAGE];

/*
 * EEPROM store
 */
EEMEM int8_t sMillivoltDeltaArrayEEPROM[NUMBER_OF_2_COMPRESSED_SAMPLES];
EEMEM int8_t sMilliampereDeltaArrayEEPROM[NUMBER_OF_2_COMPRESSED_SAMPLES];
EEMEM int8_t sMilliohmDeltaArrayEEPROM[NUMBER_OF_2_COMPRESSED_SAMPLES];
// The start values for the delta array
struct EEPROMStartValuesStruct {
    uint16_t initialDischargingMillivolt;
    uint16_t initialDischargingMilliampere;
    uint16_t initialDischargingMilliohm;

    uint16_t LoadResistorMilliohm;
    uint8_t BatteryTypeIndex;

    uint16_t CapacityMilliampereHour; // is set at end of measurement or by store button
};
EEMEM struct EEPROMStartValuesStruct EEPROMStartValues;
struct EEPROMStartValuesStruct StartValues;

/*
 * EEPROM array for discharging graph
 * Every array byte contains two 4 bit values
 * The upper 4 bit store the first value, the lower 4 bit store the second value
 * 8 is added to the 4 bit integer (range from -8 and 7) to get positive values for storage
 */
struct ValuesForDeltaStorageStruct {
    uint16_t lastStoredMilliampere;
    uint16_t lastStoredVoltageNoLoadMillivolt;
    uint16_t lastStoredMilliohm;
    uint8_t tempMilliampereDelta;
    uint8_t tempVoltageDelta;
    uint8_t tempMilliohmDelta;
    bool tempDeltaIsEmpty;
    int DeltaArrayIndex; // The index of the next values to be written. -1 to signal, that start values must be written.
} ValuesForDeltaStorage;

void getBatteryVoltageMillivolt();
bool detectAndPrintBatteryType();
void getBatteryCurrent();
void getBatteryValues();
bool checkStopCondition();
bool checkForVCCUndervoltage();
bool isBatteryRemoved();
void playEndTone(void);
void setLoad(uint8_t aNewLoadState);
void printBatteryValues();
void printValuesForPlotter(uint16_t aVoltageToPrint, uint16_t aMilliampereToPrint, uint16_t aMilliohmToPrint, bool aDoPrintCaption);
void printAsFloat(uint16_t aValueInMillis);
void LCDPrintAsFloat(uint16_t aValueInMillis);
void storeBatteryValues();
void storeCapacityToEEPROM();
void readAndPrintEEPROMData();
void printButtonUsageMessage();
void printDischargeToLowState();

void switchToStateDetectingBattery();
void switchToStateInitialESRMeasurement();
void switchToStateStoreToEEPROM();
void switchToStateStopped(bool aWriteToLCD = true);

void TogglePin(uint8_t aPinNr);
void LCDClearLine(uint8_t aLineNumber);

/*
 * Imports and definitions for start/stop button at pin 2
 */
#include "EasyButtonAtInt01.hpp"
void handleStartStopButtonPress(bool aButtonToggleState);   // The button press callback function
EasyButton Button0AtPin2(&handleStartStopButtonPress);      // Button is connected to INT0 (pin2)

/*
 * Imports and definitions for LCD
 */
#define LCD_MESSAGE_PERSIST_TIME_MILLIS     2000 // 2 second to view a message on LCD
#if defined(USE_SERIAL_LCD)
#include <LiquidCrystal_I2C.h> // Use an up to date library version which has the init method
#endif
#if defined(USE_PARALLEL_LCD)
#include "LiquidCrystal.h"
#endif

// definitions for a 1602 LCD
#define LCD_COLUMNS 16
#define LCD_ROWS 2

#if defined(USE_SERIAL_LCD) && defined(USE_PARALLEL_LCD)
#error Cannot use parallel and serial LCD simultaneously
#endif
#if defined(USE_SERIAL_LCD) || defined(USE_PARALLEL_LCD)
#define USE_LCD
#endif

#if defined(USE_SERIAL_LCD)
LiquidCrystal_I2C myLCD(0x27, LCD_COLUMNS, LCD_ROWS);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#endif
#if defined(USE_PARALLEL_LCD)
//LiquidCrystal myLCD(2, 3, 4, 5, 6, 7);
//LiquidCrystal myLCD(7, 8, A0, A1, A2, A3);
LiquidCrystal myLCD(7, 8, 3, 4, 5, 6);
#endif

/*
 * Loop timing
 */
#define LOAD_SWITCH_SETTLE_TIME_MILLIS  5   // Time for voltage to settle after load switch

//#define TEST
#if defined(TEST)
// to speed up testing the code
#undef STATE_INITIAL_ESR_DURATION_SECONDS
#undef STORAGE_PERIOD_SECONDS
#undef SAMPLE_PERIOD_MILLIS
#define STATE_INITIAL_ESR_DURATION_SECONDS   4
#define STORAGE_PERIOD_SECONDS               5
#define SAMPLE_PERIOD_MILLIS               500
#endif

#define VCC_CHECK_THRESHOLD_MILLIVOLT     3500 // 3.5 volt
#define VCC_CHECK_PERIOD_SECONDS            (60 * 5L) // check every 5 minutes if VCC is below VCC_CHECK_THRESHOLD_MILLIVOLT

unsigned long sLastMillisOfStorage;
unsigned long sLastMillisOfSample = 0;
unsigned long sLastMillisOfBatteryDetection = 0;
unsigned long sLastMillisOfVCCCheck = VCC_CHECK_PERIOD_SECONDS * MILLIS_IN_ONE_SECOND; // to force first check at startup
unsigned long sFirstMillisOfESRCheck = 0;
uint16_t sVCC;

bool sOnlyPlotterOutput; // contains the (inverted) value of the pin PIN_ONLY_PLOTTER_OUTPUT
bool sDischargeToLow; // contains the (inverted) value of the pin PIN_DISCHARGE_TO_LOW

// Helper macro for getting a macro definition as string
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

/*
 * Program starts here
 */
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_LOAD_HIGH, OUTPUT);
    pinMode(PIN_LOAD_LOW, OUTPUT);
    pinMode(PIN_ONLY_PLOTTER_OUTPUT, INPUT_PULLUP);
    pinMode(PIN_DISCHARGE_TO_LOW, INPUT_PULLUP);
    setLoad(NO_LOAD);
    digitalWrite(PIN_VOLTAGE_RANGE_EXTENSION, LOW); // prepare for later use

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif

    sOnlyPlotterOutput = !digitalRead(PIN_ONLY_PLOTTER_OUTPUT);
    if (!sOnlyPlotterOutput) {
        // Just to know which program is running on my Arduino
        Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));
        Serial.println(
                F(
                        "Connect pin " STR(PIN_ONLY_PLOTTER_OUTPUT) " to ground, to suppress such prints not suited for Arduino plotter"));
    }

    // Disable  digital input on all unused ADC channel pins to reduce power consumption
    DIDR0 = ADC0D | ADC1D;

#if defined(USE_LCD)
    /*
     * LCD initialization
     */
#  if defined(USE_SERIAL_LCD)
    myLCD.init();
    myLCD.clear();
    myLCD.backlight();
#  endif
#  if defined(USE_PARALLEL_LCD)
    myLCD.begin(LCD_COLUMNS, LCD_ROWS);
#  endif

    /*
     * LCD print program, version and date
     */
    myLCD.setCursor(0, 0);
    myLCD.print(F("Battery Tester "));
    myLCD.setCursor(0, 1);
    myLCD.print(F(VERSION_EXAMPLE "  " __DATE__));
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

    if (sOnlyPlotterOutput) {
        myLCD.setCursor(0, 1);
        myLCD.print(F("Only plotter out"));
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
    }
#endif

    tone(PIN_TONE, 2200, 100); // usage of tone() costs 1524 bytes code space

    /*
     * Get EEPROM data
     */
    readAndPrintEEPROMData();

    /*
     * Read inverted (of inverted) value to variable in order to force printing triggered by value change :-)
     */
    sDischargeToLow = digitalRead(PIN_DISCHARGE_TO_LOW);
    printDischargeToLowState();

    switchToStateDetectingBattery();
}

/*
 * The main loop with a delay of 100 ms
 */
void loop() {
    sOnlyPlotterOutput = !digitalRead(PIN_ONLY_PLOTTER_OUTPUT);

    if (millis() - sLastMillisOfVCCCheck >= VCC_CHECK_PERIOD_SECONDS * MILLIS_IN_ONE_SECOND) {
        sLastMillisOfVCCCheck = millis();
        if (checkForVCCUndervoltage()) {
            if (sMeasurementState == STATE_STORE_TO_EEPROM) {
                storeCapacityToEEPROM();
            }
            switchToStateStopped(false); // keep LCD message
        }
    }

    if (sMeasurementState == STATE_DETECTING_BATTERY) {
        if (millis() - sLastMillisOfBatteryDetection >= BATTERY_DETECTION_PERIOD_MILLIS) {
            sLastMillisOfBatteryDetection = millis();
            /*
             * Check if battery was inserted
             */
            if (detectAndPrintBatteryType()) {
                // we waited 2 seconds in detectAndPrintBatteryType(), so must check if mode has not changed
                if (sMeasurementState == STATE_DETECTING_BATTERY) {
                    switchToStateInitialESRMeasurement();
                    // If found, print button usage once
                    if (!sBatteryWasDetectedAtLeastOnce) {
                        sBatteryWasDetectedAtLeastOnce = true;
                        printButtonUsageMessage();
                    }
                    // set load for the first call of getBatteryValues() to measure the current
                    setLoad(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].LoadType);
                }
            } else {
                // Not inserted, so print VCC voltage initially
#if defined(USE_LCD)
                if (sMeasurementState == STATE_DETECTING_BATTERY) {
                    myLCD.setCursor(0, 0);
                    sVCC = getVCCVoltageMillivolt();
                    myLCD.print(sVCC / 1000.0, 2);
                    myLCD.print(F("V"));
                }
#endif
            }
        }

    } else if (millis() - sLastMillisOfSample >= SAMPLE_PERIOD_MILLIS) {
        sLastMillisOfSample = millis();
        /*
         * Do all this every second
         */

        if (sMeasurementState == STATE_STOPPED) {
            getBatteryVoltageMillivolt(); // get battery no load voltage
        } else {
            getBatteryValues(); // must be called only once per sample!
        }

        if (isBatteryRemoved()) {
            switchToStateDetectingBattery(); // switch back to start
        }

        /*
         * Always print battery values
         * must be after battery removed detection!
         */
        printBatteryValues();

        /*
         * Check for end of STATE_INITIAL_ESR_MEASUREMENT
         */
        if (sMeasurementState == STATE_INITIAL_ESR_MEASUREMENT
                && millis() - sFirstMillisOfESRCheck >= (STATE_INITIAL_ESR_DURATION_SECONDS * MILLIS_IN_ONE_SECOND)) {
            /*
             * Print message
             */
            printButtonUsageMessage();

            // If button was not pressed before, start a new data set
            if (sMeasurementState == STATE_INITIAL_ESR_MEASUREMENT) {
                // Force new data set
                ValuesForDeltaStorage.DeltaArrayIndex = -1;
                sBatteryInfo.CapacityAccumulator = 0;
                memset(sCurrentLoadResistorHistory, 0, sizeof(sCurrentLoadResistorHistory));

                sLastMillisOfSample = 0; // next sample in 1 second
                switchToStateStoreToEEPROM();
            }
            // end of state STATE_INITIAL_ESR_MEASUREMENT

        } else if (sMeasurementState == STATE_STORE_TO_EEPROM) {

            /*
             * Check for periodic storage to EEPROM
             */
            if (millis() - sLastMillisOfStorage >= STORAGE_PERIOD_SECONDS * MILLIS_IN_ONE_SECOND) {
                sLastMillisOfStorage = millis();
                storeBatteryValues();
                /*
                 * Check for switch off voltage reached, if stored to EEPROM
                 */
                if (ValuesForDeltaStorage.tempDeltaIsEmpty && checkStopCondition()) {
                    switchToStateStopped(false); // keep LCD message "finished"
                }
            }
        }

        /*
         * Blink feedback if measurement is running
         */
        if (sMeasurementState != STATE_STOPPED) {
            TogglePin(LED_BUILTIN);
        }
    }
    delay(100);
    printDischargeToLowState();
}

void switchToStateDetectingBattery() {
    sMeasurementState = STATE_DETECTING_BATTERY;
    if (!sOnlyPlotterOutput) {
        Serial.println(F("Switch to state DETECTING BATTERY"));
    }
    sLastVoltageNoLoadMillivoltForBatteryCheck = 0XFFFF; // to force first check if voltage is 0
}

void switchToStateInitialESRMeasurement() {
    sMeasurementState = STATE_INITIAL_ESR_MEASUREMENT;
    if (!sOnlyPlotterOutput) {
        Serial.println(F("Switch to state INITIAL ESR MEASUREMENT"));
    }
    sFirstMillisOfESRCheck = millis();
    memset(sESRHistory, 0, sizeof(sESRHistory));
}

void switchToStateStoreToEEPROM() {
    sMeasurementState = STATE_STORE_TO_EEPROM;
    if (!sOnlyPlotterOutput) {
        Serial.println(F("Switch to state STORE TO EEPROM"));
    }
    sLastMillisOfStorage = -(STORAGE_PERIOD_SECONDS * MILLIS_IN_ONE_SECOND); // store first value immediately
}

/*
 * aWriteToLCD if false do not overwrite "finished" message
 */
void switchToStateStopped(bool aWriteToLCD) {
    if (sMeasurementState != STATE_STOPPED) {
        sMeasurementState = STATE_STOPPED;
        setLoad(NO_LOAD);
        digitalWrite(LED_BUILTIN, LOW);

        if (!sOnlyPlotterOutput) {
            Serial.println(F("Switch to state STOPPED"));
        }
        if (aWriteToLCD) {
#if defined(USE_LCD)
            myLCD.setCursor(0, 0);
            myLCD.print(F("Stop measurement"));
            delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
            LCDClearLine(0);
            myLCD.setCursor(9, 0);
            myLCD.print(F("Stopped"));
#endif
        }
    }
}

/*
 * Prints state of pin if state changed or if aForcePrint is true
 */
void printDischargeToLowState() {
    bool tDischargeToLow = sDischargeToLow;
    sDischargeToLow = !digitalRead(PIN_DISCHARGE_TO_LOW);
    if (tDischargeToLow != sDischargeToLow) {
        if (!sOnlyPlotterOutput) {
            if (sDischargeToLow) {
                Serial.println(F("Discharge to lower voltage. e.g. 3000 mV for Li-ion"));
            } else {
                Serial.println(F("Discharge to non critical voltage. e.g. 3500 mV for Li-ion"));
            }
        }
#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        myLCD.print(F("Cut off is "));
        if (sDischargeToLow) {
            myLCD.print(F("low  "));
        } else {
            myLCD.print(F("high "));
        }
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
    }
}

/*
 * Print message "dbl press = stop" and "Press button to append to EEPROM"
 * Wait if no button was pressed
 */
void printButtonUsageMessage() {
    uint8_t tOldMeasurementState = sMeasurementState;

    if (!sOnlyPlotterOutput) {
        Serial.println(F("Double press \"Start/stop\" button to stop measurement"));
    }
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    myLCD.print(F("dbl press = stop"));
    /*
     * and wait for 2 seconds for button press
     */
    for (uint_fast8_t i = 0; i < 10; ++i) {
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS / 10);
        if (sMeasurementState != tOldMeasurementState) {
            // button press changes state
            break;
        }
    }
    if (sMeasurementState == tOldMeasurementState) {
#endif
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Press \"Start/stop\" button to append values to already stored EEPROM data"));
        }
#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        myLCD.print(F("Press button to "));
        myLCD.setCursor(0, 1);
        myLCD.print(F("append to EEPROM"));

        /*
         * and wait for 2 seconds for button press
         */
        for (uint_fast8_t i = 0; i < 10; ++i) {
            delay(LCD_MESSAGE_PERSIST_TIME_MILLIS / 10);
            if (sMeasurementState != tOldMeasurementState) {
                // button press changes state
                break;
            }
        }
    }
    LCDClearLine(0);
    LCDClearLine(1);
#endif
}

/*
 * Makes only sense for battery operated mode which in turn requires a LCD.
 */
bool checkForVCCUndervoltage() {
    sVCC = getVCCVoltageMillivolt();
    if (sVCC < VCC_CHECK_THRESHOLD_MILLIVOLT) {
#if defined(USE_LCD)
        myLCD.setCursor(0, 1);
        myLCD.print(F("VCC undervoltage"));
        myLCD.setCursor(7, 0);
        myLCD.print(F("VCC="));
        myLCD.print(getVCCVoltage(), 2);
        myLCD.print('V');
#endif
        playEndTone(); // 3 seconds
        return true;
    }
    return false;
}

/*
 * Check for removed battery
 * @return true if battery removed
 */
bool isBatteryRemoved() {

    // check only if battery was inserted before
    if (sBatteryWasInserted && sBatteryInfo.VoltageNoLoadMillivolt < BATTERY_DETECTION_MINIMAL_MILLIVOLT) {
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Battery removing detected"));
        }
        sBatteryWasInserted = false;
        return true;
    }
    return false;
}

/*
 * Ignore all presses in mode STATE_INIT / before mode STATE_SETUP
 * In mode STATE_EEPROM_DATA_PRINTED, STATE_INITIAL_ESR_MEASUREMENT and mode STATE_STOPPED:
 *      Starts measurement and appends data to already stored ones. -> goes to state STATE_STORE_TO_EEPROM
 * In mode STATE_STORE_TO_EEPROM:
 *      Stores current capacity and stops measurement. -> goes to state STATE_STOPPED
 * Always:
 *      Double click in 2 seconds stop measurement. -> goes to state STATE_STOPPED
 *      We can be called recursively, i.e. while waiting for 2 seconds we can be called for double press
 */
void handleStartStopButtonPress(bool aButtonToggleState) {
    if (sMeasurementState == STATE_SETUP_AND_READ_EEPROM || sMeasurementState == STATE_DETECTING_BATTERY) {
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Early press ignored"));
        }
        return;
    }

    if (Button0AtPin2.checkForDoublePress(LCD_MESSAGE_PERSIST_TIME_MILLIS)) {
        /*
         * Double press detected!
         * Go to STATE_STOPPED
         */
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Double press detected"));
        }
        switchToStateStopped();

    } else {
        /*
         * Single press here
         * Attention, this press can be the first press of a double press,
         * so we must wait 2 seconds and check for double press before processing single press
         */
        if (!sOnlyPlotterOutput) {
            // Print should be done after checkForDoublePress() in order to not disturb the double press detection
            Serial.print(F("Button pressed, state="));
            Serial.println(sMeasurementState);
        }

        if (sMeasurementState == STATE_STORE_TO_EEPROM) {
            /*
             * Store capacity and stop
             */
            storeCapacityToEEPROM();
            sLastMillisOfSample = millis() + LCD_MESSAGE_PERSIST_TIME_MILLIS;

#if defined(USE_LCD)
            myLCD.setCursor(0, 0);
            myLCD.print(F("Capacity stored "));
            delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#else
                delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
            switchToStateStopped(); // no check for double press required here :-)

        } else {
            /*
             * Print start message, and wait for 2 seconds for double press detection
             */
            uint8_t tOldMeasurementState = sMeasurementState;
#if defined(USE_LCD)
            myLCD.setCursor(0, 0);
#endif
            if (sMeasurementState == STATE_STOPPED) {
                if (!sOnlyPlotterOutput) {
                    Serial.println(F("Start again"));
                }
#if defined(USE_LCD)
                myLCD.print(F("Start again     "));
#endif
            } else {
                if (!sOnlyPlotterOutput) {
                    Serial.println(F("Append data to EEPROM"));
                }
#if defined(USE_LCD)
                myLCD.print(F("Append to EEPROM"));
#endif
            }
            delay(LCD_MESSAGE_PERSIST_TIME_MILLIS); // wait for double press detection
#if defined(USE_LCD)
            LCDClearLine(0);
#endif

            // Must check old value avoid switching to DetectingBattery after double press
            if (tOldMeasurementState == STATE_STOPPED) {
                // start a new measurement cycle
                switchToStateDetectingBattery();

            } else if (sMeasurementState == STATE_INITIAL_ESR_MEASUREMENT) {
                // continue storage, no stop requested during 2 seconds wait
                ValuesForDeltaStorage.tempDeltaIsEmpty = true;
                sLastMillisOfSample = millis(); // next sample in 30 seconds
                switchToStateStoreToEEPROM();

            } else if (sMeasurementState != STATE_STOPPED) {
                // append data if no double press happened
                switchToStateStoreToEEPROM();
            }
        }
//        if (!sOnlyPlotterOutput) {
//            Serial.print(F("New state="));
//            Serial.println(sMeasurementState);
//        }
    }
}

void setLoad(uint8_t aNewLoadState) {
    if (sBatteryInfo.LoadState != aNewLoadState) {
        sBatteryInfo.LoadState = aNewLoadState;

#if defined(DEBUG)
            Serial.print(F("Set load to "));
#endif

        if (aNewLoadState == NO_LOAD) {
#if defined(DEBUG)
                Serial.println(F("off"));
#endif
            digitalWrite(PIN_LOAD_LOW, LOW);    // disable 12 ohm load
            digitalWrite(PIN_LOAD_HIGH, LOW);    // disable 3 ohm load
        } else if (aNewLoadState == LOW_LOAD) {
#if defined(DEBUG)
                Serial.println(F("low"));
#endif
            digitalWrite(PIN_LOAD_LOW, HIGH);    // enable 12 ohm load
            digitalWrite(PIN_LOAD_HIGH, LOW);    // disable 3 ohm load
        } else {
#if defined(DEBUG)
                Serial.println(F("high"));
#endif
            digitalWrite(PIN_LOAD_LOW, LOW);    // disable 12 ohm load
            digitalWrite(PIN_LOAD_HIGH, HIGH);    // enable 3 ohm load
        }

    }
}

/*
 * Sets VoltageNoLoadMillivolt or VoltageLoadMillivolt
 * Provides automatic range switch between 2.2, 4.4 and 14 (to 20) volt range
 * The ranges are realized by a divider with 100 kOhm and 100 kOhm -> 2.2 V range and a divider with 100 kOhm and 33.333 kOhm -> 4.4 v range
 * The 14 volt range is realized by using the 4.4 volt range with VCC (of at least 3.5 volt) as reference.
 * With 5 volt VCC this range goes up to 20 volt.
 * Does not affect the loads
 */
void getBatteryVoltageMillivolt() {
    static bool sVoltageRangeIsLow = true;

    uint16_t tInputVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
    /*
     * Automatic range
     */
    if (sVoltageRangeIsLow && tInputVoltageRaw >= 0x3F0) {
        // switch to higher voltage range by activating the range extension resistor at pin A2
        sVoltageRangeIsLow = false;
        pinMode(PIN_VOLTAGE_RANGE_EXTENSION, OUTPUT);
        digitalWrite(PIN_VOLTAGE_RANGE_EXTENSION, LOW);    // required???
#if defined(DEBUG)
            if (!sOnlyPlotterOutput) {
                Serial.println(F("Switch to 4.4 V range"));
            }
#endif
        tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
    }
    if (!sVoltageRangeIsLow) {
        if (tInputVoltageRaw < (((0x3F0L * ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE) / ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE) - 0x10)) {
            // switch to lower voltage range by deactivating the range extension resistor at pin A2
            sVoltageRangeIsLow = true;
            pinMode(PIN_VOLTAGE_RANGE_EXTENSION, INPUT);
            digitalWrite(PIN_VOLTAGE_RANGE_EXTENSION, LOW);
#if defined(DEBUG)
                if (!sOnlyPlotterOutput) {
                    Serial.println(F("Switch to 2.2 V range"));
                }
#endif
            tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
        } else if (tInputVoltageRaw >= 0x3F0) {
            /*
             * Here we have 17 mV resolution
             * which leads to e.g. 0.3 ohm resolution at 9V and 60 mA
             */
#if defined(DEBUG)
                if (!sOnlyPlotterOutput) {
                    Serial.print(F("Switch to "));
                    Serial.print(sVCC / 1000.0, 3);
                    Serial.println(F(" V range"));
                }
#endif
            // switch to highest voltage range by using VCC as reference
            uint16_t tReadoutFor1_1Reference = waitAndReadADCChannelWithReference(ADC_1_1_VOLT_CHANNEL_MUX, DEFAULT); // 225 at 5 volt VCC
            tInputVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_VOLTAGE, DEFAULT);
#if defined(DEBUG)
                Serial.print(tInputVoltageRaw);
                Serial.print(F(" / "));
                Serial.println(tReadoutFor1_1Reference);
#endif
            // Adjust tInputVoltageRaw to a range above 1023 for computation of voltage below
            tInputVoltageRaw = (tInputVoltageRaw * 1023L) / tReadoutFor1_1Reference;
        }
    }
#if defined(DEBUG)
        Serial.print(F("tInputVoltageRaw="));
        Serial.print(tInputVoltageRaw);
#endif
    /*
     * Compute voltage
     */
    uint16_t tCurrentBatteryVoltageMillivolt;
    if (sVoltageRangeIsLow) {
        tCurrentBatteryVoltageMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE)
                * tInputVoltageRaw) / 1023);
    } else {
        tCurrentBatteryVoltageMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE)
                * tInputVoltageRaw) / 1023);
    }

    if (sBatteryInfo.LoadState == NO_LOAD) {
        sBatteryInfo.VoltageNoLoadMillivolt = tCurrentBatteryVoltageMillivolt;
    } else {
        sBatteryInfo.VoltageLoadMillivolt = tCurrentBatteryVoltageMillivolt;
    }

#if defined(DEBUG)
        Serial.print(F(" -> "));
        Serial.print(tCurrentBatteryVoltageMillivolt);
        Serial.println(F(" mV"));
#endif
}

/*
 * Maximal current for a 2 ohm shunt resistor is 550 mA, and resolution is 0.54 mA.
 */
void getBatteryCurrent() {
    uint16_t tShuntVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_CURRENT, INTERNAL);
    sBatteryInfo.Milliampere =
            (((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) * tShuntVoltageRaw) / (1023L * SHUNT_RESISTOR_MILLIOHM));
}

/*
 * Assumes that load is activated before called
 */
void getBatteryValues() {
    // Do it before deactivating the load
    getBatteryCurrent();
    getBatteryVoltageMillivolt();    // get current battery load voltage

    //Deactivate load and wait for voltage to settle
    setLoad(NO_LOAD);
    delay(LOAD_SWITCH_SETTLE_TIME_MILLIS);

    getBatteryVoltageMillivolt();    // get current battery no load voltage
    sBatteryInfo.sESRDeltaMillivolt = sBatteryInfo.VoltageNoLoadMillivolt - sBatteryInfo.VoltageLoadMillivolt;

    // restore original load state
    setLoad(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].LoadType);

    if (sBatteryInfo.Milliampere > 1) {
        // New capacity computation
        sBatteryInfo.CapacityAccumulator += sBatteryInfo.Milliampere;
        sBatteryInfo.CapacityMilliampereHour = sBatteryInfo.CapacityAccumulator
                / ((3600L * MILLIS_IN_ONE_SECOND) / SAMPLE_PERIOD_MILLIS);    // = / 3600 for 1 s sample period

        /*
         * Compute sESRAverage
         * Shift history array and insert current value
         */
        uint8_t tESRAverageHistoryCounter = 1; // we always add sESRHistory[0]
        uint32_t tESRAverageAccumulator = 0;
        for (uint_fast8_t i = HISTORY_SIZE_FOR_ESR_AVERAGE - 1; i > 0; --i) {
            if (sESRHistory[i - 1] != 0) {
                // shift i-1 to i and add to average
#if defined(DEBUG)
                    Serial.print(sESRHistory[i - 1]);
                    Serial.print('+');
#endif
                tESRAverageHistoryCounter++; // count only valid entries
                tESRAverageAccumulator += sESRHistory[i - 1];
                sESRHistory[i] = sESRHistory[i - 1];
            }
        }
        uint32_t tESRMilliohm = (sBatteryInfo.sESRDeltaMillivolt * 1000L) / sBatteryInfo.Milliampere;
        if (tESRMilliohm > 64000) {
            sESRHistory[0] = 59999; // indicate overflow
        } else {
            sESRHistory[0] = tESRMilliohm;
        }

        tESRAverageAccumulator += sESRHistory[0];
        sBatteryInfo.Milliohm = (tESRAverageAccumulator + (tESRAverageHistoryCounter / 2)) / tESRAverageHistoryCounter;

#if defined(DEBUG)
            Serial.print(sESRHistory[0]);
            Serial.print('/');
            Serial.print(tESRAverageHistoryCounter);
            Serial.print('=');
            Serial.print(sBatteryInfo.Milliohm);
            Serial.println();
#endif

        /*
         * Compute sCurrentLoadResistorAverage if array is full
         * Formula is: LoadVoltage / LoadCurrent and includes the MosFet and the connector resistance.
         * Shift load resistor history array and insert current value
         */
        uint32_t tLoadResistorAverage = 0;
        for (uint_fast8_t i = HISTORY_SIZE_FOR_LOAD_RESISTOR_AVERAGE - 1; i > 0; --i) {
            tLoadResistorAverage += sCurrentLoadResistorHistory[i - 1];
            sCurrentLoadResistorHistory[i] = sCurrentLoadResistorHistory[i - 1];
        }
        sCurrentLoadResistorHistory[0] = (sBatteryInfo.VoltageLoadMillivolt * 1000L / sBatteryInfo.Milliampere);
        tLoadResistorAverage += sCurrentLoadResistorHistory[0];
        if (sCurrentLoadResistorHistory[HISTORY_SIZE_FOR_LOAD_RESISTOR_AVERAGE - 1] != 0) {
            /*
             * as soon as array is filled up, compute rounded average load resistance value each time.
             * Required for restoring battery capacity from stored data.
             */
            sCurrentLoadResistorAverage = (tLoadResistorAverage + (HISTORY_SIZE_FOR_LOAD_RESISTOR_AVERAGE / 2))
                    / HISTORY_SIZE_FOR_LOAD_RESISTOR_AVERAGE;
        }
    }
}

/*
 * Play short melody
 * Duration 3 seconds
 */
void playEndTone(void) {
    tone(PIN_TONE, NOTE_A5);
    delay(1000);
    tone(PIN_TONE, NOTE_E5);
    delay(1000);
    tone(PIN_TONE, NOTE_A4, 1000);
}

/*
 * Check for switch off voltage reached -> end of measurement
 * @return true if stop condition met
 */
bool checkStopCondition() {
    uint16_t tSwitchOffVoltageMillivolt;
    if (sDischargeToLow) {
        tSwitchOffVoltageMillivolt = BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivoltLow;
    } else {
        tSwitchOffVoltageMillivolt = BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivolt;
    }
    if (sBatteryInfo.VoltageNoLoadMillivolt < tSwitchOffVoltageMillivolt) {
        /*
         * Switch off condition met
         */
        setLoad(NO_LOAD);
        storeCapacityToEEPROM();
        if (!sOnlyPlotterOutput) {
            Serial.print(F("Switch off voltage "));
            Serial.print(tSwitchOffVoltageMillivolt);
            Serial.print(F(" V reached, capacity="));
            Serial.print(sBatteryInfo.CapacityMilliampereHour);
            Serial.println(F(" mAh"));
        }
#if defined(USE_LCD)
        myLCD.setCursor(7, 0);
        myLCD.print(F(" Finished"));
#endif
        // Play short melody
        playEndTone();
        return true;
    }
    return false;
}

/*
 * search the "database" for a matching type
 */
uint8_t getBatteryTypeIndex(uint16_t aBatteryVoltageMillivolt) {

    // scan all threshold voltage of all battery types
    for (uint_fast8_t i = 0; i < sizeof(BatteryTypeInfoArray) / sizeof(BatteryTypeInfoStruct) - 1; i++) {
        if (aBatteryVoltageMillivolt < BatteryTypeInfoArray[i].DetectionThresholdVoltageMillivolt) {
#if defined(DEBUG)
//            Serial.print(F(" Battery index="));
//            Serial.print(i);
//            Serial.print(F(" BatteryVoltageMillivolt="));
//            Serial.print(aBatteryVoltageMillivolt);
//            Serial.print(F(" SwitchOffVoltageMillivolt="));
//            Serial.println(BatteryTypeInfoArray[i].SwitchOffVoltageMillivolt);
#endif
            return i;
        }
    }
    // High voltage is detected
    return sizeof(BatteryTypeInfoArray) / sizeof(BatteryTypeInfoStruct) - 1;
}

/*
 * Disables the load, measures the voltage to detecting battery type and enables the load if battery detected
 * @return true, if battery detected
 */
bool detectAndPrintBatteryType() {
    setLoad(NO_LOAD);
    getBatteryVoltageMillivolt();
    if (sLastVoltageNoLoadMillivoltForBatteryCheck == sBatteryInfo.VoltageNoLoadMillivolt) {
        sLastVoltageNoLoadMillivoltForBatteryCheck = sBatteryInfo.VoltageNoLoadMillivolt;
        return false;
    }

    sBatteryInfo.TypeIndex = getBatteryTypeIndex(sBatteryInfo.VoltageNoLoadMillivolt);
    // print values
    if (!sOnlyPlotterOutput) {
        Serial.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].TypeName);
        Serial.println(F(" found"));
    }

    if (sBatteryInfo.TypeIndex == NO_BATTERY_INDEX) {
#if defined(USE_LCD)
        myLCD.setCursor(5, 0);
        myLCD.print(F("  "));
        myLCD.print(F(" No batt."));
#endif
        return false;
    } else {
        printBatteryValues(); // only voltage is printed here and sLastVoltageNoLoadMillivoltForPrint is set :-)
        Serial.println();

#if defined(USE_LCD)
        myLCD.setCursor(0, 1);
        myLCD.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].TypeName);
        myLCD.print(F(" found"));
        // The current battery voltage is displayed, so clear "No batt." message selectively
        myLCD.setCursor(8, 0);
        myLCD.print(F("        "));
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
        LCDClearLine(1);
#endif
        sBatteryWasInserted = true;
        return true;
    }
}

/*
 * Evaluates sMeasurementState and prints:
 *   - sBatteryInfo.VoltageNoLoadMillivolt or sBatteryInfo.VoltageLoadMillivolt
 *   - sBatteryInfo.Milliampere
 *   - sBatteryInfo.Milliohm
 *   - optional sESRDeltaMillivolt or capacity
 * to Serial and LCD
 *
 * 1602 LCD layout
 *  0   4   8   C  F
 *  0.000V 12 330mA
 *  0.666ohm  33mAh
 *         No batt.
 *  NiCd NiMH found
 *
 */
void printBatteryValues() {
    char tString[6];

    uint8_t tMeasurementState = sMeasurementState; // Because sMeasurementState is volatile
    if (tMeasurementState != STATE_SETUP_AND_READ_EEPROM) {
        /*
         * Print VoltageNoLoadMillivolt always except for results of EEPROM data
         */
        uint16_t tVoltageNoLoadMillivolt = sBatteryInfo.VoltageNoLoadMillivolt; // saves 12 bytes programming space
        if (!sOnlyPlotterOutput) {
            if (tMeasurementState != STATE_STOPPED || abs(sLastVoltageNoLoadMillivoltForPrint - tVoltageNoLoadMillivolt) > 2) {
                sLastVoltageNoLoadMillivoltForPrint = tVoltageNoLoadMillivolt;
                printAsFloat(tVoltageNoLoadMillivolt);
                Serial.print(F(" V "));
                // Do not print newline for "0.000 V No battery found"
                if (tMeasurementState == STATE_STOPPED) {
                    Serial.println();
                }
            }
        }
#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        LCDPrintAsFloat(tVoltageNoLoadMillivolt);
        myLCD.print(F("V "));
#endif
        // cursor is now at 7, 0

        if (tMeasurementState == STATE_DETECTING_BATTERY || tMeasurementState == STATE_STOPPED) {
            //Print only voltage for this states
            return;
        }

        /*********************************************************************************
         * Here we only have state STATE_INITIAL_ESR_MEASUREMENT or STATE_STORE_TO_EEPROM
         *********************************************************************************/

        /*
         * Print down counter for STATE_INITIAL_ESR_MEASUREMENT
         */
        if (tMeasurementState == STATE_INITIAL_ESR_MEASUREMENT) {
            uint8_t tSecondsToGo = STATE_INITIAL_ESR_DURATION_SECONDS - ((millis() - sFirstMillisOfESRCheck) / 1000);
            if (!sOnlyPlotterOutput) {
                Serial.print(tSecondsToGo);
                Serial.print(F(" s ")); // seconds until discharging
            }
            if (tSecondsToGo < 10) {
#if defined(USE_LCD)
                myLCD.print(' '); // padding space for count
#endif
                tone(PIN_TONE, 2000, 40); // costs 1524 bytes code space
            }
#if defined(USE_LCD)
            myLCD.print(tSecondsToGo);
            myLCD.print(' '); // trailing space for count (just in case mA are > 999)
#endif
        } else {
            // STATE_STORE_TO_EEPROM here
#if defined(USE_LCD)
            int tDeltaArrayIndex = ValuesForDeltaStorage.DeltaArrayIndex; // saves 8 bytes of program space
            if (tDeltaArrayIndex >= 0) {
                if (tDeltaArrayIndex < 100) {
                    myLCD.print(' '); // padding space :-)
                }
                if (tDeltaArrayIndex < 10) {
                    myLCD.print(' '); // padding space :-)
                }
                myLCD.print(tDeltaArrayIndex);
            } else {
                myLCD.print(F("   ")); // we have it once, because we store values (and increment index) after print
            }
#endif
        }
        // cursor is now at 9, 0

        /*
         * Print milliampere
         */
        sprintf_P(tString, PSTR("%4u"), sBatteryInfo.Milliampere);
        if (!sOnlyPlotterOutput) {
            Serial.print(tString);
            Serial.print(F(" mA at "));
            printAsFloat(sCurrentLoadResistorHistory[0]);
            Serial.print(F(" ohm "));
        }
#if defined(USE_LCD)
        myLCD.print(tString);
        myLCD.print(F("mA"));
#endif
    }

    /*
     * End of first row, start of second one
     */

    /*
     * Print current ESR for STATE_INITIAL_ESR_MEASUREMENT
     * else print average ESR contained in sBatteryInfo.Milliohm
     */
    uint16_t tMilliohm;
    if (tMeasurementState == STATE_INITIAL_ESR_MEASUREMENT) {
        tMilliohm = sESRHistory[0];
    } else {
        tMilliohm = sBatteryInfo.Milliohm;
    }
    if (!sOnlyPlotterOutput) {
        Serial.print(F(" ESR "));
        printAsFloat(tMilliohm);
        Serial.print(F(" ohm "));
    }
#if defined(USE_LCD)
    myLCD.setCursor(0, 1);
    LCDPrintAsFloat(tMilliohm);
    myLCD.print(F("\xF4   ")); // Ohm symbol
#endif

    if (tMeasurementState == STATE_INITIAL_ESR_MEASUREMENT) {
        /*
         * Print voltage at load
         */
        uint16_t tESRDeltaMillivolt = sBatteryInfo.sESRDeltaMillivolt; // saves 4 bytes programming space
        if (!sOnlyPlotterOutput) {
            printAsFloat(tESRDeltaMillivolt);
            Serial.print(F(" V "));
        }
#if defined(USE_LCD)
        myLCD.print(' '); // leading space only for voltage
        LCDPrintAsFloat(tESRDeltaMillivolt);
        myLCD.print(F("V"));
#endif

    } else {
        /*
         * Print capacity
         */
        sprintf_P(tString, PSTR("%4u"), sBatteryInfo.CapacityMilliampereHour);
        if (!sOnlyPlotterOutput) {
            Serial.print(tString);
            Serial.print(F(" mAh"));
        }
#if defined(USE_LCD)
        myLCD.print(tString);
        myLCD.print(F("mAh"));
#endif
    }

    if (!sOnlyPlotterOutput) {
        Serial.println();
    }
}

void printAsFloat(uint16_t aValueInMillis) {
    Serial.print(((float) (aValueInMillis)) / 1000, 3);
}

void LCDPrintAsFloat(uint16_t aValueInMillis) {
#if defined(USE_LCD)
    myLCD.print(((float) (aValueInMillis)) / 1000, 3);
#endif
}

/*
 * Just clear the complete EEPROM
 */
void clearEEPROMTo_FF() {
    if (!sOnlyPlotterOutput) {
        Serial.println(F("Clear EEPROM"));
    }
    for (int i = 0; i < E2END; ++i) {
        eeprom_update_byte((uint8_t*) i, 0xFF);
    }
}

/*
 * upper 4 bit store the first value (between -8 and 7), lower 4 bit store the second value
 * @return clipped aDelta | aDelta which is stored
 */
int16_t store4BitDeltas(int16_t aDelta, uint8_t *aDeltaTemp, uint8_t *aEEPROMAddressToStoreValue) {
    if (!sOnlyPlotterOutput) {
        Serial.print(' ');
        Serial.print(aDelta);
    }
    // clip aDelta to the available range of -8 to 7
    if (aDelta > 7) {
        aDelta = 7;
    } else if (aDelta < -8) {
        aDelta = -8;
    }
    /*
     * convert delta to an unsigned value by adding 8 => -8 to 7 -> 0 to F
     * F is +7, 8 is 0, 0 is -8 tDelta is positive now :-)
     */
    uint8_t tDelta = aDelta + 8;

    uint8_t tDeltaToStore;
    if (ValuesForDeltaStorage.tempDeltaIsEmpty) {
        tDeltaToStore = tDelta << 4; // Store in upper 4 bit
        *aDeltaTemp = tDeltaToStore;
    } else {
        // upper 4 bit store the first value (between -8 and 7), lower 4 bit store the second value
        tDeltaToStore = *aDeltaTemp | tDelta;
        eeprom_write_byte(aEEPROMAddressToStoreValue, tDeltaToStore);

        if (tDeltaToStore != eeprom_read_byte(aEEPROMAddressToStoreValue)) {
            // Yes, I have seen this (starting with index 4 6 times 0xFF for current). Maybe undervoltage while powered by battery.
            tone(PIN_TONE, NOTE_C7, 20);
            delay(40);
            tone(PIN_TONE, NOTE_C6, 20);
            delay(40);
            tone(PIN_TONE, NOTE_C7, 20);
        }
    }
    if (!sOnlyPlotterOutput) {
        Serial.print(F("->0x"));
        Serial.print(tDeltaToStore, HEX);
    }
    return aDelta;
}

/*
 * Store values to EEPROM as 4 bit deltas between sBatteryInfo and ValuesForDeltaStorage and write them to EEPROM every second call
 * Upper 4 bit store the first value, lower 4 bit store the second value
 */
void storeBatteryValues() {
    if (ValuesForDeltaStorage.DeltaArrayIndex < 0) {
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Store initial values to EEPROM"));
        }
        /*
         * Initial values
         * Storing them in a local structure and storing this, costs 50 bytes code size
         * Storing them in a global structure and storing this, costs 30 bytes code size
         */
        clearEEPROMTo_FF();

        /*
         * Initially set up structure for start values, also used for printing
         * and store it to EEPROM
         */
        StartValues.initialDischargingMillivolt = sBatteryInfo.VoltageNoLoadMillivolt;
        StartValues.initialDischargingMilliampere = sBatteryInfo.Milliampere;
        StartValues.initialDischargingMilliohm = sBatteryInfo.Milliohm;
        StartValues.BatteryTypeIndex = sBatteryInfo.TypeIndex;
        StartValues.LoadResistorMilliohm = sCurrentLoadResistorAverage;
        StartValues.CapacityMilliampereHour = 0; // Capacity is written at the end or computed while reading
        eeprom_write_block(&StartValues, &EEPROMStartValues, sizeof(EEPROMStartValues));

        /*
         * Initially set up structure for delta storage
         */
        ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt = sBatteryInfo.VoltageNoLoadMillivolt;
        ValuesForDeltaStorage.lastStoredMilliampere = sBatteryInfo.Milliampere;
        ValuesForDeltaStorage.lastStoredMilliohm = sBatteryInfo.Milliohm;
        ValuesForDeltaStorage.tempDeltaIsEmpty = true;
        ValuesForDeltaStorage.DeltaArrayIndex++;

    } else if (ValuesForDeltaStorage.DeltaArrayIndex <= NUMBER_OF_2_COMPRESSED_SAMPLES) {
        if (!sOnlyPlotterOutput) {
            if (ValuesForDeltaStorage.tempDeltaIsEmpty) {
                Serial.print(F("Store values to EEPROM compress buffer"));
            } else {
                Serial.print(F("Store values to EEPROM at index "));
                Serial.print(ValuesForDeltaStorage.DeltaArrayIndex);
            }
        }
        /*
         * Append value to delta values array
         */
        int16_t tVoltageDelta = sBatteryInfo.VoltageNoLoadMillivolt - ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt;
        tVoltageDelta = store4BitDeltas(tVoltageDelta, &ValuesForDeltaStorage.tempVoltageDelta,
                reinterpret_cast<uint8_t*>(&sMillivoltDeltaArrayEEPROM[ValuesForDeltaStorage.DeltaArrayIndex]));
        ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt += tVoltageDelta;

        int16_t tMilliampereDelta = sBatteryInfo.Milliampere - ValuesForDeltaStorage.lastStoredMilliampere;
        tMilliampereDelta = store4BitDeltas(tMilliampereDelta, &ValuesForDeltaStorage.tempMilliampereDelta,
                reinterpret_cast<uint8_t*>(&sMilliampereDeltaArrayEEPROM[ValuesForDeltaStorage.DeltaArrayIndex]));
        ValuesForDeltaStorage.lastStoredMilliampere += tMilliampereDelta;

        int16_t tMilliohmDelta = sBatteryInfo.Milliohm - ValuesForDeltaStorage.lastStoredMilliohm;
        tMilliohmDelta = store4BitDeltas(tMilliohmDelta, &ValuesForDeltaStorage.tempMilliohmDelta,
                reinterpret_cast<uint8_t*>(&sMilliohmDeltaArrayEEPROM[ValuesForDeltaStorage.DeltaArrayIndex]));
        ValuesForDeltaStorage.lastStoredMilliohm += tMilliohmDelta;

        if (!sOnlyPlotterOutput) {
            Serial.println();
        }

        if (ValuesForDeltaStorage.tempDeltaIsEmpty) {
            ValuesForDeltaStorage.tempDeltaIsEmpty = false;
        } else {
            ValuesForDeltaStorage.DeltaArrayIndex++; // increase every second sample
            ValuesForDeltaStorage.tempDeltaIsEmpty = true;
        }
    }

    if (ValuesForDeltaStorage.DeltaArrayIndex == NUMBER_OF_2_COMPRESSED_SAMPLES && ValuesForDeltaStorage.tempDeltaIsEmpty) {
        /*
         * If buffer full, store capacity and print message
         */
        storeCapacityToEEPROM();
        if (!sOnlyPlotterOutput) {
            Serial.println(F("EEPROM plotter values array full -> stop writing to EEPROM, but do not stop capacity measurement"));
        }
    }

    printValuesForPlotter(sBatteryInfo.VoltageNoLoadMillivolt, sBatteryInfo.Milliampere, sBatteryInfo.Milliohm, true);
}

void storeCapacityToEEPROM() {
    eeprom_write_word(&EEPROMStartValues.CapacityMilliampereHour, sBatteryInfo.CapacityMilliampereHour); // safety net
    if (!sOnlyPlotterOutput) {
        // Print should be done after checkForDoublePress() in order to not disturb the double press detection
        Serial.print(F("Capacity "));
        Serial.print(sBatteryInfo.CapacityMilliampereHour);
        Serial.println(F(" mAh stored"));
    }
}

/*
 * The reproduced ESR is likely to be noisy if the relation between the load resistor and the ESR is big.
 * E.g. for Li-ion we have an load resistor of 12.156 ohm and a voltage of 4.158 volt.
 * We then get an ESR of 0.109 ohm for 339 mA and 0.073 ohm for 340 mA :-(.
 */
void printValuesForPlotter(uint16_t aVoltageToPrint, uint16_t aMilliampereToPrint, uint16_t aMilliohmToPrint,
        bool aDoPrintCaption) {
    if (aDoPrintCaption) {
        // Print updated plotter caption
        Serial.print(F("Voltage="));
        printAsFloat(StartValues.initialDischargingMillivolt);
        Serial.print(F("V->"));
        printAsFloat(aVoltageToPrint);
        Serial.print(F("V:"));
        Serial.print(aVoltageToPrint);
        Serial.print(F(" Current="));
        Serial.print(StartValues.initialDischargingMilliampere);
        Serial.print(F("mA->"));
        Serial.print(aMilliampereToPrint);
        Serial.print(F("mA:"));
        Serial.print(aMilliampereToPrint);
        Serial.print(F(" ESR="));
        printAsFloat(StartValues.initialDischargingMilliohm);
        Serial.print(F("ohm->"));
        printAsFloat(aMilliohmToPrint);
        Serial.print(F("ohm:"));
        Serial.print(aMilliohmToPrint);
        Serial.print(F(" LoadResistor="));
        printAsFloat(StartValues.LoadResistorMilliohm);
        Serial.print(F("ohm Capacity="));
        Serial.print(StartValues.CapacityMilliampereHour);
        Serial.print(F("mAh Duration="));
        // We have 2 4bit values per storage byte
        uint16_t tDurationMinutes = (ValuesForDeltaStorage.DeltaArrayIndex) * (2 * STORAGE_PERIOD_SECONDS) / SECONDS_IN_ONE_MINUTE;
        Serial.print(tDurationMinutes / 60);
        Serial.print(F("h_"));
        Serial.print(tDurationMinutes % 60);
        Serial.println(F("min"));
    } else {
        Serial.print(aVoltageToPrint);
        Serial.print(' ');
        Serial.print(aMilliampereToPrint);
        Serial.print(' ');
        Serial.println(aMilliohmToPrint);
    }
}

/*
 * Reads EEPROM delta values arrays
 * - print data for plotter and compute ESR on the fly from voltage, current and load resistor
 * - compute capacity from current (if defined SUPPORT_CAPACITY_RESTORE)
 * - restore battery type and capacity accumulator as well as mAh
 * - Capacity is stored in sBatteryInfo.CapacityMilliampereHour and sBatteryInfo.CapacityAccumulator
 */
uint8_t sVoltageDeltaPrintArray[NUMBER_OF_2_COMPRESSED_SAMPLES]; // only used for readAndPrintEEPROMData(), but using local variable increases code size by 100 bytes
uint8_t sMilliampereDeltaPrintArray[NUMBER_OF_2_COMPRESSED_SAMPLES];
uint8_t sMilliohmDeltaPrintArray[NUMBER_OF_2_COMPRESSED_SAMPLES];
void readAndPrintEEPROMData() {
    /*
     * First copy EEPROM to RAM
     */
    eeprom_read_block(sVoltageDeltaPrintArray, reinterpret_cast<uint8_t*>(&sMillivoltDeltaArrayEEPROM),
    NUMBER_OF_2_COMPRESSED_SAMPLES);
    eeprom_read_block(sMilliampereDeltaPrintArray, reinterpret_cast<uint8_t*>(&sMilliampereDeltaArrayEEPROM),
    NUMBER_OF_2_COMPRESSED_SAMPLES);
    eeprom_read_block(sMilliohmDeltaPrintArray, reinterpret_cast<uint8_t*>(&sMilliohmDeltaArrayEEPROM),
    NUMBER_OF_2_COMPRESSED_SAMPLES);

    // search last non 0xFF (not cleared) value
    int tLastNonZeroIndex;
    for (tLastNonZeroIndex = (NUMBER_OF_2_COMPRESSED_SAMPLES - 1); tLastNonZeroIndex >= 0; --tLastNonZeroIndex) {
        if (sMilliampereDeltaPrintArray[tLastNonZeroIndex] != 0xFF && sMilliohmDeltaPrintArray[tLastNonZeroIndex] != 0xFF) {
            break;
        }
    }
    tLastNonZeroIndex++; // Convert from 0 to NUMBER_OF_2_COMPRESSED_SAMPLES-1 to ValuesForDeltaStorage.DeltaArrayIndex to 0 to NUMBER_OF_2_COMPRESSED_SAMPLES

    if (!sOnlyPlotterOutput) {
        Serial.print(tLastNonZeroIndex * 2);
        Serial.println(F(" compressed EEPROM values found"));
    }

    ValuesForDeltaStorage.DeltaArrayIndex = tLastNonZeroIndex;
    /*
     * Read start values data for later printing
     */
    eeprom_read_block(&StartValues, &EEPROMStartValues, sizeof(EEPROMStartValues));
    sBatteryInfo.Milliohm = StartValues.initialDischargingMilliohm; // displayed in summary

    sBatteryInfo.CapacityMilliampereHour = StartValues.CapacityMilliampereHour; // required for append to EEPROM functionality
    sBatteryInfo.TypeIndex = StartValues.BatteryTypeIndex;

    uint16_t tVoltage = StartValues.initialDischargingMillivolt;
    uint16_t tMilliampere = StartValues.initialDischargingMilliampere;
    uint16_t tMilliohm = StartValues.initialDischargingMilliohm;

    uint32_t tCapacityAccumulator = tMilliampere;

    /*
     * Print the initial value and no caption to plotter
     */
    printValuesForPlotter(tVoltage, tMilliampere, tMilliohm, false);

    // DeltaArrayIndex can be from 0 to NUMBER_OF_2_COMPRESSED_SAMPLES
    for (int i = 0; i < ValuesForDeltaStorage.DeltaArrayIndex; ++i) {
        if (!sOnlyPlotterOutput) {
            Serial.print(F("EEPROM Values=0x"));
            Serial.print(sVoltageDeltaPrintArray[i], HEX);
            Serial.print(F(" 0x"));
            Serial.print(sMilliampereDeltaPrintArray[i], HEX);
            Serial.print(F(" 0x"));
            Serial.println(sMilliohmDeltaPrintArray[i], HEX);
        }
        /*
         * Process first part of compressed data
         */
        uint8_t t4BitVoltageDelta = sVoltageDeltaPrintArray[i];
        int8_t tFirst4BitDelta = (t4BitVoltageDelta >> 4) - 8;
        tVoltage += tFirst4BitDelta;

        uint8_t t4BitMilliampereDelta = sMilliampereDeltaPrintArray[i];
        tFirst4BitDelta = (t4BitMilliampereDelta >> 4) - 8;
        tMilliampere += tFirst4BitDelta;

        uint8_t t4BitMilliohmDelta = sMilliohmDeltaPrintArray[i];
        tFirst4BitDelta = (t4BitMilliohmDelta >> 4) - 8;
        tMilliohm += tFirst4BitDelta;

        tCapacityAccumulator += tMilliampere; // putting this into printValuesForPlotter() increases program size
        /*
         * Print first uncompressed values
         */
        if (!sOnlyPlotterOutput || ValuesForDeltaStorage.DeltaArrayIndex < (MAX_VALUES_DISPLAYED_IN_PLOTTER / 2)) {
            /*
             *  Skip every second (this one, since we printed initial value before loop) value,
             *  if we have more than 500 uncompressed (250 compressed) values, to fit the graph into the plotter display
             */
            printValuesForPlotter(tVoltage, tMilliampere, tMilliohm, false);
        }

        /*
         * Process second part of compressed data
         */
        int8_t tSecond4BitDelta = (t4BitVoltageDelta & 0x0F) - 8;
        tVoltage += tSecond4BitDelta;

        tSecond4BitDelta = (t4BitMilliampereDelta & 0x0F) - 8;
        tMilliampere += tSecond4BitDelta;

        tSecond4BitDelta = (t4BitMilliohmDelta & 0x0F) - 8;
        tMilliohm += tSecond4BitDelta;

        /*
         * Restoring capacity value from stored data, which have a bigger sample interval than measured data.
         * The observed delta was around 1%, so we can use this as a fallback, if no capacity data was stored e.g. in case of a sudden power down.
         * Increases program size by 184 bytes.
         */
        tCapacityAccumulator += tMilliampere;

        /*
         * Print second uncompressed values
         * Print the caption with values from the end of the measurement cycle to plotter
         */
        printValuesForPlotter(tVoltage, tMilliampere, tMilliohm, (i == ValuesForDeltaStorage.DeltaArrayIndex - 1));
    }

    uint16_t tCurrentCapacityMilliampereHourComputed = tCapacityAccumulator / (3600L / STORAGE_PERIOD_SECONDS);
    if (sBatteryInfo.CapacityMilliampereHour == 0) {
        if (!sOnlyPlotterOutput) {
            Serial.print(F("No Capacity stored, so use computed capacity of "));
            Serial.print(tCurrentCapacityMilliampereHourComputed);
            Serial.println(F(" mAh"));
        }
        sBatteryInfo.CapacityMilliampereHour = tCurrentCapacityMilliampereHourComputed;
    } else {
        if (!sOnlyPlotterOutput) {
            /*
             * The observed delta was around 1% :-)
             */
            int16_t tCurrentCapacityMilliampereHourDelta = sBatteryInfo.CapacityMilliampereHour
                    - tCurrentCapacityMilliampereHourComputed;
            Serial.print(F("Stored minus computed capacity = "));
            Serial.print(tCurrentCapacityMilliampereHourDelta);
            Serial.println(F(" mAh"));
        }
    }

    // restore capacity accumulator
    sBatteryInfo.CapacityAccumulator = sBatteryInfo.CapacityMilliampereHour
            * ((3600L * MILLIS_IN_ONE_SECOND) / SAMPLE_PERIOD_MILLIS);

    if (!sOnlyPlotterOutput) {
        Serial.print(F("EEPROM values: "));
    }

    /*
     * Store current values in structure for delta storage for append functionality
     */
    ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt = tVoltage;
    ValuesForDeltaStorage.lastStoredMilliampere = tMilliampere;
    ValuesForDeltaStorage.lastStoredMilliohm = tMilliohm;
    ValuesForDeltaStorage.tempDeltaIsEmpty = true;

    /*
     * Print battery values, but use a different state for formatting
     */
    printBatteryValues();
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    myLCD.print(getVCCVoltage(), 1);
    myLCD.print(F("V Stored data"));
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
}

void TogglePin(uint8_t aPinNr) {
    if (digitalRead(aPinNr) == HIGH) {
        digitalWrite(aPinNr, LOW);
    } else {
        digitalWrite(aPinNr, HIGH);
    }
}

void LCDClearLine(uint8_t aLineNumber) {
#if defined(USE_LCD)
    myLCD.setCursor(0, aLineNumber);
    myLCD.print("                    ");
    myLCD.setCursor(0, aLineNumber);
#endif
}
