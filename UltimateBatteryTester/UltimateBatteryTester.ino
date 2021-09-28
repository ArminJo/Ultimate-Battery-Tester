/*
 *  UltimateBatteryTester.ino
 *  Test internal resistance (ESR) of batteries and acquire and display the discharge graph
 *
 *  To suspend a measurement while in storage mode, press single for storage of current capacity
 *  If measurement is stopped, it can be started by another press
 *  If pin 11 connected to ground, verbose output for Arduino Serial Monitor is enabled. This is not suitable for Arduino Plotter.
 *
 *  Copyright (C) 2021  Armin Joachimsmeyer
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
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

#include "ADCUtils.h"
#include "pitches.h"

/*
 * Version 1.0 - 9/2021
 *    Tested version.
 * Version 0.0 - 9/2021
 *    Initial version.
 */

#define VERSION_EXAMPLE "1.0"

/*
 * You should calibrate your ADC readout by replacing this value with the voltage you measured a the AREF pin after the program started.
 * For my Nanos I measured e.g. 1060 mV and 1093 mV.
 */
#ifndef ADC_INTERNAL_REFERENCE_MILLIVOLT
#define ADC_INTERNAL_REFERENCE_MILLIVOLT 1100L    // Value measured at the AREF pin
#endif

/*
 * Activate the type of LCD you use
 * Default is parallel LCD with 2 rows of 16 characters (1602).
 * Serial LCD uses A4/A5 - the hardware I2C pins on Arduino
 */
#if !defined(USE_SERIAL_LCD) && !defined(USE_PARALLEL_LCD) && !defined(USE_NO_LCD)
#define USE_PARALLEL_LCD
#endif
//#define USE_SERIAL_LCD

/*
 * Pin and ADC definitions
 */
#define ADC_CHANNEL_VOLTAGE          0    // ADC0
#define ADC_CHANNEL_CURRENT          1    // ADC1
#define PIN_VOLTAGE_RANGE_EXTENSION A2    // This pin is low to extend the voltage range from 2.2 volt to 4.4 volt
#define PIN_LOAD_HIGH               A3    // This pin is high to switch on the high load (3 ohm)
#if defined(USE_SERIAL_LCD)
#define PIN_LOAD_LOW                12    // This pin is high to switch on the low load (10 ohm). A4 is occupied by I2C.
#else
#define PIN_LOAD_LOW                A4    // This pin is high to switch on the low load (10 ohm)
#endif
#define PIN_TONE                    10
#define PIN_SERIAL_MONITOR_OUTPUT   11      // If connected to ground, verbose output for Arduino Serial Monitor is enabled. This is not suitable for Arduino Plotter.

/*
 * External circuit definitions
 */
#define SHUNT_RESISTOR_MILLIOHM                 2000L  // 2 ohm
#define LOAD_LOW_MILLIOHM                       (1000 + SHUNT_RESISTOR_MILLIOHM) // Additional 1 ohm
#define LOAD_HIGH_MILLIOHM                      (10 * 1000 + SHUNT_RESISTOR_MILLIOHM) // Additional 10 ohm
#define ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE    2L       // Divider with 100 kOhm and 100 kOhm
#define ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE   4L       // Divider with 100 kOhm and 33.333 kOhm

#define NO_LOAD     0
#define LOW_LOAD    1 // 10 Ohm
#define HIGH_LOAD   2 // 3 Ohm

/*
 * Values for different battery types
 */
struct BatteryTypeInfoStruct {
    const char TypeName[11];
    uint16_t NominalVoltageMillivolt; // Not used yet
    uint16_t DetectionThresholdVoltageMillivolt; // type is detected if voltage is below this threshold
    uint16_t SwitchOffVoltageMillivolt;
    uint8_t LoadType; // High or low
};
#define NO_BATTERY_INDEX    0
struct BatteryTypeInfoStruct BatteryTypeInfoArray[] = { { "No battery", 0, 1000, 0, NO_LOAD }, { "NiCd NiMH ", 1200, 1420, 1100,
HIGH_LOAD }, { "Alkali    ", 1500, 1550, 1300, HIGH_LOAD }, { "NiZn batt.", 1650, 1800, 1500, HIGH_LOAD }, { "LiFePO    ", 3200,
        3400, 2700, LOW_LOAD }, { "LiPo batt.", 3700, 4300, 3500, LOW_LOAD }, { "Voltage   ", 0, 0, 0, NO_LOAD } };

/*
 * Current battery values set by getBatteryValues()
 */
struct BatteryInfoStruct {
    uint16_t VoltageNoLoadMillivolt;
    uint16_t VoltageLoadMillivolt;
    int16_t Milliampere;
    uint16_t ESRMilliohm;
    uint16_t sESRTestDeltaMillivolt = 0; // only displayed at initial ESR testing
    uint32_t CapacityAccumulator;
    uint16_t CapacityMilliampereHour;

    uint8_t LoadState;
    uint8_t TypeIndex;
} sBatteryInfo;

/*
 * current value in sCurrentLoadResistorHistory[0]. Used for computing and storing the average.
 * Precise load resistor value is required for restoring ESR from stored voltage and current.
 */
#define HISTORY_SIZE_FOR_AVERAGE 16
uint16_t sCurrentLoadResistorHistory[HISTORY_SIZE_FOR_AVERAGE];
uint16_t sCurrentLoadResistorAverage;

/*
 * EEPROM array for discharging graph
 * Every array byte contains two 4 bit values
 * The upper 4 bit store the first value, the lower 4 bit store the second value
 * 8 is added to the 4 bit integer (range from -8 and 7) to get positive values for storage
 */
struct ValuesForDeltaStorageStruct {
    uint16_t lastStoredMilliampere;
    uint16_t lastStoredVoltageNoLoadMillivolt;
    uint8_t tempMilliampereDelta;
    uint8_t tempVoltageDelta;
    bool tempDeltaIsEmpty;
    int DeltaArrayIndex; // The index of the next values to be written. -1 to signal, that start values must be written.
} ValuesForDeltaStorage;

/*
 * EEPROM store
 */
#define NUMBER_OF_SAMPLES 500 // 8h 20min for 30 seconds per sample
EEMEM int8_t sVoltageDeltaArrayEEPROM[NUMBER_OF_SAMPLES];
EEMEM int8_t sMilliampereDeltaArrayEEPROM[NUMBER_OF_SAMPLES];
// The start values for the delta array
struct EEPROMStartValuesStruct {
    uint16_t initialDischargingVoltage;
    uint16_t initialDischargingMilliampere;

    uint16_t LoadResistorMilliohm;
    uint8_t BatteryTypeIndex;

    uint16_t CapacityMilliampereHour; // is set at end of measurement or by store button
};
EEMEM struct EEPROMStartValuesStruct EEPROMStartValues;

void getBatteryVoltageMillivolt();
void detectAndPrintBatteryType();
void getBatteryCurrent();
void getBatteryValues();
void checkStopCondition();
void playEndTone(void);
void setLoad(uint8_t aNewLoadState);
void printBatteryValues();
void computeESRAndPrintValuesForPlotter(uint16_t aVoltageToPrint, uint16_t aMilliampereToPrint, bool aDoPrintCaption);
void printAsFloat(int16_t aValueInMillis);
void LCDPrintAsFloat(int16_t aValueInMillis);
void storeBatteryValues();
void printEEPROMData();

void switchToStateStoreToEEPROM();
void switchToStateInitialized();

void TogglePin(uint8_t aPinNr);
void LCDClearLine(uint8_t aLineNumber);

/*
 * Imports and definitions for continue button at pin 2
 */
#define USE_BUTTON_0  // Enable code for button 0 at INT0.
#include "EasyButtonAtInt01.hpp"
void handleStartStopButtonPress(bool aButtonToggleState);    // The button press callback function
EasyButton Button0AtPin2(&handleStartStopButtonPress); // Button is connected to INT0 (pin2)

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

#define MILLIS_IN_ONE_SECOND 1000L

/*
 * Measurement timing
 */
#define LOAD_SWITCH_SETTLE_TIME_MILLIS  5   // Time for voltage to settle after load switch

//#define TEST
#if defined(TEST)
// to speed up testing the code
#define STORAGE_PERIOD_SECONDS  5
#define SAMPLE_PERIOD_MILLIS    500
#define STATE_INITIAL_ESR_DURATION_SECONDS  4
#else
#define STATE_INITIAL_ESR_DURATION_SECONDS  30 // 30 seconds before starting discharge and plotting, to have time to just test for ESR of battery.
#define STORAGE_PERIOD_SECONDS          30L // 30 seconds
#define SAMPLE_PERIOD_MILLIS            MILLIS_IN_ONE_SECOND
#define VCC_CHECK_PERIOD_SECONDS        (60 * 5L) // 5 minutes
#define VCC_CHECK_THRESHOLD_MILLIVOLT  3500 // 3.5 volt
#endif

unsigned long sLastMillisOfStorage;
unsigned long sLastMillisOfSample;
unsigned long sLastMillisOfVCCCheck = VCC_CHECK_PERIOD_SECONDS * MILLIS_IN_ONE_SECOND; // to force first check at startup
unsigned long sFirstMillisOfESRCheck;

/*
 * Tester state machine
 */
#define STATE_SETUP               0
#define STATE_INITIALIZED         1 // start of 1. loop, if no button pressed before
#define STATE_INITIAL_ESR         2 // only voltage and ESR measurement every n seconds for STATE_INITIAL_ESR_DURATION_SECONDS seconds
#define STATE_STORE_TO_EEPROM     3
#define STATE_STOPPED  4 // Switch off voltage reached, until removal of battery
volatile uint8_t sMeasurementState = STATE_SETUP;

bool sOnlyPlotterOutput;
//#define DEBUG

/*
 * Program starts here
 */
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_LOAD_HIGH, OUTPUT);
    pinMode(PIN_LOAD_LOW, OUTPUT);
    pinMode(PIN_SERIAL_MONITOR_OUTPUT, INPUT_PULLUP);
    setLoad(NO_LOAD);
    digitalWrite(PIN_VOLTAGE_RANGE_EXTENSION, LOW); // prepare for later use

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif

    sOnlyPlotterOutput = digitalRead(PIN_SERIAL_MONITOR_OUTPUT);
    if (!sOnlyPlotterOutput) {
        // Just to know which program is running on my Arduino
        Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));
    } else {
        Serial.println(F("Battery Tester"));
    }

    // Disable  digital input on all unused ADC channel pins to reduce power consumption
    DIDR0 = ADC0D | ADC1D;
    /*
     * We must wait for ADC channel to switch to 1.1 volt reference
     */
    checkAndWaitForReferenceAndChannelToSwitch(ADC_CHANNEL_VOLTAGE, INTERNAL);

#if defined(USE_LCD)
    /*
     * LCD initialization
     */
#if defined(USE_SERIAL_LCD)
    myLCD.init();
    myLCD.clear();
    myLCD.backlight();
#endif
#if defined(USE_PARALLEL_LCD)
    myLCD.begin(LCD_COLUMNS, LCD_ROWS);
#endif
    myLCD.setCursor(0, 0);
    myLCD.print(F("Battery Tester "));
    myLCD.setCursor(0, 1);
    myLCD.print(F(VERSION_EXAMPLE "  " __DATE__));
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
//    myLCD.setCursor(3, 1);
//    // Clear part of date not overwritten by " No batt." message
//    myLCD.print(F("     "));
#endif
    tone(PIN_TONE, 2200, 100); // usage of tone() costs 1524 bytes code space
    printEEPROMData();
    // check if button press has not changed state before
    if (sMeasurementState == STATE_SETUP) {
        switchToStateInitialized();
    }
    delay(2 * LCD_MESSAGE_PERSIST_TIME_MILLIS);
    LCDClearLine(0);
}

/*
 * The main loop with a delay of 100 ms
 */
void loop() {
    sOnlyPlotterOutput = digitalRead(PIN_SERIAL_MONITOR_OUTPUT);

    if (millis() - sLastMillisOfVCCCheck >= VCC_CHECK_PERIOD_SECONDS * MILLIS_IN_ONE_SECOND) {
        sLastMillisOfVCCCheck = millis();
        if (getVCCVoltageMillivolt() < VCC_CHECK_THRESHOLD_MILLIVOLT) {
#if defined(USE_LCD)
            myLCD.setCursor(0, 0);
            myLCD.print(F("VCC undervoltage"));
            myLCD.setCursor(0, 1);
            myLCD.blink();
            myLCD.print(F("     "));
            myLCD.print(getVCCVoltage(), 2);
            myLCD.print('V');
            myLCD.print(F("      "));
#endif
            for (uint8_t i = 0; i < 5; ++i) {
                playEndTone(); // 3 seconds
                delay(2000);
            }
            myLCD.noBlink();
            LCDClearLine(0);
            LCDClearLine(1);
        }
    }

    if (millis() - sLastMillisOfSample >= SAMPLE_PERIOD_MILLIS) {
        sLastMillisOfSample = millis();
        /*
         * Do this every second
         */

        if (sMeasurementState == STATE_INITIALIZED) {
            /*
             * Blocking wait for battery to be inserted
             */
            detectAndPrintBatteryType(); // This activates the load before return
            if (sMeasurementState == STATE_INITIALIZED) {
                sMeasurementState = STATE_INITIAL_ESR;
                if (!sOnlyPlotterOutput) {
                    Serial.println(F("Switch to state INITIAL_ESR"));
                }
                sFirstMillisOfESRCheck = millis();
            }
        }

        uint16_t tLastBatteryVoltageNoLoadMillivolt = sBatteryInfo.VoltageNoLoadMillivolt; // store current no load voltage for display at "No batt." detection

        if (sMeasurementState == STATE_STOPPED) {
            getBatteryVoltageMillivolt(); // get current battery no load voltage
        } else {
            getBatteryValues(); // must be called only once per sample!
        }
        /*
         * Check for removed battery
         */
        if (sBatteryInfo.VoltageNoLoadMillivolt < 100) {
#if defined(USE_LCD)
            if (sMeasurementState != STATE_SETUP) {
                // move current displayed voltage right if we already have one
                for (int i = 0; i < 10; ++i) {
                    myLCD.setCursor(i, 0);
                    myLCD.print(' ');
                    LCDPrintAsFloat(tLastBatteryVoltageNoLoadMillivolt);
                    myLCD.print('V');
                    delay(120);
                }
            }
#endif
            switchToStateInitialized();
        }

        printBatteryValues(); // do no battery detection before!

        /*
         * Check for end of STATE_INITIAL_ESR
         */
        if (sMeasurementState == STATE_INITIAL_ESR
                && millis() - sFirstMillisOfESRCheck >= (STATE_INITIAL_ESR_DURATION_SECONDS * MILLIS_IN_ONE_SECOND)) {
            /*
             * Print message
             */
            if (!sOnlyPlotterOutput) {
                Serial.println(F("Press button \"Continue\" to append values to already stored EEPROM data"));
            }
#if defined(USE_LCD)
            myLCD.setCursor(0, 0);
            myLCD.print(F("Press cont. to  "));
            myLCD.setCursor(0, 1);
            myLCD.print(F("append to data  "));
#endif

            /*
             * and wait for 2 seconds for button press
             */
            for (uint8_t i = 0; i < 10; ++i) {
                delay(LCD_MESSAGE_PERSIST_TIME_MILLIS / 10);
                if (sMeasurementState != STATE_INITIAL_ESR) {
                    // button press sets state to discharge
                    break;
                }
            }
            LCDClearLine(1);

            // If button was not pressed before, start a new data set
            if (sMeasurementState == STATE_INITIAL_ESR) {
                // Force new data set
                ValuesForDeltaStorage.DeltaArrayIndex = -1;
                sBatteryInfo.CapacityAccumulator = 0;
                memset(sCurrentLoadResistorHistory, 0, sizeof(sCurrentLoadResistorHistory));

                sLastMillisOfSample = 0; // next sample in 1 second
                switchToStateStoreToEEPROM();
            }

        } else if (sMeasurementState == STATE_STORE_TO_EEPROM) {

            /*
             * Check for periodic storage to EEPROM
             */
            if (millis() - sLastMillisOfStorage >= STORAGE_PERIOD_SECONDS * MILLIS_IN_ONE_SECOND) {
                sLastMillisOfStorage = millis();
                storeBatteryValues();
            }

            /*
             * Check for switch off voltage reached -> end of measurement
             */
            checkStopCondition();
        }

        /*
         * Toggle LED
         */
        if (sMeasurementState != STATE_STOPPED) {
            // blink feedback that measurement is running
            TogglePin(LED_BUILTIN);
        }
    }
    delay(100);
}

void switchToStateInitialized() {
    sMeasurementState = STATE_INITIALIZED;
    if (!sOnlyPlotterOutput) {
        Serial.println(F("Switch to state INITIALIZED"));
    }
}

/*
 * If button at pin 2 is pressed, the discharge data is continued at the place it was already stored in EEPROM.
 * This enables to connect the tester to the Arduino Serial Plotter at any time in the measurement without loosing data already acquired.
 * This is done because connecting to the Serial Plotter resets the tester, and to avoid to start a fresh measurement, you must press this button.
 */
void switchToStateStoreToEEPROM() {
    sMeasurementState = STATE_STORE_TO_EEPROM;
    if (!sOnlyPlotterOutput) {
        Serial.println(F("Switch to state STORE TO EEPROM"));
    }
    sLastMillisOfStorage = 0; // store first value immediately
}

void stopMeasurement() {
    sMeasurementState = STATE_STOPPED;
    if (!sOnlyPlotterOutput) {
        Serial.println(F("Switch to state STOPPED"));
    }
    setLoad(NO_LOAD);
    digitalWrite(LED_BUILTIN, LOW);
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    myLCD.print(F("Stop measurement"));
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
    LCDClearLine(0);
#endif
}

/*
 * In mode STATE_EEPROM_DATA_PRINTED, STATE_INITIAL_ESR and mode STATE_STOPPED:
 *      Starts measurement and appends data to already stored ones. -> goes to state STATE_STORE_TO_EEPROM
 * In mode STATE_STORE_TO_EEPROM:
 *      Stores current capacity and stops measurement. -> goes to state STATE_STOPPED
 * Always:
 *      Double click in 2 seconds stop measurement. -> goes to state STATE_STOPPED
 */
void handleStartStopButtonPress(bool aButtonToggleState) {
    if (Button0AtPin2.checkForDoublePress(LCD_MESSAGE_PERSIST_TIME_MILLIS)) {
        /*
         * Always stop at double press or in the first 2 seconds after booting
         * if not already stopped
         */
        if (sMeasurementState != STATE_STOPPED) {
            stopMeasurement();
        }

    } else if (sMeasurementState == STATE_STORE_TO_EEPROM) {
        /*
         * Store capacity and stop
         */
        eeprom_write_word(&EEPROMStartValues.CapacityMilliampereHour, sBatteryInfo.CapacityMilliampereHour);
        sLastMillisOfSample = millis() + LCD_MESSAGE_PERSIST_TIME_MILLIS;

#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        myLCD.print(F("Capacity stored "));
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
        stopMeasurement();

    } else if (sMeasurementState >= STATE_INITIALIZED) {
        /*
         * Continue
         * Wait for 2 seconds for double press if not double click for stop was detected
         */
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Continue button pressed"));
        }
#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        myLCD.print(F("Continue now    "));
        uint8_t tOldMeasurementState = sMeasurementState;
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS); // wait for eventual changing state to stop / STATE_STOPPED
        LCDClearLine(0);
#else
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif

        if (tOldMeasurementState == STATE_STOPPED) {
            // start anew
            switchToStateInitialized();
        } else if (sMeasurementState != STATE_STOPPED) {
            // continue storage, no stop requested during 2 seconds wait
            ValuesForDeltaStorage.tempDeltaIsEmpty = true;
//            ValuesForDeltaStorage.tempVoltageDelta = 0;
//            ValuesForDeltaStorage.tempMilliampereDelta = 0;

            sLastMillisOfSample = millis(); // next sample in 30 seconds
            switchToStateStoreToEEPROM();
        }
    }
}

void setLoad(uint8_t aNewLoadState) {
    sBatteryInfo.LoadState = aNewLoadState;
    if (aNewLoadState == NO_LOAD) {
        digitalWrite(PIN_LOAD_LOW, LOW);    // disable 10 ohm load
        digitalWrite(PIN_LOAD_HIGH, LOW);    // disable 3 ohm load
    } else if (aNewLoadState == LOW_LOAD) {
        digitalWrite(PIN_LOAD_LOW, HIGH);    // enable 10 ohm load
        digitalWrite(PIN_LOAD_HIGH, LOW);    // disable 3 ohm load
    } else {
        // Load high here
        digitalWrite(PIN_LOAD_LOW, LOW);    // disable 10 ohm load
        digitalWrite(PIN_LOAD_HIGH, HIGH);    // enable 3 ohm load
    }
}

/*
 * Sets sCurrentBatteryVoltage*Millivolt
 * Provides automatic range switch between 2.2 and 4.4 volt range
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
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Switch to high voltage range"));
        }
        tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
    }
    if (!sVoltageRangeIsLow) {
        if (tInputVoltageRaw < (((0x3F0L * ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE) / ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE) - 0x10)) {
            // switch to lower voltage range by deactivating the range extension resistor at pin A2
            sVoltageRangeIsLow = true;
            pinMode(PIN_VOLTAGE_RANGE_EXTENSION, INPUT);
            digitalWrite(PIN_VOLTAGE_RANGE_EXTENSION, LOW);
            if (!sOnlyPlotterOutput) {
                Serial.println(F("Switch to low voltage range"));
            }
            tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
        } else if (tInputVoltageRaw >= 0x3F0) {
            if (!sOnlyPlotterOutput) {
                Serial.println(F("Switch to highest voltage range"));
            }
            // switch to highest voltage range by using VCC as reference
            uint16_t tReadoutFor1_1Reference = waitAndReadADCChannelWithReference(ADC_1_1_VOLT_CHANNEL_MUX, DEFAULT); // 225 at 5 volt VCC
#ifdef DEBUG
            Serial.print(tReadoutFor1_1Reference);
#endif
            tInputVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_VOLTAGE, DEFAULT);
#ifdef DEBUG
            Serial.print(F(" - "));
            Serial.println(tInputVoltageRaw);
#endif
            tInputVoltageRaw = (tInputVoltageRaw * 1023L) / tReadoutFor1_1Reference;
        }
    }
#ifdef DEBUG
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

#ifdef DEBUG
    Serial.print(F(" -> "));
    Serial.print(tCurrentBatteryVoltageMillivolt);
    Serial.println(F(" mV"));
#endif
}

void getBatteryCurrent() {
    uint16_t tShuntVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_CURRENT, INTERNAL);
    sBatteryInfo.Milliampere =
            (((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) * tShuntVoltageRaw) / (1023L * SHUNT_RESISTOR_MILLIOHM));
}

/*
 * Assumes that load is activated before called
 */
void getBatteryValues() {
//Do it before deactivating the load
    getBatteryCurrent();
    getBatteryVoltageMillivolt();    // get current battery load voltage

//Deactivate load and wait for voltage to settle
    setLoad(NO_LOAD);
    delay(LOAD_SWITCH_SETTLE_TIME_MILLIS);

    getBatteryVoltageMillivolt();    // get current battery no load voltage
    sBatteryInfo.sESRTestDeltaMillivolt = sBatteryInfo.VoltageNoLoadMillivolt - sBatteryInfo.VoltageLoadMillivolt;

// restore original load state
    setLoad(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].LoadType);

//    Serial.print(F("Delta millivolt="));
//    Serial.println(tBatteryVoltageDeltaMillivolt);
    if (sBatteryInfo.Milliampere > 1) {
        // capacity computation
        sBatteryInfo.CapacityAccumulator += sBatteryInfo.Milliampere;
        sBatteryInfo.CapacityMilliampereHour = sBatteryInfo.CapacityAccumulator
                / ((3600L * MILLIS_IN_ONE_SECOND) / SAMPLE_PERIOD_MILLIS);    // = / 3600 for 1 s sample period

        sBatteryInfo.ESRMilliohm = (sBatteryInfo.sESRTestDeltaMillivolt * 1000L) / sBatteryInfo.Milliampere;

        /*
         * shift load resistor history array and insert current value
         */
        uint32_t tLoadResistorAverage = 0;
        for (uint8_t i = HISTORY_SIZE_FOR_AVERAGE - 1; i > 0; --i) {
            tLoadResistorAverage += sCurrentLoadResistorHistory[i - 1];
            sCurrentLoadResistorHistory[i] = sCurrentLoadResistorHistory[i - 1];
        }
        sCurrentLoadResistorHistory[0] = (sBatteryInfo.VoltageNoLoadMillivolt * 1000L / sBatteryInfo.Milliampere)
                - sBatteryInfo.ESRMilliohm;
        tLoadResistorAverage += sCurrentLoadResistorHistory[0];
        if (sCurrentLoadResistorHistory[HISTORY_SIZE_FOR_AVERAGE - 1] != 0) {
            // If array is filled up, compute rounded average load resistance value. Required for restoring battery capacity from stored data.
            sCurrentLoadResistorAverage = (tLoadResistorAverage + (HISTORY_SIZE_FOR_AVERAGE / 2)) / HISTORY_SIZE_FOR_AVERAGE;
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
 */
void checkStopCondition() {
    if (sBatteryInfo.VoltageNoLoadMillivolt < BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivolt
            && sBatteryInfo.VoltageNoLoadMillivolt > BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivolt - 100) {
        /*
         * Switch off condition met
         */
        setLoad(NO_LOAD);
        eeprom_write_word(&EEPROMStartValues.CapacityMilliampereHour, sBatteryInfo.CapacityMilliampereHour);
        sMeasurementState = STATE_STOPPED;
        if (!sOnlyPlotterOutput) {
            Serial.print(F("Switch off voltage "));
            Serial.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivolt);
            Serial.print(F(" V reached met, capacity="));
            Serial.print(sBatteryInfo.CapacityMilliampereHour);
            Serial.print(F(" mAh"));
        }
#if defined(USE_LCD)
        myLCD.setCursor(7, 0);
        myLCD.print(F(" Finished"));
#endif
        // Play short melody
        playEndTone();
    }
}

/*
 * search the "database" for a matching type
 */
uint8_t detectBatteryTypeIndex(uint16_t aBatteryVoltageMillivolt) {

// scan all threshold voltage of all battery types
    for (uint8_t i = 0; i < sizeof(BatteryTypeInfoArray) / sizeof(BatteryTypeInfoStruct) - 1; i++) {
        if (aBatteryVoltageMillivolt < BatteryTypeInfoArray[i].DetectionThresholdVoltageMillivolt) {
#ifdef DEBUG
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
 * disables the load for detecting battery type and switches it on after found
 */
void detectAndPrintBatteryType() {
    setLoad(NO_LOAD);
    do {
        getBatteryVoltageMillivolt();
        printBatteryValues();

        sBatteryInfo.TypeIndex = detectBatteryTypeIndex(sBatteryInfo.VoltageNoLoadMillivolt);
        // print values
        if (!sOnlyPlotterOutput) {
            Serial.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].TypeName);
            Serial.println(F(" found"));
        }
#if defined(USE_LCD)
        if (sBatteryInfo.TypeIndex == NO_BATTERY_INDEX) {
            myLCD.setCursor(7, 1);
            myLCD.print(F(" No batt."));
        } else {
            myLCD.setCursor(0, 1);
            myLCD.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].TypeName);
            myLCD.print(F(" found"));
        }
#endif
        TogglePin(LED_BUILTIN);
        delay(500);
    } while (sBatteryInfo.TypeIndex == NO_BATTERY_INDEX);
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#if defined(USE_LCD)
    LCDClearLine(1);
#endif
    setLoad(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].LoadType);
}

/*
 * Evaluates sMeasurementState and prints:
 *   - sCurrentBatteryVoltageMillivolt
 *   - sBatteryInfo.Milliampere
 *   - sBatteryInfo.ESRMilliohm
 *   - optional sESRTestDeltaMillivolt or capacity
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
    if (tMeasurementState != STATE_SETUP) {
        /*
         * Print voltage but not for results of EEPROM data
         */
        if (!sOnlyPlotterOutput) {
            printAsFloat(sBatteryInfo.VoltageNoLoadMillivolt);
            Serial.print(F(" V "));
        }
#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        LCDPrintAsFloat(sBatteryInfo.VoltageNoLoadMillivolt);
        myLCD.print(F("V "));
#endif
        // cursor is now at 7, 0

        /*
         * Print only voltage for this state
         * Line printed is: "0.000 V No battery detected"
         */
        if (tMeasurementState == STATE_INITIALIZED) {
            // no newline here
            return;
        } else if (tMeasurementState == STATE_STOPPED) {
            if (!sOnlyPlotterOutput) {
                Serial.println();
            }
            return;
        }

        /*
         * Here we have state STATE_INITIAL_ESR or STATE_STORE_TO_EEPROM
         */

        /*
         * Print down counter for STATE_INITIAL_ESR
         */
        if (tMeasurementState == STATE_INITIAL_ESR) {
            uint8_t tSecondsToGo = STATE_INITIAL_ESR_DURATION_SECONDS - ((millis() - sFirstMillisOfESRCheck) / 1000);
            if (!sOnlyPlotterOutput) {
                Serial.print(tSecondsToGo);
                Serial.print(F(" s ")); // seconds until discharging
            }
#if defined(USE_LCD)
            if (tSecondsToGo < 10) {
                myLCD.print(' ');
                tone(PIN_TONE, 2000, 40); // costs 1524 bytes code space
            }
            myLCD.print(tSecondsToGo);
#endif
        } else {
#if defined(USE_LCD)
            myLCD.print(F("  "));
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
            Serial.print(F(" Ohm "));
        }
#if defined(USE_LCD)
        myLCD.print(tString);
        myLCD.print(F("mA "));
#endif
    }

    /*
     * End of first row, start of second one
     */

    /*
     * Print ESR
     */
    if (!sOnlyPlotterOutput) {
        Serial.print(F(" ESR: "));
        printAsFloat(sBatteryInfo.ESRMilliohm);
        Serial.print(F(" Ohm "));
    }
#if defined(USE_LCD)
    myLCD.setCursor(0, 1);
    LCDPrintAsFloat(sBatteryInfo.ESRMilliohm);
    myLCD.print(F("\xF4   ")); // Ohm symbol
#endif

    if (tMeasurementState == STATE_INITIAL_ESR) {
        /*
         * Print voltage at load
         */
        if (!sOnlyPlotterOutput) {
            printAsFloat(sBatteryInfo.sESRTestDeltaMillivolt);
            Serial.print(F(" V "));
        }
#if defined(USE_LCD)
        LCDPrintAsFloat(sBatteryInfo.sESRTestDeltaMillivolt);
        myLCD.print(F("V "));
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

void computeESRAndPrintValuesForPlotter(uint16_t aVoltageToPrint, uint16_t aMilliampereToPrint, bool aDoPrintCaption) {
    int16_t tESRToPrint = ((aVoltageToPrint * 1000L) / aMilliampereToPrint) - sCurrentLoadResistorAverage;
    Serial.print(aVoltageToPrint);
    Serial.print(' ');
    Serial.print(aMilliampereToPrint);
    Serial.print(' ');
    Serial.println(tESRToPrint);
    sBatteryInfo.ESRMilliohm = tESRToPrint;
    if (aDoPrintCaption) {
        // Print updated plotter caption
        Serial.print(F("Voltage="));
        printAsFloat(aVoltageToPrint);
        Serial.print(F("mV Current="));
        Serial.print(aMilliampereToPrint);
        Serial.print(F("mA ESR="));
        printAsFloat(tESRToPrint);
        Serial.print(F("Ohm LoadResistor="));
        printAsFloat(sCurrentLoadResistorAverage);
        Serial.print(F("Ohm Capacity="));
        Serial.print(sBatteryInfo.CapacityMilliampereHour);
        Serial.print(F("mAh Duration="));
        // We have 2 4bit values per storage byte
        uint16_t tDurationMinutes = (ValuesForDeltaStorage.DeltaArrayIndex + 1) / ((60 / STORAGE_PERIOD_SECONDS) / 2);
        Serial.print(tDurationMinutes / 60);
        Serial.print(F("h_"));
        Serial.print(tDurationMinutes % 60);
        Serial.println(F("min"));
    }
}

void printAsFloat(int16_t aValueInMillis) {
    Serial.print(((float) (aValueInMillis)) / 1000, 3);
}
void LCDPrintAsFloat(int16_t aValueInMillis) {
    myLCD.print(((float) (aValueInMillis)) / 1000, 3);
}

void clearEEPROMToZero() {
    if (!sOnlyPlotterOutput) {
        Serial.println(F("Clear EEPROM"));
    }
    for (int i = 0; i < E2END; ++i) {
        eeprom_update_byte((uint8_t*) i, 0);
    }
}

/*
 * upper 4 bit store the first value (between -8 and 7), lower 4 bit store the second value
 */
int8_t store4BitDeltas(int8_t aDelta, uint8_t *aDeltaTemp, uint8_t *aEEPROMAddress) {
    if (aDelta > 7) {
        aDelta = 7;
    } else if (aDelta < -8) {
        aDelta = -8;
    }
    uint8_t tDelta = aDelta + 8; // F is 7, 8 is 0, 0 is -8 tDelta is positive :-)

    if (ValuesForDeltaStorage.tempDeltaIsEmpty) {
        *aDeltaTemp = tDelta * 16;
    } else {
        // upper 4 bit store the first value (between -8 and 7), lower 4 bit store the second value
        uint8_t tDeltaToStore = *aDeltaTemp | tDelta;
        eeprom_write_byte(aEEPROMAddress, tDeltaToStore);
        if (tDeltaToStore != eeprom_read_byte(aEEPROMAddress)) {
            // Yes, I have seen this (starting with index 4 6 times 0xFF for current). Maybe undervoltage while powered by battery.
            tone(PIN_TONE, NOTE_C7, 20);
            delay(40);
            tone(PIN_TONE, NOTE_C6, 20);
            delay(40);
            tone(PIN_TONE, NOTE_C7, 20);
        }
    }

    return aDelta;
}

/*
 * Store values to EEPROM as 4 bit deltas and write them to EEPROM every second call
 */
void storeBatteryValues() {
    if (ValuesForDeltaStorage.DeltaArrayIndex < 0) {
        /*
         * Initial values
         * Storing them in a local structure and storing this, costs 50 bytes code size
         * Storing them in a global structure and storing this, costs 30 bytes code size
         */
        clearEEPROMToZero();

        eeprom_write_word(&EEPROMStartValues.initialDischargingVoltage, sBatteryInfo.VoltageNoLoadMillivolt);
        eeprom_write_word(&EEPROMStartValues.initialDischargingMilliampere, sBatteryInfo.Milliampere);
        eeprom_write_byte(&EEPROMStartValues.BatteryTypeIndex, sBatteryInfo.TypeIndex);
        eeprom_write_word(&EEPROMStartValues.LoadResistorMilliohm, sCurrentLoadResistorAverage);

        ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt = sBatteryInfo.VoltageNoLoadMillivolt;
        ValuesForDeltaStorage.lastStoredMilliampere = sBatteryInfo.Milliampere;
        ValuesForDeltaStorage.tempDeltaIsEmpty = true;
        ValuesForDeltaStorage.DeltaArrayIndex++;

    } else if (ValuesForDeltaStorage.DeltaArrayIndex < NUMBER_OF_SAMPLES) {
        /*
         * Append value to delta values array
         */
        int8_t tVoltageDelta = sBatteryInfo.VoltageNoLoadMillivolt - ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt;
        tVoltageDelta = store4BitDeltas(tVoltageDelta, &ValuesForDeltaStorage.tempVoltageDelta,
                reinterpret_cast<uint8_t*>(&sVoltageDeltaArrayEEPROM[ValuesForDeltaStorage.DeltaArrayIndex]));
        ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt += tVoltageDelta;

        int8_t tMilliampereDelta = sBatteryInfo.Milliampere - ValuesForDeltaStorage.lastStoredMilliampere;
        tMilliampereDelta = store4BitDeltas(tMilliampereDelta, &ValuesForDeltaStorage.tempMilliampereDelta,
                reinterpret_cast<uint8_t*>(&sMilliampereDeltaArrayEEPROM[ValuesForDeltaStorage.DeltaArrayIndex]));
        ValuesForDeltaStorage.lastStoredMilliampere += tMilliampereDelta;

        if (ValuesForDeltaStorage.tempDeltaIsEmpty) {
            ValuesForDeltaStorage.tempDeltaIsEmpty = false;
        } else {
            ValuesForDeltaStorage.DeltaArrayIndex++; // increase every second sample
            ValuesForDeltaStorage.tempDeltaIsEmpty = true;
        }
    }

    computeESRAndPrintValuesForPlotter(sBatteryInfo.VoltageNoLoadMillivolt, sBatteryInfo.Milliampere, true);
}

/*
 * Reads EEPROM delta values arrays
 * If the arrays each contain more than MINIMUM_NUMBER_OF_SAMPLES_FOR_CONTINUE entries,
 * then:
 *      - print data for plotter and compute ESR on the fly from voltage, current and load resistor
 *      - compute capacity from current
 *      - restore battery type and capacity accumulator as well as mAh
 */
uint8_t sVoltageDeltaPrintArray[NUMBER_OF_SAMPLES]; // only used for printEEPROMData(), but using local variable increases code size by 100 bytes
uint8_t sMilliampereDeltaPrintArray[NUMBER_OF_SAMPLES];
void printEEPROMData() {
    /*
     * First copy EEPROM to RAM
     */
    eeprom_read_block(sVoltageDeltaPrintArray, reinterpret_cast<uint8_t*>(&sVoltageDeltaArrayEEPROM),
    NUMBER_OF_SAMPLES);
    eeprom_read_block(sMilliampereDeltaPrintArray, reinterpret_cast<uint8_t*>(&sMilliampereDeltaArrayEEPROM),
    NUMBER_OF_SAMPLES);

// search last non zero (not cleared) value
    int tLastNonZeroIndex;
    for (tLastNonZeroIndex = NUMBER_OF_SAMPLES - 1; tLastNonZeroIndex >= 0; --tLastNonZeroIndex) {
        if (sVoltageDeltaPrintArray[tLastNonZeroIndex] != 0) {
            break;
        }
    }
    tLastNonZeroIndex++; // convert to ValuesForDeltaStorage.DeltaArrayIndex from 0 to NUMBER_OF_SAMPLES

    ValuesForDeltaStorage.DeltaArrayIndex = tLastNonZeroIndex;
    uint16_t tVoltage = eeprom_read_word(&EEPROMStartValues.initialDischargingVoltage);
    uint16_t tMilliampere = eeprom_read_word(&EEPROMStartValues.initialDischargingMilliampere);
    sBatteryInfo.CapacityMilliampereHour = eeprom_read_word(&EEPROMStartValues.CapacityMilliampereHour);
    sCurrentLoadResistorAverage = eeprom_read_word(&EEPROMStartValues.LoadResistorMilliohm);
    sBatteryInfo.TypeIndex = eeprom_read_byte(&EEPROMStartValues.BatteryTypeIndex);

#if defined SUPPORT_CAPACITY_RESTORE
    uint32_t tCapacityAccumulator = tMilliampere;
#endif
    computeESRAndPrintValuesForPlotter(tVoltage, tMilliampere, false);

    for (int i = 0; i < ValuesForDeltaStorage.DeltaArrayIndex; ++i) {

        if (!sOnlyPlotterOutput) {
            Serial.print(F("EEPROM Values=0x"));
            Serial.print(sVoltageDeltaPrintArray[i], HEX);
            Serial.print(F(" 0x"));
            Serial.println(sMilliampereDeltaPrintArray[i], HEX);
        }
        uint8_t t4BitVoltageDelta = sVoltageDeltaPrintArray[i];
        int8_t tFirst4BitDelta = (t4BitVoltageDelta >> 4) - 8;
        tVoltage += tFirst4BitDelta;

        uint8_t t4BitMilliampereDelta = sMilliampereDeltaPrintArray[i];
        tFirst4BitDelta = (t4BitMilliampereDelta >> 4) - 8;
        tMilliampere += tFirst4BitDelta;
#if defined SUPPORT_CAPACITY_RESTORE
        tCapacityAccumulator += tMilliampere; // putting this into computeESRAndPrintValuesForPlotter increases program size
#endif
        computeESRAndPrintValuesForPlotter(tVoltage, tMilliampere, false);

        int8_t tSecond4BitDelta = (t4BitVoltageDelta & 0x0F) - 8;
        tVoltage += tSecond4BitDelta;

        tSecond4BitDelta = (t4BitMilliampereDelta & 0x0F) - 8;
        tMilliampere += tSecond4BitDelta;
#if defined SUPPORT_CAPACITY_RESTORE
        tCapacityAccumulator += tMilliampere;
#endif
        if (i == ValuesForDeltaStorage.DeltaArrayIndex / 2) {
            computeESRAndPrintValuesForPlotter(tVoltage, tMilliampere, true);
        } else {
            computeESRAndPrintValuesForPlotter(tVoltage, tMilliampere, false);
        }
    }

#if defined SUPPORT_CAPACITY_RESTORE
    uint16_t tCurrentCapacityMilliampereHourComputed = tCapacityAccumulator / (3600L / STORAGE_PERIOD_SECONDS);
    if (sBatteryInfo.CapacityMilliampereHour == 0) {
        sBatteryInfo.CapacityMilliampereHour = tCurrentCapacityMilliampereHourComputed;
    }

    if (!sOnlyPlotterOutput) {
        /*
         * The observed delta was around 1% :-)
         */
        int16_t tCurrentCapacityMilliampereHourDelta = sBatteryInfo.CapacityMilliampereHour - tCurrentCapacityMilliampereHourComputed;
        Serial.print(F("Stored - computed capacity="));
        Serial.print(tCurrentCapacityMilliampereHourDelta);
        Serial.println(F("mAh"));
    }
#endif

// restore capacity accumulator
    sBatteryInfo.CapacityAccumulator = sBatteryInfo.CapacityMilliampereHour * ((3600L * MILLIS_IN_ONE_SECOND) / SAMPLE_PERIOD_MILLIS);

    printBatteryValues(); // must be before LCD print below, since state may be STOPPED here.
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    myLCD.print(getVCCVoltage(), 1);
    myLCD.print(F("V Stored data"));

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
