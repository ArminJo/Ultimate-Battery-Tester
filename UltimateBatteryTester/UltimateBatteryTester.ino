/*
 *  BatteryTester.ino
 *  Test internal resistance of batteries
 *
 *  If a LiPo supply is detected, the LCD display timing for standalone usage (without serial connection) is activated.
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

/*
 * Version 0.9 - 9/2021
 * Initial version.
 */

#define VERSION_EXAMPLE "1.0"

/*
 * You shoukd calibrate your ADC readout by replacing this value with the voltage you measured a the AREF pin after the program started.
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
#if !defined(USE_SERIAL_LCD) && !defined(USE_NO_LCD)
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

/*
 * External circuit definitions
 */
#define SHUNT_RESISTOR_MILLIOHM                 2000  // 2 ohm
#define LOAD_LOW_MILLIOHM                       (1000 + SHUNT_RESISTOR_MILLIOHM) // Additional 1 ohm
#define LOAD_HIGH_MILLIOHM                      (10000 + SHUNT_RESISTOR_MILLIOHM) // Additional 10 ohm
#define SHUNT_RESISTOR_MILLIOHM                 2000  // 2 ohm
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
    uint16_t NominalVoltageMillivolt;
    uint16_t SwitchOffVoltageMillivolt;
    uint8_t LoadType; // High or low
};
#define NO_BATTERY_INDEX    0
struct BatteryTypeInfoStruct BatteryTypeInfoArray[] = { { "No battery", 0, 0, NO_LOAD }, { "NiCd NiMH ", 1200, 1100, HIGH_LOAD }, {
        "Alkali    ", 1500, 1300, HIGH_LOAD }, { "NiZn batt.", 1650, 1500, HIGH_LOAD }, { "LiFePO    ", 3200, 2700, LOW_LOAD }, {
        "LiPo batt.", 3700, 3500, LOW_LOAD } };

/*
 * Variables used by the program
 */
uint16_t sCurrentBatteryESRMilliohm;
uint16_t sCurrentLoadMilliampere;
uint16_t sLastStoredBatteryMilliampere;
#define HISTORY_SIZE_FOR_AVERAGE 16
uint16_t sCurrentLoadResistorHistory[HISTORY_SIZE_FOR_AVERAGE]; // current value in sCurrentLoadResistorHistory[0]. Used for computing the average
uint16_t sCurrentBatteryVoltageNoLoadMillivolt;
uint16_t sCurrentBatteryVoltageLoadMillivolt;
uint16_t sLastBatteryVoltageNoLoadMillivolt; // Voltage to be displayed at "No batt." detection
uint16_t sLastStoredBatteryVoltageNoLoadMillivolt;
uint16_t sESRTestDeltaMillivolt = 0; // only displayed at initial ESR testing
uint32_t sCapacityAccumulator;
uint16_t sCurrentCapacityMilliamperehour;
uint8_t sCurrentLoadState;
uint8_t sCurrentBatteryTypeIndex;

/*
 * EEPROM array for discharging graph
 * 720 = 6 hours with 30 seconds per sample
 */
#define NUMBER_OF_SAMPLES 500 // 4h 10min
int sDeltaArrayIndex; // The index of the next values to be written. -1 to signal, that start values must be written.
int8_t sVoltageDeltaPrintArray[NUMBER_OF_SAMPLES];
int8_t sESRDeltaPrintArray[NUMBER_OF_SAMPLES];
int8_t sMilliampereDeltaPrintArray[NUMBER_OF_SAMPLES];
/*
 * EEPROM store
 */
EEMEM int8_t sVoltageDeltaArrayEEPROM[NUMBER_OF_SAMPLES];
EEMEM int8_t sMilliampereDeltaArrayEEPROM[NUMBER_OF_SAMPLES];
// The start values for the delta array
EEMEM uint16_t sInitialDischargingVoltageEEPROM;
EEMEM uint16_t sInitialDischargingMilliampereEEPROM;
EEMEM uint8_t BatteryTypeIndexEEPROM;
EEMEM uint16_t sLoadResistorEEPROM;
EEMEM uint16_t sCurrentCapacityMilliamperehourEEPROM;

//#define ENABLE_PLOTTER_OUTPUT

/*
 * Tester state machine
 */
#define STATE_INITIAL             0
#define STATE_NO_BATTERY_DETECTED 1
#define STATE_INITIAL_ESR         2 // only voltage and ESR measurement every n seconds for STATE_INITIAL_ESR_DURATION_SECONDS seconds
#define STATE_STORE_TO_EEPROM     3
#define STATE_DISCHARGE_FINISHED  4 // Switch off voltage reached, until removal of battery
uint8_t sMeasurementState = STATE_INITIAL;

void getBatteryVoltage();
void detectAndPrintBatteryType();
void getLoadCurrent();
void getBatteryValues();
void setLoad(uint8_t aNewLoadState);
void printBatteryValues();
void storePlotterDataToEEPROM();
void printPlotterData();

void switchToStateStoreToEEPROM();

void TogglePin(uint8_t aPinNr);
void LCDClearLine(uint8_t aLineNumber);

/*
 * Imports and definitions for continue button at pin 2
 */
#define USE_BUTTON_0  // Enable code for button 0 at INT0.
#include "EasyButtonAtInt01.hpp"
void handleContinueButtonPress(bool aButtonToggleState);    // The button press callback function
EasyButton Button0AtPin2(&handleContinueButtonPress); // Button is connected to INT0 (pin2)

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
 * Measurement timing
 */
unsigned long sLastMillisOfStorage;
unsigned long sLastMillisOfSample;
unsigned long sFirstMillisOfESRCheck;

#define LOAD_SWITCH_SETTLE_TIME_MILLIS  5   // Time for voltage to settle after load switch

//#define TEST
#if defined(TEST)
// to speed up testing the code
#define STORAGE_PERIOD_SECONDS  5
#define SAMPLE_PERIOD_MILLIS    500
#define STATE_INITIAL_ESR_DURATION_SECONDS  4
#else
#  if defined(ENABLE_PLOTTER_OUTPUT) && !defined(USE_LCD)
#define STATE_INITIAL_ESR_DURATION_SECONDS  1
#  else
#define STATE_INITIAL_ESR_DURATION_SECONDS  30 // 30 seconds before starting discharge and plotting, to have time to just test for ESR of battery.
#  endif
#define STORAGE_PERIOD_SECONDS          30 // 30 seconds
#define SAMPLE_PERIOD_MILLIS            (1000 + LOAD_SWITCH_SETTLE_TIME_MILLIS)
#endif

//#define DEBUG

/*
 * Program starts here
 */
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_LOAD_HIGH, OUTPUT);
    pinMode(PIN_LOAD_LOW, OUTPUT);
    setLoad(NO_LOAD);
    digitalWrite(PIN_VOLTAGE_RANGE_EXTENSION, LOW); // prepare for later use

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif

#if !defined(ENABLE_PLOTTER_OUTPUT)
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));
#else
    Serial.println(F("Battery Tester"));
#endif

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
    myLCD.setCursor(3, 1);
    // Clear part of date not overwritten by " No batt." message
    myLCD.print(F("    "));
#endif
    printPlotterData();

}

void loop() {

    if (millis() - sLastMillisOfSample >= SAMPLE_PERIOD_MILLIS) {
        sLastMillisOfSample = millis();
        /*
         * Do this every second
         */

        if (sMeasurementState <= STATE_NO_BATTERY_DETECTED) {
            Button0AtPin2.ButtonStateHasJustChanged = false;
            /*
             * Blocking wait for battery to be inserted
             */
            detectAndPrintBatteryType(); // This activates the load before return
            sMeasurementState = STATE_INITIAL_ESR;
            if (Button0AtPin2.ButtonStateHasJustChanged) {
                // was ignored before state change
                handleContinueButtonPress(NULL);
            } else {
#if !defined(ENABLE_PLOTTER_OUTPUT)
                Serial.println(F("Switch to state INITIAL_ESR"));
#endif
                sFirstMillisOfESRCheck = millis();
            }
        }

        getBatteryValues();
        /*
         * Check for removed battery
         */
        if (sCurrentBatteryVoltageNoLoadMillivolt < 100) {
            if (sMeasurementState != STATE_INITIAL) {
                // move current displayed voltage right if we already have one
                for (int i = 0; i < 10; ++i) {
                    myLCD.setCursor(i, 0);
                    myLCD.print(' ');
                    myLCD.print((float) (sLastBatteryVoltageNoLoadMillivolt) / 1000, 3);
                    myLCD.print('V');
                    delay(120);
                }
            }
            sMeasurementState = STATE_NO_BATTERY_DETECTED;
#if !defined(ENABLE_PLOTTER_OUTPUT)
            Serial.println(F("Switch to state NO_BATTERY_DETECTED"));
#endif
        }

        printBatteryValues(); // do no battery detection before!

        if (sMeasurementState == STATE_INITIAL_ESR) {
            /*
             * Check for end of STATE_INITIAL_ESR
             */
            if (sMeasurementState == STATE_INITIAL_ESR) {
                if (millis() - sFirstMillisOfESRCheck >= (STATE_INITIAL_ESR_DURATION_SECONDS * 1000L)) {
                    /*
                     * Print message
                     */
#if !defined(ENABLE_PLOTTER_OUTPUT)
                    Serial.println(F("Press button \"Continue\" to append values to already stored EEPROM data"));
#endif
                    myLCD.setCursor(0, 0);
                    myLCD.print(F("Press cont. to  "));
                    myLCD.setCursor(0, 1);
                    myLCD.print(F("append to data  "));

                    /*
                     * and wait for 2 seconds for button press
                     */
                    for (uint8_t i = 0; i < 10; ++i) {
                        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS / 10);
                        if (Button0AtPin2.ButtonStateHasJustChanged) {
                            // button press sets state to discharge
                            break;
                        }
                    }

                    // If button was not pressed before, start a new data set
                    if (!Button0AtPin2.ButtonStateHasJustChanged) {
                        sDeltaArrayIndex = -1;
                        sCapacityAccumulator = 0;
                        switchToStateStoreToEEPROM();
                    }
                }
            }

        } else if (sMeasurementState == STATE_STORE_TO_EEPROM) {

            /*
             * Check for periodic storage to EEPROM
             */
            if (millis() - sLastMillisOfStorage >= STORAGE_PERIOD_SECONDS * 1000L) {
                sLastMillisOfStorage = millis();
                storePlotterDataToEEPROM();
                Serial.print(sCurrentBatteryVoltageNoLoadMillivolt);
                Serial.print(' ');
                Serial.println(sCurrentBatteryESRMilliohm);
            }

            /*
             * Check for switch off voltage reached -> end of measurement
             */
            if (sCurrentBatteryVoltageNoLoadMillivolt < BatteryTypeInfoArray[sCurrentBatteryTypeIndex].SwitchOffVoltageMillivolt
                    && sCurrentBatteryVoltageNoLoadMillivolt
                            > BatteryTypeInfoArray[sCurrentBatteryTypeIndex].SwitchOffVoltageMillivolt - 100) {
                setLoad(NO_LOAD);
                eeprom_write_word(&sCurrentCapacityMilliamperehourEEPROM, sCurrentCapacityMilliamperehour);
                sMeasurementState = STATE_DISCHARGE_FINISHED;

#if !defined(ENABLE_PLOTTER_OUTPUT)
                Serial.print(F("Battery voltage is now lower than switch off voltage "));
                Serial.print((float) (BatteryTypeInfoArray[sCurrentBatteryTypeIndex].SwitchOffVoltageMillivolt) / 1000, 3);
                Serial.println(F(" V"));
                Serial.println(F("Switch to state DISCHARGE_FINISHED"));
#endif
#if defined(USE_LCD)
                myLCD.setCursor(7, 0);
                myLCD.print(F(" Finished"));
#endif
            }
        }

        /*
         * Toggle LED
         */
        if (sMeasurementState != STATE_DISCHARGE_FINISHED) {
            // blink feedback that measurement is running
            TogglePin(LED_BUILTIN);
        }
    }
}

/*
 * If button at pin 2 is pressed, the discharge data is continued at the place it was already stored in EEPROM.
 * This enables to connect the tester to the Arduino Serial Plotter at any time in the measurement without loosing data already acquired.
 * This is done because connecting to the Serial Plotter resets the tester, and to avoid to start a fresh measurement, you must press this button.
 */
void switchToStateStoreToEEPROM() {
    sMeasurementState = STATE_STORE_TO_EEPROM;
#if !defined(ENABLE_PLOTTER_OUTPUT)
    Serial.println(F("Switch to state DISCHARGE"));
#endif
    Serial.println(F("Voltage ESR"));
    sLastMillisOfStorage = 0; // store first value immediately
}

void handleContinueButtonPress(bool aButtonToggleState) {
    if (sMeasurementState == STATE_STORE_TO_EEPROM) {
        eeprom_write_word(&sCurrentCapacityMilliamperehourEEPROM, sCurrentCapacityMilliamperehour);
#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        myLCD.print(F("Capacity stored "));
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
    } else if (sMeasurementState == STATE_INITIAL_ESR) {
#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        myLCD.print(F("Continue now    "));
        LCDClearLine(1);
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
        switchToStateStoreToEEPROM();
    }
}

void setLoad(uint8_t aNewLoadState) {
    sCurrentLoadState = aNewLoadState;
    if (aNewLoadState == NO_LOAD) {
        digitalWrite(PIN_LOAD_LOW, LOW);    // disable 10 ohm load
        digitalWrite(PIN_LOAD_HIGH, LOW);   // disable 3 ohm load
    } else if (aNewLoadState == LOW_LOAD) {
        digitalWrite(PIN_LOAD_LOW, HIGH);    // enable 10 ohm load
        digitalWrite(PIN_LOAD_HIGH, LOW);  // disable 3 ohm load
    } else {
        // Load high here
        digitalWrite(PIN_LOAD_LOW, LOW);    // disable 10 ohm load
        digitalWrite(PIN_LOAD_HIGH, HIGH);  // enable 3 ohm load
    }
}

/*
 * Sets sCurrentBatteryVoltageMillivolt
 * Provides automatic range switch between 2.2 and 4.4 volt range
 * Does not affect the loads
 */
void getBatteryVoltage() {
    static bool sVoltageRangeIsLow = true;

    uint16_t tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
    /*
     * do automatic range
     */
    if (sVoltageRangeIsLow && tInputVoltageRaw >= 0x3F0) {
        // switch to higher voltage range by activating the range extension resistor at pin A2
        sVoltageRangeIsLow = false;
        pinMode(PIN_VOLTAGE_RANGE_EXTENSION, OUTPUT);
        digitalWrite(PIN_VOLTAGE_RANGE_EXTENSION, LOW); // required???
        Serial.println(F("Switch to high voltage range"));
        tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
    }
    if (!sVoltageRangeIsLow
            && tInputVoltageRaw
                    < (((0x3F0L * ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE) / ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE) - 0x10)) {
        // switch to lower voltage range by deactivating the range extension resistor at pin A2
        sVoltageRangeIsLow = true;
        pinMode(PIN_VOLTAGE_RANGE_EXTENSION, INPUT);
        digitalWrite(PIN_VOLTAGE_RANGE_EXTENSION, LOW);
        Serial.println(F("Switch to low voltage range"));
        tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
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

    if (sCurrentLoadState == NO_LOAD) {
        sCurrentBatteryVoltageNoLoadMillivolt = tCurrentBatteryVoltageMillivolt;
    } else {
        sCurrentBatteryVoltageLoadMillivolt = tCurrentBatteryVoltageMillivolt;
    }

#ifdef DEBUG
    Serial.print(F(" -> "));
    Serial.print(sCurrentBatteryVoltageMillivolt);
    Serial.println(F(" mV"));
#endif
}

void getLoadCurrent() {
    uint16_t tShuntVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_CURRENT, INTERNAL);
    sCurrentLoadMilliampere = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) * tShuntVoltageRaw) / (1023L * SHUNT_RESISTOR_MILLIOHM));
}

/*
 * Assumes that load is activated before called
 */
void getBatteryValues() {
    //Do it before deactivating the load
    getLoadCurrent();
    getBatteryVoltage(); // get current battery load voltage

    //Deactivate load and wait for voltage to settle
    setLoad(NO_LOAD);
    delay(LOAD_SWITCH_SETTLE_TIME_MILLIS);

    sLastBatteryVoltageNoLoadMillivolt = sCurrentBatteryVoltageNoLoadMillivolt; // store current no load voltage for display at "No batt." detection
    getBatteryVoltage(); // get current battery no load voltage
    sESRTestDeltaMillivolt = sCurrentBatteryVoltageNoLoadMillivolt - sCurrentBatteryVoltageLoadMillivolt;

    // restore original load state
    setLoad(BatteryTypeInfoArray[sCurrentBatteryTypeIndex].LoadType);

//    Serial.print(F("Delta millivolt="));
//    Serial.println(tBatteryVoltageDeltaMillivolt);
    if (sCurrentLoadMilliampere > 1) {
        sCapacityAccumulator += sCurrentLoadMilliampere;
        sCurrentCapacityMilliamperehour = sCapacityAccumulator / ((3600L * 1000) / SAMPLE_PERIOD_MILLIS);

        sCurrentBatteryESRMilliohm = (sESRTestDeltaMillivolt * 1000L) / sCurrentLoadMilliampere;

        /*
         * shift load resistor history array and insert current value
         */
        for (uint8_t i = HISTORY_SIZE_FOR_AVERAGE - 1; i > 0; --i) {
            sCurrentLoadResistorHistory[i] = sCurrentLoadResistorHistory[i - 1];
        }
        sCurrentLoadResistorHistory[0] = (sCurrentBatteryVoltageNoLoadMillivolt * 1000L / sCurrentLoadMilliampere)
                - sCurrentBatteryESRMilliohm;
    }
}

/*
 * search the "database" for a matching type
 */
uint8_t detectBatteryTypeIndex(uint16_t aBatteryVoltageMillivolt) {

// scan all threshold voltage of all battery types
    for (uint8_t i = sizeof(BatteryTypeInfoArray) / sizeof(BatteryTypeInfoStruct) - 1; i > 0; i--) {
        if (aBatteryVoltageMillivolt >= BatteryTypeInfoArray[i].SwitchOffVoltageMillivolt) {
#ifdef DEBUG
            Serial.print(F(" Battery index="));
            Serial.print(i);
            Serial.print(F(" BatteryVoltageMillivolt="));
            Serial.print(aBatteryVoltageMillivolt);
            Serial.print(F(" SwitchOffVoltageMillivolt="));
            Serial.println(BatteryTypeInfoArray[i].SwitchOffVoltageMillivolt);
#endif
            return i;
        }
    }
// no Battery detected
    return NO_BATTERY_INDEX;
}

/*
 * disables the load for detecting battery type and switches it on after found
 */
void detectAndPrintBatteryType() {
    setLoad(NO_LOAD);
    do {
        getBatteryVoltage();
        printBatteryValues();

        sCurrentBatteryTypeIndex = detectBatteryTypeIndex(sCurrentBatteryVoltageNoLoadMillivolt);
        // print values
        Serial.print(BatteryTypeInfoArray[sCurrentBatteryTypeIndex].TypeName);
#if defined(ENABLE_PLOTTER_OUTPUT)
        Serial.println();
#else
        Serial.println(F(" found"));
#endif
#if defined(USE_LCD)
        if (sCurrentBatteryTypeIndex == NO_BATTERY_INDEX) {
            myLCD.setCursor(7, 1);
            myLCD.print(F(" No batt."));
        } else {
            myLCD.setCursor(0, 1);
            myLCD.print(BatteryTypeInfoArray[sCurrentBatteryTypeIndex].TypeName);
            myLCD.print(F(" found"));
        }
#endif
        TogglePin(LED_BUILTIN);
        delay(500);
    } while (sCurrentBatteryTypeIndex == NO_BATTERY_INDEX);
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#if defined(USE_LCD)
    LCDClearLine(1);
#endif
    setLoad(BatteryTypeInfoArray[sCurrentBatteryTypeIndex].LoadType);
}

/*
 * Evaluates sMeasurementState and prints:
 *   - sCurrentBatteryVoltageMillivolt
 *   - sCurrentLoadMilliampere
 *   - sCurrentBatteryESRMilliohm
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

    /*
     * Always print voltage
     */
#if !defined(ENABLE_PLOTTER_OUTPUT)
    Serial.print((float) (sCurrentBatteryVoltageNoLoadMillivolt) / 1000, 3);
    Serial.print(F(" V "));
#endif
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    myLCD.print((float) (sCurrentBatteryVoltageNoLoadMillivolt) / 1000, 3);
    myLCD.print(F("V "));
#endif
    // cursor is now at 7, 0

    /*
     * Print only voltage for this states
     */
    if (sMeasurementState <= STATE_NO_BATTERY_DETECTED) {
        // no newline here since line printed is: "0.000 V No battery detected"
        return;
    } else if (sMeasurementState == STATE_DISCHARGE_FINISHED) {
#if !defined(ENABLE_PLOTTER_OUTPUT)
        Serial.println();
#endif
        return;
    }

    /*
     * Here we have state STATE_INITIAL_ESR or STATE_STORE_TO_EEPROM
     */

    /*
     * Print down counter for STATE_INITIAL_ESR
     */
    if (sMeasurementState == STATE_INITIAL_ESR) {
        uint8_t tSecondsToGo = STATE_INITIAL_ESR_DURATION_SECONDS - ((millis() - sFirstMillisOfESRCheck) / 1000);
#if !defined(ENABLE_PLOTTER_OUTPUT)
        Serial.print(tSecondsToGo);
        Serial.print(F(" s ")); // seconds until discharging
#endif
#if defined(USE_LCD)
        if (tSecondsToGo < 10) {
            myLCD.print(' ');
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
    sprintf_P(tString, PSTR("%4u"), sCurrentLoadMilliampere);
#if !defined(ENABLE_PLOTTER_OUTPUT)
    Serial.print(tString);
    Serial.print(F(" mA at "));
    Serial.print((float) (sCurrentLoadResistorHistory[0]) / 1000, 3);
    Serial.print(F(" Ohm "));
#endif
#if defined(USE_LCD)
    myLCD.print(tString);
    myLCD.print(F("mA "));
#endif

    /*
     * end of first row
     */

    /*
     * Print ESR
     */
#if !defined(ENABLE_PLOTTER_OUTPUT)
    Serial.print(F(" ESR: "));
    Serial.print((float) (sCurrentBatteryESRMilliohm) / 1000, 3);
    Serial.print(F(" Ohm "));
#endif
#if defined(USE_LCD)
    myLCD.setCursor(0, 1);
    myLCD.print((float) (sCurrentBatteryESRMilliohm) / 1000, 3);
    myLCD.print(F("\xF4   ")); // Ohm symbol
#endif

    if (sMeasurementState == STATE_INITIAL_ESR) {
        /*
         * Print voltage at load
         */
#if !defined(ENABLE_PLOTTER_OUTPUT)
        Serial.print((float) (sESRTestDeltaMillivolt) / 1000, 3);
        Serial.print(F(" V "));
#endif
#if defined(USE_LCD)
        myLCD.print((float) (sESRTestDeltaMillivolt) / 1000, 3);
        myLCD.print(F("V "));
#endif

    } else {
        /*
         * Print capacity
         */
        sprintf_P(tString, PSTR("%4u"), sCurrentCapacityMilliamperehour);
#if !defined(ENABLE_PLOTTER_OUTPUT)
        Serial.print(tString);
        Serial.print(F(" mAh"));
#endif
#if defined(USE_LCD)
        myLCD.print(tString);
        myLCD.print(F("mAh"));
#endif
    }

#if !defined(ENABLE_PLOTTER_OUTPUT)
    Serial.println();
#endif
}

void clearEEPROMToZero() {
#if !defined(ENABLE_PLOTTER_OUTPUT)
    Serial.println(F("Clear EEPROM"));
#endif
    for (int i = 0; i < E2END; ++i) {
        eeprom_update_byte((uint8_t*) i, 0);
    }
}

void storePlotterDataToEEPROM() {
    if (sDeltaArrayIndex < 0) {
        /*
         * Initial values
         */
        clearEEPROMToZero();
        eeprom_write_word(&sInitialDischargingVoltageEEPROM, sCurrentBatteryVoltageNoLoadMillivolt);
        eeprom_write_word(&sInitialDischargingMilliampereEEPROM, sCurrentLoadMilliampere);
        eeprom_write_byte(&BatteryTypeIndexEEPROM, sCurrentBatteryTypeIndex);

        // Compute load resistance. Required for restoring battery capacity from stored data.
        // get rounded load resistor average value
        uint32_t tLoadResistorAverage = 0;
        for (int i = 0; i < HISTORY_SIZE_FOR_AVERAGE; ++i) {
            tLoadResistorAverage += sCurrentLoadResistorHistory[i];
        }
        tLoadResistorAverage = (tLoadResistorAverage + (HISTORY_SIZE_FOR_AVERAGE / 2)) / HISTORY_SIZE_FOR_AVERAGE;

        eeprom_write_word(&sLoadResistorEEPROM, tLoadResistorAverage);
        Serial.print(F("Voltage ESR LoadResistor="));
        Serial.print((float) (tLoadResistorAverage) / 1000, 3);
        Serial.println(F("Ohm"));

    } else if (sDeltaArrayIndex < NUMBER_OF_SAMPLES) {
        /*
         * Array of delta values
         */
        eeprom_write_byte(reinterpret_cast<uint8_t*>(&sVoltageDeltaArrayEEPROM[sDeltaArrayIndex]),
                sCurrentBatteryVoltageNoLoadMillivolt - sLastStoredBatteryVoltageNoLoadMillivolt);
        eeprom_write_byte(reinterpret_cast<uint8_t*>(&sMilliampereDeltaArrayEEPROM[sDeltaArrayIndex]),
                sCurrentLoadMilliampere - sLastStoredBatteryMilliampere);

    }
    sDeltaArrayIndex++;
    sLastStoredBatteryVoltageNoLoadMillivolt = sCurrentBatteryVoltageNoLoadMillivolt;
    sLastStoredBatteryMilliampere = sCurrentLoadMilliampere;

}

/*
 * Reads EEPROM delta values arrays
 * If the arrays each contain more than MINIMUM_NUMBER_OF_SAMPLES_FOR_CONTINUE entries,
 * then:
 *      - print data for plotter and compute ESR on the fly from voltage, current and load resistor
 *      - compute capacity from current
 *      - restore battery type and capacity accumulator as well as mAh
 */
void printPlotterData() {
    /*
     * First copy EEPROM to RAM
     */
    eeprom_read_block(sVoltageDeltaPrintArray, reinterpret_cast<uint8_t*>(&sVoltageDeltaArrayEEPROM),
    NUMBER_OF_SAMPLES);
    eeprom_read_block(sMilliampereDeltaPrintArray, reinterpret_cast<uint8_t*>(&sMilliampereDeltaArrayEEPROM),
    NUMBER_OF_SAMPLES);

// search last non zero value
    int tLastNonZeroIndex;
    for (tLastNonZeroIndex = NUMBER_OF_SAMPLES - 1; tLastNonZeroIndex >= 0; --tLastNonZeroIndex) {
        if (sVoltageDeltaPrintArray[tLastNonZeroIndex] != 0) {
            break;
        }
    }
    tLastNonZeroIndex++; // convert to sDeltaArrayIndex

    sDeltaArrayIndex = tLastNonZeroIndex;
    uint16_t tVoltageToPrint = eeprom_read_word(&sInitialDischargingVoltageEEPROM);
    uint16_t tMilliampere = eeprom_read_word(&sInitialDischargingMilliampereEEPROM);
    uint16_t tStoredCapacityMilliamperehour = eeprom_read_word(&sCurrentCapacityMilliamperehourEEPROM);
    uint16_t tLoadResistor = eeprom_read_word(&sLoadResistorEEPROM);
    sCurrentBatteryTypeIndex = eeprom_read_byte(&BatteryTypeIndexEEPROM);

    // Print plotter caption
    Serial.print(F("Voltage ESR LoadResistor="));
    Serial.print(tLoadResistor);
    Serial.println(F("milliohm"));

    uint32_t tCapacityAccumulator = 0;
    // i = 1 gets the first ([0] element from the array
    for (int i = 0; i <= sDeltaArrayIndex; ++i) {
        int16_t tESRToPrint = ((tVoltageToPrint * 1000L) / tMilliampere) - tLoadResistor;
        tCapacityAccumulator += tMilliampere;

        Serial.print(tVoltageToPrint);
        Serial.print(' ');
        Serial.println(tESRToPrint);
        tVoltageToPrint += sVoltageDeltaPrintArray[i]; // last fetched element may be array overflow sVoltageDeltaPrintArray[NUMBER_OF_SAMPLES], but it is not used
        tMilliampere += sMilliampereDeltaPrintArray[i];
    }
    uint16_t tCurrentCapacityMilliamperehourComputed = tCapacityAccumulator / (3600L / STORAGE_PERIOD_SECONDS);

    if (tStoredCapacityMilliamperehour == 0) {
        sCurrentCapacityMilliamperehour = tCurrentCapacityMilliamperehourComputed;
    } else {
        int16_t tCurrentCapacityMilliamperehourDelta = tStoredCapacityMilliamperehour - tCurrentCapacityMilliamperehourComputed;
        Serial.print(F("Voltage ESR LoadResistor="));
        Serial.print(tLoadResistor);
        Serial.print(F("milliohm mAhDelta="));
        Serial.println(tCurrentCapacityMilliamperehourDelta);
        sCurrentCapacityMilliamperehour = tStoredCapacityMilliamperehour;
    }

    // restore capacity accumulator
    sCapacityAccumulator = (sCurrentCapacityMilliamperehour * (3600L * 1000)) / SAMPLE_PERIOD_MILLIS;
}

void TogglePin(uint8_t aPinNr) {
    if (digitalRead(aPinNr) == HIGH) {
        digitalWrite(aPinNr, LOW);
    } else {
        digitalWrite(aPinNr, HIGH);
    }
}

void LCDClearLine(uint8_t aLineNumber) {
    myLCD.setCursor(0, aLineNumber);
    myLCD.print("                    ");
    myLCD.setCursor(0, aLineNumber);
}
