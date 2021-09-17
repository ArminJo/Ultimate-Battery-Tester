/*
 *  UltimateBatteryTester.ino
 *  Test internal resistance (ESR) of batteries and acquire and display the discharge graph
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
 * Version 0.9 - 9/2021
 * Initial version.
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
#define PIN_TONE                    11

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
uint8_t sMilliampereDeltaTemp;
uint8_t sVoltageDeltaTemp;
bool sDeltaTempIsEmpty;
uint16_t sESRTestDeltaMillivolt = 0; // only displayed at initial ESR testing
uint32_t sCapacityAccumulator;
uint16_t sCurrentCapacityMilliamperehour;
uint8_t sCurrentLoadState;
uint8_t sCurrentBatteryTypeIndex;

/*
 * EEPROM array for discharging graph
 * Every array byte contains two 4 bit values
 * The upper 4 bit store the first value, the lower 4 bit store the second value
 * 8 is added to the 4 bit integer (range from -8 and 7) to get positive values for storage
 */
#define NUMBER_OF_SAMPLES 500 // 8h 20min for 30 seconds per sample
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

#define ENABLE_PLOTTER_OUTPUT
#define ENABLE_PLOTTER_MILLIAMPERE_OUTPUT

/*
 * Tester state machine
 */
#define STATE_INITIAL             0
#define STATE_EEPROM_DATA_PRINTED 1
#define STATE_NO_BATTERY_DETECTED 2
#define STATE_INITIAL_ESR         3 // only voltage and ESR measurement every n seconds for STATE_INITIAL_ESR_DURATION_SECONDS seconds
#define STATE_STORE_TO_EEPROM     4
#define STATE_DISCHARGE_FINISHED  5 // Switch off voltage reached, until removal of battery
volatile uint8_t sMeasurementState = STATE_INITIAL;

void getBatteryVoltage();
void detectAndPrintBatteryType();
void getLoadCurrent();
void getBatteryValues();
void setLoad(uint8_t aNewLoadState);
void printBatteryValues();
void printAsFloat(uint16_t aValueInMillis);
void LCDPrintAsFloat(uint16_t aValueInMillis);
void storePlotterDataToEEPROM();
void printEEPROMData();

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
#define SAMPLE_PERIOD_MILLIS            1000
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
//    myLCD.setCursor(3, 1);
//    // Clear part of date not overwritten by " No batt." message
//    myLCD.print(F("     "));
#endif
    tone(PIN_TONE, 2200, 100); // costs 1524 bytes code space
    printEEPROMData();
    if (!Button0AtPin2.ButtonStateHasJustChanged) {
        sMeasurementState = STATE_EEPROM_DATA_PRINTED;
    }
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
            if (!Button0AtPin2.ButtonStateHasJustChanged) {
                sMeasurementState = STATE_INITIAL_ESR;
#if !defined(ENABLE_PLOTTER_OUTPUT)
                Serial.println(F("Switch to state INITIAL_ESR"));
#endif
                sFirstMillisOfESRCheck = millis();
            }
        }

        if (sMeasurementState == STATE_DISCHARGE_FINISHED) {
            getBatteryVoltage(); // get current battery load voltage
        } else {
            getBatteryValues(); // must be called only once per sample!
        }
        /*
         * Check for removed battery
         */
        if (sCurrentBatteryVoltageNoLoadMillivolt < 100) {
#if defined(USE_LCD)
            if (sMeasurementState != STATE_INITIAL) {
                // move current displayed voltage right if we already have one
                for (int i = 0; i < 10; ++i) {
                    myLCD.setCursor(i, 0);
                    myLCD.print(' ');
                    LCDPrintAsFloat(sLastBatteryVoltageNoLoadMillivolt);
                    myLCD.print('V');
                    delay(120);
                }
            }
#endif
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
                tone(PIN_TONE, NOTE_G6);
                delay(150);
                tone(PIN_TONE, NOTE_E6);
                delay(150);
                tone(PIN_TONE, NOTE_C6, 250);

#if !defined(ENABLE_PLOTTER_OUTPUT)
                Serial.print(F("Battery voltage is now lower than switch off voltage "));
                printAsFloat(BatteryTypeInfoArray[sCurrentBatteryTypeIndex].SwitchOffVoltageMillivolt);
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
#if defined(ENABLE_PLOTTER_MILLIAMPERE_OUTPUT)
    Serial.println(F("Voltage[mV] Current[mA] ESR[mOhm]"));
#else
    Serial.println(F("Voltage[mV] ESR[mOhm]"));
#endif
    sLastMillisOfStorage = 0; // store first value immediately
}

void handleContinueButtonPress(bool aButtonToggleState) {
    if (Button0AtPin2.checkForDoublePress(LCD_MESSAGE_PERSIST_TIME_MILLIS)) {
        setLoad(NO_LOAD);
        sMeasurementState = STATE_DISCHARGE_FINISHED;
#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        myLCD.print(F("Stop measurement"));
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
        LCDClearLine(0);
#endif

    } else if (sMeasurementState == STATE_STORE_TO_EEPROM) {
        eeprom_write_word(&sCurrentCapacityMilliamperehourEEPROM, sCurrentCapacityMilliamperehour);
#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        myLCD.print(F("Capacity stored "));
        sLastMillisOfSample = millis() + LCD_MESSAGE_PERSIST_TIME_MILLIS;
#endif

    } else if (sMeasurementState <= STATE_INITIAL_ESR) {
#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        myLCD.print(F("Continue now    "));
        LCDClearLine(1);
        // wait for 2 seconds for double press
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
 * Sets sCurrentBatteryVoltage*Millivolt
 * Provides automatic range switch between 2.2 and 4.4 volt range
 * Does not affect the loads
 */
void getBatteryVoltage() {
    static bool sVoltageRangeIsLow = true;

    uint16_t tInputVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
    /*
     * Automatic range
     */
    if (sVoltageRangeIsLow && tInputVoltageRaw >= 0x3F0) {
        // switch to higher voltage range by activating the range extension resistor at pin A2
        sVoltageRangeIsLow = false;
        pinMode(PIN_VOLTAGE_RANGE_EXTENSION, OUTPUT);
        digitalWrite(PIN_VOLTAGE_RANGE_EXTENSION, LOW); // required???
#if !defined(ENABLE_PLOTTER_OUTPUT)
        Serial.println(F("Switch to high voltage range"));
#endif
        tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
    }
    if (!sVoltageRangeIsLow) {
        if (tInputVoltageRaw < (((0x3F0L * ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE) / ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE) - 0x10)) {
            // switch to lower voltage range by deactivating the range extension resistor at pin A2
            sVoltageRangeIsLow = true;
            pinMode(PIN_VOLTAGE_RANGE_EXTENSION, INPUT);
            digitalWrite(PIN_VOLTAGE_RANGE_EXTENSION, LOW);
#if !defined(ENABLE_PLOTTER_OUTPUT)
            Serial.println(F("Switch to low voltage range"));
#endif
            tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
        } else if (tInputVoltageRaw >= 0x3F0) {
#if !defined(ENABLE_PLOTTER_OUTPUT)
            Serial.println(F("Switch to highest voltage range"));
#endif
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

    if (sCurrentLoadState == NO_LOAD) {
        sCurrentBatteryVoltageNoLoadMillivolt = tCurrentBatteryVoltageMillivolt;
    } else {
        sCurrentBatteryVoltageLoadMillivolt = tCurrentBatteryVoltageMillivolt;
    }

#ifdef DEBUG
    Serial.print(F(" -> "));
    Serial.print(tCurrentBatteryVoltageMillivolt);
    Serial.println(F(" mV"));
#endif
}

void getLoadCurrent() {
    uint16_t tShuntVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_CURRENT, INTERNAL);
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
        sCurrentCapacityMilliamperehour = sCapacityAccumulator / ((3600L * 1000) / SAMPLE_PERIOD_MILLIS); // = / 3600 for 1 s sample period

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
        getBatteryVoltage();
        printBatteryValues();

        sCurrentBatteryTypeIndex = detectBatteryTypeIndex(sCurrentBatteryVoltageNoLoadMillivolt);
        // print values
#if !defined(ENABLE_PLOTTER_OUTPUT)
        Serial.print(BatteryTypeInfoArray[sCurrentBatteryTypeIndex].TypeName);
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

    uint8_t tMeasurementState = sMeasurementState; // Because sMeasurementState is volatile
    if (tMeasurementState != STATE_INITIAL) {
        /*
         * Print voltage but not for results of EEPROM data
         */
#if !defined(ENABLE_PLOTTER_OUTPUT)
        printAsFloat(sCurrentBatteryVoltageNoLoadMillivolt);
        Serial.print(F(" V "));
#endif
#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        LCDPrintAsFloat(sCurrentBatteryVoltageNoLoadMillivolt);
        myLCD.print(F("V "));
#endif
        // cursor is now at 7, 0

        /*
         * Print only voltage for this states
         */
        if (tMeasurementState <= STATE_NO_BATTERY_DETECTED) {
            // no newline here since line printed is: "0.000 V No battery detected"
            return;
        } else if (tMeasurementState == STATE_DISCHARGE_FINISHED) {
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
        if (tMeasurementState == STATE_INITIAL_ESR) {
            uint8_t tSecondsToGo = STATE_INITIAL_ESR_DURATION_SECONDS - ((millis() - sFirstMillisOfESRCheck) / 1000);
#if !defined(ENABLE_PLOTTER_OUTPUT)
            Serial.print(tSecondsToGo);
            Serial.print(F(" s ")); // seconds until discharging
#endif
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
        sprintf_P(tString, PSTR("%4u"), sCurrentLoadMilliampere);
#if !defined(ENABLE_PLOTTER_OUTPUT)
        Serial.print(tString);
        Serial.print(F(" mA at "));
        printAsFloat(sCurrentLoadResistorHistory[0]);
        Serial.print(F(" Ohm "));
#endif
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
#if !defined(ENABLE_PLOTTER_OUTPUT)
    Serial.print(F(" ESR: "));
    printAsFloat(sCurrentBatteryESRMilliohm);
    Serial.print(F(" Ohm "));
#endif
#if defined(USE_LCD)
    myLCD.setCursor(0, 1);
    LCDPrintAsFloat(sCurrentBatteryESRMilliohm);
    myLCD.print(F("\xF4   ")); // Ohm symbol
#endif

    if (tMeasurementState == STATE_INITIAL_ESR) {
        /*
         * Print voltage at load
         */
#if !defined(ENABLE_PLOTTER_OUTPUT)
        printAsFloat(sESRTestDeltaMillivolt);
        Serial.print(F(" V "));
#endif
#if defined(USE_LCD)
        LCDPrintAsFloat(sESRTestDeltaMillivolt);
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

/*
 * upper 4 bit store the first value (between -8 and 7), lower 4 bit store the second value
 */
int8_t store4BitDeltas(int8_t aDelta, uint8_t *aDeltaTemp, uint8_t *aEEPROMAddress) {
    if (aDelta > 7) {
        aDelta = 7;
    } else if (aDelta < -8) {
        aDelta = -8;
    }
    uint8_t tDelta = aDelta + 8;  // tDelta is positive :-)

    if (sDeltaTempIsEmpty) {
        *aDeltaTemp = tDelta * 16;
    } else {
        // upper 4 bit store the first value (between -8 and 7), lower 4 bit store the second value
        uint8_t tDeltaToStore = *aDeltaTemp | tDelta;
        eeprom_write_byte(aEEPROMAddress, tDeltaToStore);
    }

    return aDelta;
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
#if defined(ENABLE_PLOTTER_MILLIAMPERE_OUTPUT)
        Serial.print(F("Voltage[mV] Current[mA] ESR[mOhm] LoadResistor="));
#else
        Serial.print(F("Voltage[mV] ESR[mOhm] LoadResistor="));
#endif
        printAsFloat(tLoadResistorAverage);
        Serial.println(F("Ohm"));
        sLastStoredBatteryVoltageNoLoadMillivolt = sCurrentBatteryVoltageNoLoadMillivolt;
        sLastStoredBatteryMilliampere = sCurrentLoadMilliampere;
        sDeltaTempIsEmpty = true;
        sDeltaArrayIndex++;

    } else if (sDeltaArrayIndex < NUMBER_OF_SAMPLES) {
        /*
         * Append value to delta values array
         */
        int8_t tVoltageDelta = sCurrentBatteryVoltageNoLoadMillivolt - sLastStoredBatteryVoltageNoLoadMillivolt;
        tVoltageDelta = store4BitDeltas(tVoltageDelta, &sVoltageDeltaTemp,
                reinterpret_cast<uint8_t*>(&sVoltageDeltaArrayEEPROM[sDeltaArrayIndex]));
        sLastStoredBatteryVoltageNoLoadMillivolt += tVoltageDelta;

        int8_t tMilliampereDelta = sCurrentLoadMilliampere - sLastStoredBatteryMilliampere;
        tMilliampereDelta = store4BitDeltas(tMilliampereDelta, &sMilliampereDeltaTemp,
                reinterpret_cast<uint8_t*>(&sMilliampereDeltaArrayEEPROM[sDeltaArrayIndex]));
        sLastStoredBatteryMilliampere += tMilliampereDelta;

        if (sDeltaTempIsEmpty) {
            sDeltaTempIsEmpty = false;
        } else {
            sDeltaArrayIndex++; // increase every second sample
            sDeltaTempIsEmpty = true;
        }
    }
    Serial.print(sCurrentBatteryVoltageNoLoadMillivolt);
    Serial.print(' ');
#if defined(ENABLE_PLOTTER_MILLIAMPERE_OUTPUT)
    Serial.print(sCurrentLoadMilliampere);
    Serial.print(' ');
#endif
    Serial.println(sCurrentBatteryESRMilliohm);
}

void PrintValuesForPlotter(uint16_t aMilliampere, uint16_t aVoltageToPrint) {
    int16_t tESRToPrint = ((aVoltageToPrint * 1000L) / aMilliampere) - sCurrentLoadResistorHistory[0];
    Serial.print(aVoltageToPrint);
    Serial.print(' ');
#if defined(ENABLE_PLOTTER_MILLIAMPERE_OUTPUT)
    Serial.print(aMilliampere);
    Serial.print(' ');
#endif
    Serial.println(tESRToPrint);
    sCurrentBatteryESRMilliohm = tESRToPrint;
}

void printAsFloat(uint16_t aValueInMillis) {
    Serial.print(((float) (aValueInMillis)) / 1000, 3);
}
void LCDPrintAsFloat(uint16_t aValueInMillis) {
    myLCD.print(((float) (aValueInMillis)) / 1000, 3);
}

/*
 * Reads EEPROM delta values arrays
 * If the arrays each contain more than MINIMUM_NUMBER_OF_SAMPLES_FOR_CONTINUE entries,
 * then:
 *      - print data for plotter and compute ESR on the fly from voltage, current and load resistor
 *      - compute capacity from current
 *      - restore battery type and capacity accumulator as well as mAh
 */
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
    tLastNonZeroIndex++; // convert to sDeltaArrayIndex from 0 to NUMBER_OF_SAMPLES

    sDeltaArrayIndex = tLastNonZeroIndex;
    uint16_t tVoltageToPrint = eeprom_read_word(&sInitialDischargingVoltageEEPROM);
    uint16_t tMilliampere = eeprom_read_word(&sInitialDischargingMilliampereEEPROM);
    uint16_t tStoredCapacityMilliamperehour = eeprom_read_word(&sCurrentCapacityMilliamperehourEEPROM);
    sCurrentLoadResistorHistory[0] = eeprom_read_word(&sLoadResistorEEPROM);
    sCurrentBatteryTypeIndex = eeprom_read_byte(&BatteryTypeIndexEEPROM);

// Print plotter caption
#if defined(ENABLE_PLOTTER_MILLIAMPERE_OUTPUT)
    Serial.print(F("Voltage[mV] Current[mA] ESR[mOhm] LoadResistor="));
#else
    Serial.print(F("Voltage[mV] ESR[mOhm] LoadResistor="));
#endif
    printAsFloat(sCurrentLoadResistorHistory[0]);
    Serial.println(F("Ohm"));

    uint32_t tCapacityAccumulator = tMilliampere;

    PrintValuesForPlotter(tMilliampere, tVoltageToPrint);

    for (int i = 0; i < sDeltaArrayIndex; ++i) {

        uint8_t t4BitVoltageDelta = sVoltageDeltaPrintArray[i];
        int8_t tFirst4BitDelta = (t4BitVoltageDelta >> 4) - 8;
        tVoltageToPrint += tFirst4BitDelta;

        uint8_t t4BitMilliampereDelta = sMilliampereDeltaPrintArray[i];
        tFirst4BitDelta = (t4BitMilliampereDelta >> 4) - 8;
        tMilliampere += tFirst4BitDelta;
        tCapacityAccumulator += tMilliampere; // putting this into PrintValuesForPlotter increases program size

        PrintValuesForPlotter(tMilliampere, tVoltageToPrint);

        int8_t tSecond4BitDelta = (t4BitVoltageDelta & 0x0F) - 8;
        tVoltageToPrint += tSecond4BitDelta;

        tSecond4BitDelta = (t4BitMilliampereDelta & 0x0F) - 8;
        tMilliampere += tSecond4BitDelta;
        tCapacityAccumulator += tMilliampere;

        PrintValuesForPlotter(tMilliampere, tVoltageToPrint);
    }

#if defined(ENABLE_PLOTTER_MILLIAMPERE_OUTPUT)
    Serial.print(F("Voltage[mV] Current[mA] ESR[mOhm] LoadResistor="));
#else
        Serial.print(F("Voltage[mV] ESR[mOhm] LoadResistor="));
#endif
    printAsFloat(sCurrentLoadResistorHistory[0]);
    Serial.print(F("Ohm Capacity="));

    uint16_t tCurrentCapacityMilliamperehourComputed = tCapacityAccumulator / (3600L / STORAGE_PERIOD_SECONDS);
    if (tStoredCapacityMilliamperehour == 0) {
        sCurrentCapacityMilliamperehour = tCurrentCapacityMilliamperehourComputed;
        Serial.print(sCurrentCapacityMilliamperehour);
    } else {
        int16_t tCurrentCapacityMilliamperehourDelta = tStoredCapacityMilliamperehour - tCurrentCapacityMilliamperehourComputed;
        sCurrentCapacityMilliamperehour = tStoredCapacityMilliamperehour;
        Serial.print(sCurrentCapacityMilliamperehour);
        Serial.print(F("mAh_Delta="));
        Serial.print(tCurrentCapacityMilliamperehourDelta);
    }

    Serial.print(F("mAh Duration="));
    uint16_t tDurationMinutes = (sDeltaArrayIndex - 2) * (2 * STORAGE_PERIOD_SECONDS) / 60;
    Serial.print(tDurationMinutes / 60);
    Serial.print(F("h_"));
    Serial.print(tDurationMinutes % 60);
    Serial.println(F("min"));

// restore capacity accumulator
    sCapacityAccumulator = sCurrentCapacityMilliamperehour * ((3600L * 1000) / SAMPLE_PERIOD_MILLIS);
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    myLCD.print(F("Stored data     "));
#endif
    printBatteryValues();
    delay(2 * LCD_MESSAGE_PERSIST_TIME_MILLIS);
    LCDClearLine(0);
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
