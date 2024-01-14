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
 *  The first 5 hours and 37 min, data is stored to EEPROM in a delta format. this are 337 + initial samples.
 *  When EEPROM space is exhausted, data is compressed, so another 5.6 hours fit into it.
 *  This allows to store 674 samples of voltage and milliampere and results in :
 *      22h 24min at 2 minutes per storage
 *      11h 12min at 1 minute per storage, for a Li-ion this is equivalent to around 3300 mAh
 *       5h 36min at 30 seconds per storage
 *
 *  One EEPROM block contains the initial voltage and current values as well as the capacity, battery type and value of the used load resistor.
 *  These values are stored at the beginning of the measurement
 *  The compressed values are stored as 4 bit deltas and written to EEPROM every second sample.
 *  The upper 4 bit store the first value, lower 4 bit store the second value.
 *  The capacity is stored at end of measurement or on button press during the storage.
 *
 *
 *  Copyright (C) 2021-2024  Armin Joachimsmeyer
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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

#include "pitches.h"

/*
 * Version 4.0 - 12/2023
 *    Use capacity between NominalFullVoltageMillivolt and SwitchOffVoltageMillivoltHigh as standard capacity to enable better comparison.
 *    If powered by USB plotter pin logic is reversed, i.e. plotter output is enabled if NOT connected to ground.
 *    In state detecting battery, you can toggle cutoff voltage between high, low and zero (0.1 V) with stop button.
 *    Fix bug for appending to compressed data.
 *    Synchronizing of LCD access for button handler, avoiding corrupted display content.
 *    Print improvements.
 *    Support for storage period of 120 s.
 *    Compression improved for rapidly descending voltage.
 *    Moving seldom used function of pin 10 to pin A5.
 *    New Logger mode with separate shunt enabled by pin 10.
 *
 * Version 3.2.1 - 11/2023
 *    BUTTON_IS_ACTIVE_HIGH is not default any more
 * Version 3.2 - 10/2023
 *    Cutoff LCD message improved
 * Version 3.1 - 3/2023
 *    Fixed "conversion does not clear rest of EEPROM" bug
 * Version 3.0 - 12/2022
 *    Improved compression
 * Version 2.3 - 10/2022
 *    Increase no load settle time especially for NiMh batteries
 *    Attention tones
 * Version 2.2 - 8/2022
 *    ESR > 64 bug fixed.
 *    Display of changes on pin DISCHARGE_TO_LOW_PIN
 * Version 2.1 - 3/2022
 *    ESR is stored.
 * Version 2.0 - 3/2022
 *    Improved version.
 * Version 1.0 - 9/2021
 *    Tested version.
 * Version 0.0 - 9/2021
 *    Initial version.
 */

#define VERSION_EXAMPLE "4.0"
//#define DEBUG

/*
 * You should calibrate your ADC readout by replacing this value with the voltage you measured a the AREF pin after the program started.
 * For my Nanos I measured e.g. 1060 mV and 1093 mV.
 */
#if !defined(ADC_INTERNAL_REFERENCE_MILLIVOLT)
#define ADC_INTERNAL_REFERENCE_MILLIVOLT    1100L // Change to value measured at the AREF pin. If value > real AREF voltage, measured values are > real values
#endif

/*
 * Pin and ADC definitions
 * Start/Stop button is connected to INT0 pin 2
 * Pin 3 to 8 are used for parallel LCD connection
 */
#define ADC_CHANNEL_VOLTAGE          0 // ADC0 for voltage measurement
#define ADC_CHANNEL_CURRENT          1 // ADC1 for current measurement
#define ADC_CHANNEL_LOGGER_CURRENT   4 // ADC1 for current measurement for Logger
#define VOLTAGE_RANGE_EXTENSION_PIN A2 // This pin is low to extend the voltage range from 2.2 volt to 4.4 volt
#define LOAD_HIGH_PIN               A3 // This pin is high to switch on the high load (3 ohm)
// A4 + A5, the hardware I2C pins on Arduino, are used for Serial LCD
#define LOAD_LOW_PIN                12 // This pin is high to switch on the low load (10 ohm). A4 is occupied by I2C for serial LCD display.
#define BUZZER_PIN                   9
// Mode pins
#define ONLY_PLOTTER_OUTPUT_PIN     A5 // If powered by Li-ion, verbose output to Arduino Serial Monitor is disabled, if connected to ground. This is intended for Arduino Plotter mode.
#define ONLY_LOGGER_AT_A4_PIN       10 // If connected to ground, current is measured at the shunt at A4 and voltage still at A0.
// If powered by USB verbose verbose output to Arduino Serial Monitor is disabled, if NOT connected to ground.
#define DISCHARGE_TO_LOW_PIN        11 // If connected to ground, "cut off is low" is displayed and discharge ends at a lower voltage. E.g. Li-ion discharge ends at 3000 mV instead of 3500 mV

/*
 * Imports and definitions for start/stop button at pin 2
 */
#define USE_BUTTON_0                // Enable code for button 0 at INT0 / pin 2.
#define NO_BUTTON_RELEASE_CALLBACK
//#define BUTTON_IS_ACTIVE_HIGH     // If you have an active high button (sensor button) attached
#include "EasyButtonAtInt01.hpp"
void handleStartStopButtonPress(bool aButtonToggleState);   // The button press callback function
EasyButton startStopButton0AtPin2(&handleStartStopButtonPress);      // Button is connected to INT0 (pin2)
void checkForDelayedButtorProcessing(); // If LCD is in use do not process button
volatile bool sInLCDPrint;              // To synchronize LCD access for button handler
bool sOnlyPlotterOutput;                // contains the (inverted) value of the pin ONLY_PLOTTER_OUTPUT_PIN

bool sOnlyLoggerFunctionality;           // contains the (inverted) value of the pin ONLY_LOGGER_AT_A4_PIN

#define DISCHARGE_CUTOFF_LEVEL_NORMAL   0 // is default case
#define DISCHARGE_CUTOFF_LEVEL_LOW      1
#define DISCHARGE_CUTOFF_LEVEL_ZERO     2 // End discharging at 0.05 volt (DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT)
uint8_t sDischargeCutoffLevel; // One of DISCHARGE_CUTOFF_LEVEL_NORMAL, DISCHARGE_CUTOFF_LEVEL_LOW and DISCHARGE_CUTOFF_LEVEL_ZERO. Starts with the (inverted) value of the pin DISCHARGE_TO_LOW_PIN
#define DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT   50   // 50 mV
#define NO_BATTERY_MILLIVOLT            (DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT / 2)   // 25 mV
bool sLastValueOfDischargeToLowPin; // To support changing between normal and low by using pin DISCHARGE_TO_LOW_PIN

/*
 * External circuit definitions
 */
#define SHUNT_AT_A4_RESISTOR_MILLIOHM           200L  // 0.2 ohm
#define SHUNT_RESISTOR_MILLIOHM                 2000L  // 2 ohm
#define LOAD_LOW_MILLIOHM                       (1000 + SHUNT_RESISTOR_MILLIOHM) // Additional 1 ohm
#define LOAD_HIGH_MILLIOHM                      (10 * 1000 + SHUNT_RESISTOR_MILLIOHM) // Additional 10 ohm
#define ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE    2L       // Divider with 100 kOhm and 100 kOhm -> 2.2 V range
#define ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE   4L       // Divider with 100 kOhm and 33.333 kOhm -> 4.4 V range

//#define NO_TONE_WARNING_FOR_VOLTAGE_TOO_LOW_FOR_STANDARD_CAPACITY_COMPUTATION

/*
 * Activate the type of LCD you use
 * Default is parallel LCD with 2 rows of 16 characters (1602).
 * Serial LCD uses A4/A5 - the hardware I2C pins on Arduino
 */
#if !defined(USE_SERIAL_LCD) && !defined(USE_PARALLEL_LCD) && !defined(USE_NO_LCD)
#define USE_PARALLEL_LCD
#endif
//#define USE_SERIAL_LCD

// definitions for a 1602 LCD
#define LCD_COLUMNS 16
#define LCD_ROWS 2

#define LCD_MESSAGE_PERSIST_TIME_MILLIS     2000 // 2 second to view a message on LCD
#if defined(USE_SERIAL_LCD)
#include <LiquidCrystal_I2C.h> // Use an up to date library version which has the init method
#endif
#if defined(USE_PARALLEL_LCD)
#include "LiquidCrystal.h"
#endif

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
#define MILLIS_IN_ONE_SECOND 1000L
#define SECONDS_IN_ONE_MINUTE 60L
//#define TEST // to speed up testing the code
#if defined(TEST)
#define STATE_INITIAL_ESR_DURATION_SECONDS        4 // 4 seconds (-2 for initial display of append message) before starting discharge and storing, to have time to just test for ESR of battery.
#define SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS   500 // The time of the activated load for one sample.
#define NUMBER_OF_SAMPLES_PER_STORAGE             5 // 1 minute, if we have 1 sample per second
#else
#define NUMBER_OF_INITIAL_ESR_SAMPLES           30 // Before starting discharge and storing, to have time to just test for ESR of battery. 30 seconds with SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS as 1000.
#define SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS   MILLIS_IN_ONE_SECOND // 1 s. The time of the activated load for one sample.
#  if !defined(NUMBER_OF_SAMPLES_PER_STORAGE)
#define NUMBER_OF_SAMPLES_PER_STORAGE           SECONDS_IN_ONE_MINUTE // 60, if we have 1 sample per second (SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS)
//#define NUMBER_OF_SAMPLES_PER_STORAGE           (2 * SECONDS_IN_ONE_MINUTE) // 120, if we have 1 sample per second (SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS)
#  endif
#endif
#define NUMBER_OF_SECONDS_PER_STORAGE           NUMBER_OF_SAMPLES_PER_STORAGE // NUMBER_OF_SAMPLES_PER_STORAGE, if we have 1 sample per second
#define NUMBER_OF_STORAGES_PER_HOUR             (3600L / NUMBER_OF_SECONDS_PER_STORAGE)

#define MAX_VALUES_DISPLAYED_IN_PLOTTER         500 // The Arduino 1.8 Plotter displays 500 values before scrolling
#define BATTERY_DETECTION_PERIOD_MILLIS         (MILLIS_IN_ONE_SECOND / 2) // 500 ms
#define BATTERY_DETECTION_MINIMAL_MILLIVOLT     50

/*
 * Values for different battery types
 */
struct BatteryTypeInfoStruct {
    const char TypeName[11];
    uint16_t DetectionThresholdVoltageMillivolt; // Type is detected if voltage is below this threshold
    uint16_t NominalFullVoltageMillivolt;       // The voltage to start the "standard" capacity computation
    uint16_t SwitchOffVoltageMillivoltHigh;     // The voltage to stop the "standard" capacity computation
    uint16_t SwitchOffVoltageMillivoltLow;
    uint8_t LoadType;                           // High (3 Ohm) or low (12 Ohm)
    uint16_t LoadSwitchSettleTimeMillis;        // Time for voltage to settle after load switch was disabled
};

#define LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT          4300 // Maximum Voltage if fully loaded
#define LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT     4100 // Start voltage for Li-ion standard capacity measurement
#define LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT        3400 // Switch off voltage for Li-ion standard capacity measurement
#define LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW    3000 // Switch off voltage for extended capacity measurement

#define NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT          1100 // Switch off voltage for NI-MH capacity measurement
#define NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW      1000 // Switch off voltage for extended capacity measurement

#define NO_LOAD     0
#define LOW_LOAD    1 // 12 ohm
#define HIGH_LOAD   2 // 3 ohm

#define TYPE_INDEX_NO_BATTERY    0
#define TYPE_INDEX_DEFAULT       6
#define TYPE_INDEX_MAX          10

struct BatteryTypeInfoStruct BatteryTypeInfoArray[] = { { "No battery", 100, 0, 0, 0, NO_LOAD, 0 }, /* Below 100 mV and not below 50, to avoid toggling between no and low batt */
{ "Low batt. ", 1000, 0, NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW, DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT, HIGH_LOAD, 100 }, /* For researching of worn out batteries. */
{ "NiCd NiMH ", 1460, 1400, NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT, NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW, HIGH_LOAD, 100 }, /*400 mA*/
{ "Alkali    ", 1550, 1500, 1300, 1000, HIGH_LOAD, 100 }, /*500 mA*/
{ "NiZn batt.", 1800, 1650, 1500, 1300, HIGH_LOAD, 100 }, /*550 mA*/
{ "LiFePO4   ", 3400, 3400, 3000, 2700, LOW_LOAD, 10 }, /*270 mA https://www.jackery.com/blogs/knowledge/ultimate-guide-to-lifepo4-voltage-chart*/
{ "Li-ion    ", 5000, LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT /*4100*/, LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT/*3400*/,
LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW/*3V*/,
LOW_LOAD, 10 }, /*300 mA*/
{ "LiIo 2pack", 2 * LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT, 2 * LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT, 2
        * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT /*7V*/, 2 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW /*6V*/,
LOW_LOAD, 10 }, /*620 mA*/
{ "9 V Block ", 9200, 9000, 7700, 7000, LOW_LOAD, 10 }, /*750 mA => external resistor recommended*/
{ "LiIo 3pack", 3 * LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT, 3 * LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT, 3
        * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT, 3 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW, LOW_LOAD, 10 }, /*925 mA*/
{ "LiIo 4pack", 4 * LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT, 4 * LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT, 4
        * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT, 4 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW, LOW_LOAD, 10 } /*1233 mA*/};

/*
 * Current battery values set by getBatteryValues()
 */
struct BatteryInfoStruct {
    uint16_t VoltageNoLoadMillivolt;
    uint16_t VoltageLoadMillivolt;
    int16_t Milliampere;
    uint16_t Milliohm; // Average of last 60 values. ESR - Equivalent Series Resistor | internal battery resistance.
    uint16_t sESRDeltaMillivolt = 0; // only displayed at initial ESR testing
    uint32_t CapacityAccumulator;
    uint16_t CapacityMilliampereHour;
    uint16_t CapacityMilliampereHourValueAtHighCutoff;
    bool isStandardCapacity;

    uint8_t LoadState; // NO_LOAD | LOW_LOAD 12 ohm | HIGH_LOAD 3 ohm
    uint8_t TypeIndex;
} sBatteryInfo;

uint16_t sLastVoltageNoLoadMillivoltForBatteryCheck;
uint16_t sLastVoltageNoLoadMillivoltForPrintAndCountdown;
bool sBatteryWasInserted = false;
bool sBatteryValuesWerePrintedAtLeastOnce = false;

/*
 * Tester state machine
 */
#define STATE_SETUP_AND_READ_EEPROM         0
#define STATE_DETECTING_BATTERY             1 // Check if battery is inserted and determine type
#define STATE_INITIAL_ESR_MEASUREMENT       2 // Only voltage and ESR measurement every n seconds for STATE_INITIAL_ESR_DURATION_SECONDS seconds
#define STATE_STORE_TO_EEPROM               3 // Main measurement state, get values and store to EEPROM
#define STATE_STOPPED                       4 // Switch off voltage reached, until removal of battery
volatile uint8_t sMeasurementState = STATE_SETUP_AND_READ_EEPROM;

// Override defaults defined in ADCUtils.h
#define LI_ION_VCC_UNDERVOLTAGE_THRESHOLD_MILLIVOLT  3500 // 3.5 volt
#define VCC_CHECK_PERIOD_MILLIS                     (60000L) // check every minute
#define VCC_UNDERVOLTAGE_CHECKS_BEFORE_STOP         5 // Shutdown after 5 times below VCC_UNDERVOLTAGE_THRESHOLD_MILLIVOLT or below VCC_EMERGENCY_UNDERVOLTAGE_THRESHOLD_MILLIVOLT
#include "ADCUtils.hpp"

/*
 * Attention timing
 */
#define STATE_BATTERY_DETECTION_ATTENTION_PERIOD_MILLIS     (MILLIS_IN_ONE_SECOND * 60)
#define STATE_STOP_ATTENTION_PERIOD_MILLIS                  (MILLIS_IN_ONE_SECOND * 600)
unsigned long sLastStateDetectingBatteryBeepMillis;
unsigned long sLastStateStoppedBeepMillis;

unsigned long sSampleCountForStoring;
unsigned long sLastMillisOfSample = 0;
unsigned long sLastMillisOfBatteryDetection = 0;
uint8_t sNumbersOfESRChecksToGo;
uint16_t sVCCMillivolt;

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
#define HISTORY_SIZE_FOR_ESR_AVERAGE    60
uint16_t sESRHistory[HISTORY_SIZE_FOR_ESR_AVERAGE];

/*
 * EEPROM store
 */
#define FLAG_NO_COMPRESSION     0xFF
#define FLAG_COMPRESSION        0xFE
// The start values for the delta array
struct EEPROMStartValuesStruct {
    uint16_t initialDischargingMillivolt;
    uint16_t initialDischargingMilliampere;
    uint16_t initialDischargingMilliohm;
    uint8_t compressionFlag; // 0xFF -> not compressed, else compressed

    uint16_t LoadResistorMilliohm;
    uint8_t DischargeCutoffLevel;

    uint16_t CapacityMilliampereHour; // is set at end of measurement or by store button
};
#if defined(TEST)
#define MAX_NUMBER_OF_SAMPLES   9
#else
// EEPROM size for values is (1024 - sizeof(EEPROMStartValues)) / 3 = 1012 / 3 = 337.3
#define MAX_NUMBER_OF_SAMPLES      (E2END - sizeof(EEPROMStartValuesStruct)) / 3 // 337 + 1 since we always have the initial value. 5.6 h / 11.2 h for 1 minute sample rate
#endif
EEMEM int8_t sMillivoltDeltaArrayEEPROM[MAX_NUMBER_OF_SAMPLES];
EEMEM int8_t sMilliampereDeltaArrayEEPROM[MAX_NUMBER_OF_SAMPLES];
EEMEM int8_t sMilliohmDeltaArrayEEPROM[MAX_NUMBER_OF_SAMPLES];

EEMEM struct EEPROMStartValuesStruct EEPROMStartValues;
struct EEPROMStartValuesStruct StartValues;
/*
 * Every compressed array byte contains two 4 bit values
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
    bool compressionIsActive;
    int DeltaArrayIndex; // The index of the next values to be written. -1 to signal, that start values must be written.
} ValuesForDeltaStorage;

bool sDoPrintCaption = true; // Value used for (recursive) call to printValuesForPlotter().

uint8_t sVoltageDeltaArray[MAX_NUMBER_OF_SAMPLES]; // only used for readAndProcessEEPROMData(), but using local variable increases code size by 100 bytes
uint8_t sMilliampereDeltaArray[MAX_NUMBER_OF_SAMPLES];
uint8_t sMilliohmDeltaArray[MAX_NUMBER_OF_SAMPLES];

void getBatteryVoltageMillivolt();
bool detectAndPrintBatteryType();
void getLoggerCurrent();
void getLoggerValues();
void getBatteryCurrent();
void getBatteryValues();
bool checkStopCondition();
bool checkForBatteryRemoved();
void playEndTone();
void playAttentionTone();
void setLoad(uint8_t aNewLoadState);
void printStoredData();
void printVoltageNoLoadMillivolt();
void printBatteryValues();
void printValuesForPlotter(uint16_t aVoltageToPrint, uint16_t aMilliampereToPrint, uint16_t aMilliohmToPrint, bool aDoPrintCaption);
void printMillisValueAsFloat(uint16_t aValueInMillis);
void LCDPrintAsFloatWith2Decimals(uint16_t aValueInMillis);
void LCDPrintAsFloatWith3Decimals(uint16_t aValueInMillis);

void storeBatteryValuesToEEPROM(uint16_t aVoltageNoLoadMillivolt, uint16_t aMilliampere, uint16_t aMilliohm);
void storeCapacityAndDischargeModeToEEPROM();
void copyEEPROMDataToRam();
void readAndProcessEEPROMData(bool aDoConvertInsteadOfPrint);

void delayAndCheckForButtonPress();
void printButtonUsageMessage();
char getDischargeModeAsCharacter();
void printDischargeMode();
void printlnIfNotPlotterOutput();
void printStateString(uint8_t aState);

void switchToStateDetectingBattery();
void switchToStateInitialESRMeasurement();
void switchToStateStoreToEEPROM();
void switchToStateStopped(char aReasonCharacter);

void TogglePin(uint8_t aPinNr);
void LCDClearLine(uint8_t aLineNumber);

// Helper macro for getting a macro definition as string
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

/*
 * Program starts here
 */
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LOAD_HIGH_PIN, OUTPUT);
    pinMode(LOAD_LOW_PIN, OUTPUT);
    pinMode(ONLY_PLOTTER_OUTPUT_PIN, INPUT_PULLUP);
    pinMode(DISCHARGE_TO_LOW_PIN, INPUT_PULLUP);
    pinMode(ONLY_LOGGER_AT_A4_PIN, INPUT_PULLUP);
    setLoad(NO_LOAD);
    digitalWrite(VOLTAGE_RANGE_EXTENSION_PIN, LOW); // prepare for later use

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif

    sOnlyLoggerFunctionality = !digitalRead(ONLY_LOGGER_AT_A4_PIN);

    sOnlyPlotterOutput = !digitalRead(ONLY_PLOTTER_OUTPUT_PIN); // default behavior
    // If powered by USB, ONLY_PLOTTER_OUTPUT_PIN logic is reversed. I.e. plotter output is enabled if NOT connected to ground.
    bool tPoweredByUSB = isVCCUSBPowered();
    if (tPoweredByUSB) {
        sOnlyPlotterOutput = !sOnlyPlotterOutput; // reversed behavior if powered by USB
    }
    if (!sOnlyPlotterOutput) {
        // Just to know which program is running on my Arduino
        Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));
        /*
         * Button pin info
         */
        Serial.print(F("Button pin="));
        Serial.println(INT0_PIN);
        Serial.print(F("To suppress such prints not suited for Arduino plotter, "));
        if (tPoweredByUSB) {
            Serial.print(F("dis"));
        }
        Serial.print(F("connect pin " STR(ONLY_PLOTTER_OUTPUT_PIN) " "));
        if (tPoweredByUSB) {
            Serial.print(F("from"));
        } else {
            Serial.print(F("to"));
        }
        Serial.println(F(" ground"));

#if (NUMBER_OF_SAMPLES_PER_STORAGE * SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS != 60000)
        Serial.print(F("Sample period="));
        Serial.print(SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS);
        Serial.print(F("ms, storage period="));
        Serial.print(NUMBER_OF_SECONDS_PER_STORAGE);
        Serial.println('s');
#endif
        Serial.print(F("Maximum number of uncompressed samples="));
        Serial.print(MAX_NUMBER_OF_SAMPLES + 1); // + 1 since we always have the initial value.
        Serial.print(F(", compressed="));
        Serial.print(MAX_NUMBER_OF_SAMPLES * 2); // The initial value is nor compressed.
        Serial.print(F(" | "));
        Serial.print(((MAX_NUMBER_OF_SAMPLES * 2) * (NUMBER_OF_SECONDS_PER_STORAGE / SECONDS_IN_ONE_MINUTE)) / 60);
        Serial.print(F("h "));
        Serial.print(((MAX_NUMBER_OF_SAMPLES * 2) * (NUMBER_OF_SECONDS_PER_STORAGE / SECONDS_IN_ONE_MINUTE)) % 60);
        Serial.println(F("min"));
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
    myLCD.backlight(); // Switch backlight LED on
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

    myLCD.setCursor(0, 1);
    if (sOnlyPlotterOutput) {
        myLCD.print(F("Only plotter out"));
    } else {
        myLCD.print(F("No plotter out  "));
    }
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#if (NUMBER_OF_SAMPLES_PER_STORAGE * SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS != 60000)
    myLCD.setCursor(0, 1);
    myLCD.print(NUMBER_OF_SECONDS_PER_STORAGE);
    myLCD.print(F(" s / storage "));
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
#endif

    tone(BUZZER_PIN, 2200, 100); // usage of tone() costs 1524 bytes code space

    /*
     * Get and print EEPROM data
     * get sDischargeCutoffLevel for later appending
     */
    readAndProcessEEPROMData(false);
    printStoredData();

    printlnIfNotPlotterOutput(); // end of stored data

    /*
     * Read value to variable in order to force printing triggered by value change :-)
     */
    sLastValueOfDischargeToLowPin = digitalRead(DISCHARGE_TO_LOW_PIN);

    /*
     * If battery is still inserted, keep cutoff mode. I.e. measurement is likely to be continued.
     * If battery was removed, cutoff mode can be chosen by pressing stop button.
     */
    getBatteryVoltageMillivolt();
    if (sBatteryInfo.VoltageNoLoadMillivolt < NO_BATTERY_MILLIVOLT) {
        // Battery is removed here, so start with mode determined by pin
        sDischargeCutoffLevel = !sLastValueOfDischargeToLowPin;
        StartValues.DischargeCutoffLevel = sDischargeCutoffLevel; // Required, in order to keep mode during conversion to compressed.
    }
    printDischargeMode(); // print actual discharge mode

    if (sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff != 0) {
        char tString[6];
        if (!sOnlyPlotterOutput) {
            if (sBatteryInfo.isStandardCapacity) {
                Serial.print(F("Standard "));
            }
            Serial.print(F("capacity="));
            Serial.print(sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff);
            Serial.println(F(" mAh"));
        }
#if defined(USE_LCD)
        myLCD.setCursor(7, 1);
        if (sBatteryInfo.isStandardCapacity) {
            // replace h, l or z by s
            myLCD.print('s');
        } else if (sBatteryInfo.CapacityMilliampereHour == sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff) {
            myLCD.setCursor(8, 1);
        } else {
            // remove h, l or z, because it is NOT standard capacity and not total capacity
            myLCD.print(' ');
        }
        sprintf_P(tString, PSTR("%5u"), sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff);
        myLCD.print(tString);
        myLCD.print(F("mAh"));

#endif
    }
    switchToStateDetectingBattery();
}

/*
 * The main loop with a delay of 100 ms
 */
void loop() {

    /*
     * Check for VCC undervoltage during measurements
     */
    if (sMeasurementState != STATE_STOPPED && isVCCUndervoltageMultipleTimes()) {
#if defined(USE_LCD)
        myLCD.setCursor(0, 1);
        myLCD.print(F("VCC undervoltage"));
        myLCD.setCursor(7, 0);
        myLCD.print(F("VCC="));
        myLCD.print(getVCCVoltage(), 2);
        myLCD.print('V');
#endif
        playEndTone(); // 3 seconds
        switchToStateStopped('U');
    }

    /*
     * Handle battery detection
     */
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
                    // If found, print button usage once at start of InitialESRMeasurement
                    if (!sBatteryValuesWerePrintedAtLeastOnce) {
                        printButtonUsageMessage();
                    }
                    // set load for the first call of getBatteryValues() to measure the current
                    setLoad(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].LoadType);
                }
            } else {
                // Not inserted, so print VCC voltage initially, but not after first insertion
#if defined(USE_LCD)
                if (sMeasurementState == STATE_DETECTING_BATTERY && !sBatteryValuesWerePrintedAtLeastOnce) {
                    myLCD.setCursor(0, 0);
                    sVCCMillivolt = getVCCVoltageMillivolt();
                    myLCD.print(sVCCMillivolt / 1000.0, 2);
                    myLCD.print(F("V"));
                }
#endif
                /*
                 * if not connected to USB, check for attention every minute
                 */
                if (!isVCCUSBPowered() && millis() - sLastStateDetectingBatteryBeepMillis >= STATE_BATTERY_DETECTION_ATTENTION_PERIOD_MILLIS) {
                    sLastStateDetectingBatteryBeepMillis = millis();
                    playAttentionTone();
                }
            }
        }
        /*
         * End of battery detection handling
         */
    } else if ((unsigned) (millis() - sLastMillisOfSample)
            >= (SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS + BatteryTypeInfoArray[sBatteryInfo.TypeIndex].LoadSwitchSettleTimeMillis)) {
        /*
         * Here battery is inserted and sample period (one second) expired
         * sMeasurementState is STATE_INITIAL_ESR_MEASUREMENT or STATE_STORE_TO_EEPROM or STATE_STOPPED
         * Do all this every second (of battery load)
         */
        sLastMillisOfSample = millis();

        if (sMeasurementState == STATE_STOPPED) {
            // Get battery no load voltage and print if changed
            getBatteryVoltageMillivolt();
            if (abs(sLastVoltageNoLoadMillivoltForPrintAndCountdown - sBatteryInfo.VoltageNoLoadMillivolt) > 5) {
                printVoltageNoLoadMillivolt();
                printlnIfNotPlotterOutput();
            }
            /*
             * Check for attention every 10 minute, after the current measurement was finished
             */
            if (millis() - sLastStateStoppedBeepMillis >= STATE_STOP_ATTENTION_PERIOD_MILLIS) {
                sLastStateStoppedBeepMillis = millis();
                playAttentionTone();
            }

        } else {
            if (sOnlyLoggerFunctionality) {
                getLoggerValues();
            } else {
                getBatteryValues(); // must be called only once per sample!
            }

            if (checkForBatteryRemoved()) {
                switchToStateDetectingBattery(); // switch back to start and do not overwrite already displayed values
            } else {
                printBatteryValues();
                sBatteryValuesWerePrintedAtLeastOnce = true;
            }
        }

        /*
         * Check for end of STATE_INITIAL_ESR_MEASUREMENT
         */
        if (sMeasurementState == STATE_INITIAL_ESR_MEASUREMENT && sNumbersOfESRChecksToGo == 0) {
            if (sBatteryInfo.VoltageNoLoadMillivolt < BatteryTypeInfoArray[sBatteryInfo.TypeIndex].NominalFullVoltageMillivolt) {
                if (!sOnlyPlotterOutput) {
                    Serial.print(F("Start voltage "));
                    Serial.print(sBatteryInfo.VoltageNoLoadMillivolt);
                    Serial.print(F(" V is below NominalFullVoltageMillivolt of "));
                    Serial.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].NominalFullVoltageMillivolt);
                    Serial.println(F(" V => standard capacity can not be computed!"));
                }
#if defined(USE_LCD)
                myLCD.setCursor(0, 0);
                myLCD.print(F("Voltage too low "));
                myLCD.setCursor(0, 1);
                myLCD.print(F("for std capacity"));
#endif
#if !defined(NO_TONE_WARNING_FOR_VOLTAGE_TOO_LOW_FOR_STANDARD_CAPACITY_COMPUTATION)
                for (uint_fast8_t i = 0; i < 3; ++i) {
                    delay(700);
                    tone(BUZZER_PIN, NOTE_C7, 200);
                    delay(400);
                    tone(BUZZER_PIN, NOTE_A6, 200);
                }
#else
#  if defined(USE_LCD)
                delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#  endif
#endif
            }

            // If button was not pressed before, start a new data set
            if (sMeasurementState == STATE_INITIAL_ESR_MEASUREMENT) {
                // Force new data set
                ValuesForDeltaStorage.DeltaArrayIndex = -1;
                sBatteryInfo.CapacityAccumulator = 0;
                memset(sCurrentLoadResistorHistory, 0, sizeof(sCurrentLoadResistorHistory)); // Clear history array

                switchToStateStoreToEEPROM();
                sSampleCountForStoring = NUMBER_OF_SAMPLES_PER_STORAGE; // store first value immediately
            }
            /*
             * end of state STATE_INITIAL_ESR_MEASUREMENT
             */
        }

        /*
         * Handle storing to EEPROM
         */
        if (sMeasurementState == STATE_STORE_TO_EEPROM) {
            sSampleCountForStoring++;
            /*
             * Check for periodic storage to EEPROM
             */
            if (sSampleCountForStoring >= NUMBER_OF_SAMPLES_PER_STORAGE) {
                sSampleCountForStoring = 0;
                storeBatteryValuesToEEPROM(sBatteryInfo.VoltageNoLoadMillivolt, sBatteryInfo.Milliampere, sBatteryInfo.Milliohm);

                if (ValuesForDeltaStorage.DeltaArrayIndex == MAX_NUMBER_OF_SAMPLES && ValuesForDeltaStorage.compressionIsActive) {
                    /*
                     * Print message and stop, if compressed buffer is full
                     */
                    if (!sOnlyPlotterOutput) {
                        Serial.println(F("EEPROM delta values array full -> stop measurement"));
                    }
                    switchToStateStopped('F');

                } else if (ValuesForDeltaStorage.tempDeltaIsEmpty) {
                    /*
                     * Check for terminating condition, i.e. switch off voltage reached
                     */
                    if (checkStopCondition()) {
                        switchToStateStopped('-');
#if defined(USE_LCD)
                        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS); // show "stopped"
                        myLCD.setCursor(7, 0);
                        myLCD.print(F(" Finished"));
#endif
// Play short melody
                        playEndTone();
                    }
                }
            }
            /*
             * End of handle storing to EEPROM
             */
        }
    } // end of handling each second

    delay(100);

    /*
     * Check for plotter mode pin change
     */
    sOnlyPlotterOutput = !digitalRead(ONLY_PLOTTER_OUTPUT_PIN);
    if (isVCCUSBPowered()) {
        sOnlyPlotterOutput = !sOnlyPlotterOutput; // reversed behavior if powered by USB
    }

    /*
     * Check for discharge mode pin change
     */
    bool tValueOfDischargeToLowPin = digitalRead(DISCHARGE_TO_LOW_PIN);
    if (sLastValueOfDischargeToLowPin != tValueOfDischargeToLowPin) {
        sLastValueOfDischargeToLowPin = tValueOfDischargeToLowPin;
        sDischargeCutoffLevel = !sLastValueOfDischargeToLowPin;
        // Update EEPROM value for right start value for next appending
        if (StartValues.DischargeCutoffLevel != sDischargeCutoffLevel) {
            StartValues.DischargeCutoffLevel = sDischargeCutoffLevel;
            eeprom_update_byte(&EEPROMStartValues.DischargeCutoffLevel, sDischargeCutoffLevel);
        }
        printDischargeMode();
    }
}

void printStateString(uint8_t aState) {
    if (!sOnlyPlotterOutput) {
        if (aState == STATE_DETECTING_BATTERY) {
            Serial.print(F("DETECTING BATTERY"));
        } else if (aState == STATE_INITIAL_ESR_MEASUREMENT) {
            Serial.print(F("INITIAL ESR MEASUREMENT"));
        } else if (aState == STATE_STORE_TO_EEPROM) {
            Serial.print(F("STORE TO EEPROM"));
        } else if (aState == STATE_STOPPED) {
            Serial.print(F("STOPPED"));
        }
    }
}

void printlnIfNotPlotterOutput() {
    if (!sOnlyPlotterOutput) {
        Serial.println();
    }
}

void printSwitchStateString() {
    if (!sOnlyPlotterOutput) {
        Serial.print(F("Switch to state "));
        printStateString(sMeasurementState);
    }
}

void switchToStateDetectingBattery() {
    sMeasurementState = STATE_DETECTING_BATTERY;
    printSwitchStateString();
    printlnIfNotPlotterOutput();
    sLastStateDetectingBatteryBeepMillis = millis();
    sLastVoltageNoLoadMillivoltForBatteryCheck = 0XFFFF; // to force first check if voltage is 0
}

void switchToStateInitialESRMeasurement() {
    sMeasurementState = STATE_INITIAL_ESR_MEASUREMENT;
    printSwitchStateString();
    printlnIfNotPlotterOutput();
    sNumbersOfESRChecksToGo = NUMBER_OF_INITIAL_ESR_SAMPLES;
    memset(sESRHistory, 0, sizeof(sESRHistory));
}

void switchToStateStoreToEEPROM() {
    sMeasurementState = STATE_STORE_TO_EEPROM;
    printSwitchStateString();
    printlnIfNotPlotterOutput();
    // Required for right appending to EEPROM, so do it here
    ValuesForDeltaStorage.tempDeltaIsEmpty = true;
    eeprom_update_byte(&EEPROMStartValues.DischargeCutoffLevel, sDischargeCutoffLevel);
}

/*
 * aWriteToLCD default is true.
 * @param aReasonCharacter, '-' for terminating condition met (regular end of measurement), U for VCC undervoltage, F for EEPROM full,
 *                           D for button double press, B for button press.
 */
void switchToStateStopped(char aReasonCharacter) {
    if (sMeasurementState != STATE_STOPPED) {
        setLoad(NO_LOAD);
        auto tOldMeasurementState = sMeasurementState;
        sMeasurementState = STATE_STOPPED;

        if (!sOnlyPlotterOutput) {
            printSwitchStateString();
            Serial.print(F(", reason="));
            Serial.println(aReasonCharacter);
        }

#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        myLCD.print(F("Stop measurement"));
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
        LCDClearLine(0);
#endif
        if (tOldMeasurementState == STATE_STORE_TO_EEPROM) {
            storeCapacityAndDischargeModeToEEPROM(); // Store capacity and discharge mode
        }

#if defined(USE_LCD)
        myLCD.setCursor(7, 0);
        myLCD.print(F("Stopped "));
        myLCD.print(aReasonCharacter);
#endif
        sLastVoltageNoLoadMillivoltForPrintAndCountdown = 0; // to force display of NoLoad voltage
    }
}

char getDischargeModeAsCharacter() {
    if (sDischargeCutoffLevel == DISCHARGE_CUTOFF_LEVEL_ZERO) {
        return 'z';
    } else if (sDischargeCutoffLevel == DISCHARGE_CUTOFF_LEVEL_LOW) {
        return 'l';
    } else { // DISCHARGE_CUTOFF_LEVEL_NORMAL
        return 'h';
    }
}

void LCDPrintDischargeMode() {
#if defined(USE_LCD)
    auto tDischargeCutoffLevel = sDischargeCutoffLevel;
    myLCD.setCursor(0, 0);
    if (tDischargeCutoffLevel == DISCHARGE_CUTOFF_LEVEL_ZERO) {
        myLCD.print(F("Cut off is 50 mV"));
    } else {
        if (sBatteryInfo.TypeIndex == TYPE_INDEX_NO_BATTERY) {
            // Long text without voltage
            myLCD.print(F("Cut off is "));
            if (tDischargeCutoffLevel == DISCHARGE_CUTOFF_LEVEL_LOW) {
                myLCD.print(F("low  "));
            } else { // DISCHARGE_CUTOFF_LEVEL_NORMAL
                myLCD.print(F("high "));
            }
        } else {
            // Short text with voltage e.g. "Cutoff high 3.5V"
            myLCD.print(F("Cutoff "));
            uint16_t tSwitchOffVoltageMillivolt;
            if (tDischargeCutoffLevel == DISCHARGE_CUTOFF_LEVEL_LOW) {
                myLCD.print(F("low "));
                tSwitchOffVoltageMillivolt = BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivoltLow;
            } else { // DISCHARGE_CUTOFF_LEVEL_NORMAL
                myLCD.print(F("high"));
                tSwitchOffVoltageMillivolt = BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivoltHigh;
            }
            if (tSwitchOffVoltageMillivolt < 10000) {
                myLCD.print(' ');
            }
            myLCD.print(((float) tSwitchOffVoltageMillivolt) / 1000, 1);
            myLCD.print('V');
        }
    }
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
}

/*
 * Prints state of discharge mode
 */
void printDischargeMode() {
    auto tDischargeCutoffLevel = sDischargeCutoffLevel;
    if (!sOnlyPlotterOutput) {
        if (tDischargeCutoffLevel == DISCHARGE_CUTOFF_LEVEL_ZERO) {
            Serial.println(F("Discharge to 50 mV"));
        } else {
            if (sBatteryInfo.TypeIndex == TYPE_INDEX_NO_BATTERY) {
                if (tDischargeCutoffLevel == DISCHARGE_CUTOFF_LEVEL_LOW) {
                    Serial.println(F("Discharge to low voltage. e.g. 3000 mV for Li-ion"));
                } else { // DISCHARGE_CUTOFF_LEVEL_NORMAL
                    Serial.println(F("Discharge to high voltage. e.g. 3450 mV for Li-ion"));
                }
            } else {
                if (tDischargeCutoffLevel == DISCHARGE_CUTOFF_LEVEL_LOW) {
                    Serial.print(F("Discharge to low voltage "));
                    Serial.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivoltLow);
                } else { // DISCHARGE_CUTOFF_LEVEL_NORMAL
                    Serial.print(F("Discharge to high voltage "));
                    Serial.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivoltHigh);
                }
                Serial.println(F(" mV"));
            }
        }
    }
    LCDPrintDischargeMode();
}

/*
 * Delay for LCD_MESSAGE_PERSIST_TIME_MILLIS but terminate if state was changed by button press
 */
void delayAndCheckForButtonPress() {
    uint8_t tOldMeasurementState = sMeasurementState;
    for (uint_fast8_t i = 0; i < 10; ++i) {
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS / 10);
        if (sMeasurementState != tOldMeasurementState) {
            // Button press changes state here
            break;
        }
    }
}

/*
 * Print message "dbl press = stop" and "Press button to append to EEPROM"
 * Wait if no button was pressed
 */
void printButtonUsageMessage() {
#if defined(USE_LCD)
    uint8_t tOldMeasurementState = sMeasurementState;
#endif

    if (!sOnlyPlotterOutput) {
        Serial.println(F("Double press \"Start/stop\" button to stop measurement"));
    }
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    myLCD.print(F("dbl press = stop"));
    /*
     * and wait for 2 seconds for button press
     */
    delayAndCheckForButtonPress();

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
        delayAndCheckForButtonPress();
    }
#endif
}

void checkForDelayedButtorProcessing() {
// sInLCDPrint is false here, only if button handler was called after setting it to true
    if (!sInLCDPrint) {
        handleStartStopButtonPress(false); // delayed call to button handler
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Call delayed button processing"));
        }

    } else {
        sInLCDPrint = false; // Enable printing by button handler
    }
}

/*
 * Check for removed battery
 * @return true if battery removed
 */
bool checkForBatteryRemoved() {

// check only if battery was inserted before
    if (sBatteryWasInserted && sBatteryInfo.VoltageNoLoadMillivolt < NO_BATTERY_MILLIVOLT) {
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Battery removing detected"));
        }
        sBatteryWasInserted = false;
        return true;
    }
    return false;
}

/*
 * !!!Called in ISR Context!!!
 * !!! We can be called recursively, i.e. while waiting for 2 seconds we can be called for double press !!!
 * Because interrupts are still enabled, millis() is working here :-)
 *
 * Ignore all presses in mode STATE_SETUP_AND_READ_EEPROM and STATE_DETECTING_BATTERY
 * Double click in 2 seconds stop measurement. -> Goes to state STATE_STOPPED
 *
 * Single press:
 * In mode STATE_DETECTING_BATTERY:
 *      Cycle discharge mode / cutoff level
 * In mode STATE_STOPPED:
 *      Switch to state STATE_DETECTING_BATTERY
 * In mode STATE_INITIAL_ESR_MEASUREMENT:
 *      Appends data to already stored ones in EEPROM -> switch to state STATE_STORE_TO_EEPROM
 * In mode STATE_STORE_TO_EEPROM:
 *      Stores current capacity and stops measurement -> switch to state STATE_STOPPED
 */
void handleStartStopButtonPress(bool aButtonToggleState) {
    (void) aButtonToggleState;

    if (sMeasurementState == STATE_SETUP_AND_READ_EEPROM) {
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Early press ignored"));
        }
        return;
    }

// Do not process button  as long as LCD is in use
    if (sInLCDPrint) {
        sInLCDPrint = false; // this forces a new call to handleStartStopButtonPress() at the end of PrintBatteryValues()
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Press detected while LCD in use"));
        }
        return;
    }

    bool tIsDoublePress = startStopButton0AtPin2.checkForDoublePress( EASY_BUTTON_DOUBLE_PRESS_DEFAULT_MILLIS);
    if (!sOnlyPlotterOutput) {
// Print should be done after checkForDoublePress() in order to not disturb the double press detection
        Serial.print(F("Button pressed, state="));
        printStateString(sMeasurementState);
        Serial.println();
    }

    if (tIsDoublePress && sMeasurementState != STATE_DETECTING_BATTERY) {
        /*
         * Double press detected!
         * Go to STATE_STOPPED
         */
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Double press detected"));
        }
        switchToStateStopped('D');

    } else {
        /*
         * Single press here
         * Attention, this press can be the first press of a double press,
         * so we must wait 2 seconds and check for double press before processing single press
         */

        if (sMeasurementState == STATE_DETECTING_BATTERY) {
            // Toggle discharge to normal, to low and to zero
            sDischargeCutoffLevel++;
            if (sDischargeCutoffLevel > DISCHARGE_CUTOFF_LEVEL_ZERO) {
                sDischargeCutoffLevel = DISCHARGE_CUTOFF_LEVEL_NORMAL;
            }
            printDischargeMode();

        } else if (sMeasurementState == STATE_STORE_TO_EEPROM) {
            switchToStateStopped('B'); // no check for double press required here :-)

        } else {

            /*
             * Only state STATE_INITIAL_ESR_MEASUREMENT and STATE_STOPPED left here!
             * Print start message, and wait for 2 seconds for double press detection
             */
            uint8_t tOldMeasurementState = sMeasurementState;
#if defined(USE_LCD)
            myLCD.setCursor(0, 0);
#endif
            if (tOldMeasurementState == STATE_STOPPED) {
                if (!sOnlyPlotterOutput) {
                    Serial.println(F("Start again"));
                }
#if defined(USE_LCD)
                myLCD.print(F("Start again     "));
#endif
            } else {
                // STATE_INITIAL_ESR_MEASUREMENT here
                if (!sOnlyPlotterOutput) {
                    Serial.println(F("Append data to EEPROM"));
                }
#if defined(USE_LCD)
                myLCD.print(F("Append to EEPROM"));
#endif
            }

            /*
             * wait for double press detection, which means sMeasurementState can now also have value STATE_STOPPED here
             */
            delayAndCheckForButtonPress();

// Must check old value (before possible double press) in order to avoid switching from STATE_STOPPED to DetectingBattery at each double press.
            if (tOldMeasurementState == STATE_STOPPED) {
                // start a new measurement cycle
                switchToStateDetectingBattery();

            } else if (sMeasurementState == STATE_INITIAL_ESR_MEASUREMENT) {
                // No stop requested during 2 seconds wait -> append to EEPROM
                switchToStateStoreToEEPROM();
                /*
                 * Store next sample in 60 seconds, because we assume double press directly after entering state STATE_INITIAL_ESR_MEASUREMENT
                 * Otherwise we would start the appended data with a short sampling period.
                 */
                sSampleCountForStoring = 0;
            }
        }
#if defined(DEBUG)
        if (!sOnlyPlotterOutput) {
            Serial.print(F("New state="));
            printStateString(sMeasurementState);
            Serial.println();
        }
#endif
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
            digitalWrite(LOAD_LOW_PIN, LOW);    // disable 12 ohm load
            digitalWrite(LOAD_HIGH_PIN, LOW);    // disable 3 ohm load
        } else if (aNewLoadState == LOW_LOAD) {
#if defined(DEBUG)
            Serial.println(F("low"));
#endif
            digitalWrite(LOAD_LOW_PIN, HIGH);    // enable 12 ohm load
            digitalWrite(LOAD_HIGH_PIN, LOW);    // disable 3 ohm load
        } else {
#if defined(DEBUG)
            Serial.println(F("high"));
#endif
            digitalWrite(LOAD_LOW_PIN, LOW);    // disable 12 ohm load
            digitalWrite(LOAD_HIGH_PIN, HIGH);    // enable 3 ohm load
        }

    }
}

/*
 * Sets VoltageNoLoadMillivolt or VoltageLoadMillivolt
 * Provides automatic range switch between 2.2, 4.4 and 14 (up to 20 with 5V VCC) volt range
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
        pinMode(VOLTAGE_RANGE_EXTENSION_PIN, OUTPUT);
        digitalWrite(VOLTAGE_RANGE_EXTENSION_PIN, LOW);    // required???
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
            pinMode(VOLTAGE_RANGE_EXTENSION_PIN, INPUT);
            digitalWrite(VOLTAGE_RANGE_EXTENSION_PIN, LOW);
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
                Serial.print(sVCCMillivolt / 1000.0, 3);
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
void getLoggerCurrent() {
    uint16_t tShuntVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_LOGGER_CURRENT, INTERNAL);
    sBatteryInfo.Milliampere = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) * tShuntVoltageRaw)
            / (1023L * SHUNT_AT_A4_RESISTOR_MILLIOHM));
}

void getLoggerValues() {
    getLoggerCurrent();
    getBatteryVoltageMillivolt();    // get current battery load voltage
    if (sBatteryInfo.Milliampere > 1) {
        // New capacity computation
        sBatteryInfo.CapacityAccumulator += sBatteryInfo.Milliampere;
        sBatteryInfo.CapacityMilliampereHour = sBatteryInfo.CapacityAccumulator
                / ((3600L * MILLIS_IN_ONE_SECOND) / SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS); // = / 3600 for 1 s sample period
    }
    sBatteryInfo.Milliohm = 0;
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
// Do it before deactivating the load
    getBatteryCurrent();
    getBatteryVoltageMillivolt();    // get current battery load voltage

// Deactivate load and wait for voltage to settle
// During the no load period switch on the LED
    setLoad(NO_LOAD);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].LoadSwitchSettleTimeMillis);
    getBatteryVoltageMillivolt();    // get current battery no load voltage
// restore original load state
    setLoad(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].LoadType);
    digitalWrite(LED_BUILTIN, LOW);

    sBatteryInfo.sESRDeltaMillivolt = sBatteryInfo.VoltageNoLoadMillivolt - sBatteryInfo.VoltageLoadMillivolt;

    if (sBatteryInfo.Milliampere > 1) {
        // New capacity computation
        sBatteryInfo.CapacityAccumulator += sBatteryInfo.Milliampere;
        sBatteryInfo.CapacityMilliampereHour = sBatteryInfo.CapacityAccumulator
                / ((3600L * MILLIS_IN_ONE_SECOND) / SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS);    // = / 3600 for 1 s sample period

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
        // insert current value
        uint32_t tESRMilliohm = (sBatteryInfo.sESRDeltaMillivolt * 1000L) / sBatteryInfo.Milliampere;
        if (tESRMilliohm > __UINT16_MAX__) {
            sESRHistory[0] = __UINT16_MAX__; // indicate overflow
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
void playEndTone() {
    tone(BUZZER_PIN, NOTE_A5);
    delay(1000);
    tone(BUZZER_PIN, NOTE_E5);
    delay(1000);
    tone(BUZZER_PIN, NOTE_A4, 1000);
    delay(1000);
}

void playAttentionTone() {
    tone(BUZZER_PIN, NOTE_C7, 40);
    delay(100);
    tone(BUZZER_PIN, NOTE_C7, 40);
    delay(100);
}

/*
 * Check for switch off voltage reached -> end of measurement
 * @return true if stop condition met
 */
bool checkStopCondition() {
    uint16_t tSwitchOffVoltageMillivolt;
    if (sDischargeCutoffLevel == DISCHARGE_CUTOFF_LEVEL_ZERO) {
        tSwitchOffVoltageMillivolt = DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT;
    } else if (sDischargeCutoffLevel == DISCHARGE_CUTOFF_LEVEL_LOW) {
        tSwitchOffVoltageMillivolt = BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivoltLow;
    } else { // DISCHARGE_CUTOFF_LEVEL_NORMAL
        tSwitchOffVoltageMillivolt = BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivoltHigh;
    }
    if (sBatteryInfo.VoltageNoLoadMillivolt < tSwitchOffVoltageMillivolt) {
        /*
         * Switch off condition met
         */
        setLoad(NO_LOAD);
        if (!sOnlyPlotterOutput) {
            Serial.print(F("Switch off voltage "));
            Serial.print(tSwitchOffVoltageMillivolt);
            Serial.print(F(" mV reached, capacity="));
            Serial.print(sBatteryInfo.CapacityMilliampereHour);
            Serial.println(F(" mAh"));
        }

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
            Serial.print(F(" Battery index="));
            Serial.print(i);
            Serial.print(F(" BatteryVoltageMillivolt="));
            Serial.print(aBatteryVoltageMillivolt);
            Serial.print(F(" SwitchOffVoltageMillivolt="));
            Serial.println(BatteryTypeInfoArray[i].SwitchOffVoltageMillivoltHigh);
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

    if (sBatteryInfo.TypeIndex == TYPE_INDEX_NO_BATTERY) {
#if defined(USE_LCD)
        myLCD.setCursor(5, 0);
        myLCD.print(F("  "));
        myLCD.print(F(" No batt."));
#endif
        return false;

    } else {
        // print values
        printVoltageNoLoadMillivolt();
        if (!sOnlyPlotterOutput) {
            Serial.print(F(" => "));
            Serial.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].TypeName);
            Serial.println(F(" found"));
        }

#if defined(USE_LCD)
        myLCD.setCursor(0, 1);
        myLCD.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].TypeName);
        myLCD.print(F(" found"));
// The current battery voltage is displayed, so clear "No batt." message selectively
        myLCD.setCursor(7, 0);
        myLCD.print(F("         "));
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
        LCDClearLine(1);
#endif
        sBatteryWasInserted = true;
        return true;
    }
}

/*
 * Called exclusively from setup() after readAndProcessEEPROMData()
 */
void printStoredData() {
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    myLCD.print(getVCCVoltage(), 1);
    myLCD.print(F("V Stored data"));
#endif
    /*
     * Print battery values, and use state STATE_SETUP_AND_READ_EEPROM for formatting
     * "0.061o l 1200mAh" using sBatteryInfo.Milliohm
     */
    printBatteryValues();
#if defined(USE_LCD)
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
}

void printVoltageNoLoadMillivolt() {
    uint16_t tVoltageNoLoadMillivolt = sBatteryInfo.VoltageNoLoadMillivolt; // saves 12 bytes programming space
    if (!sOnlyPlotterOutput) {
        sLastVoltageNoLoadMillivoltForPrintAndCountdown = tVoltageNoLoadMillivolt;
        printMillisValueAsFloat(tVoltageNoLoadMillivolt);
        Serial.print(F(" V"));
    }
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    LCDPrintAsFloatWith3Decimals(tVoltageNoLoadMillivolt);
    myLCD.print(F("V "));
// cursor is now at 7, 0
#endif

}

/*
 * Evaluates sMeasurementState and prints:
 *   - sBatteryInfo.VoltageNoLoadMillivolt
 *   - sBatteryInfo.Milliampere
 *   - sBatteryInfo.Milliohm
 *   - optional sESRDeltaMillivolt or capacity
 * to Serial and LCD
 *
 * STATE_INITIAL_ESR_MEASUREMENT:
 * "4.030 V, 27 s  329 mA at 11.896 ohm, ESR=0.329 ohm, 0.108 V"
 *  0   4   8   C  F
 * "4.030V 18  329mA" printing down counter
 * "0.061o l  0.128V" using current ESR from sESRHistory[0]
 *
 * STATE_STORE_TO_EEPROM:
 * "4.030 V,  329 mA at 11.949 ohm, ESR=0.329 ohm, capacity=1200 mAh
 *  0   4   8   C  F
 * "4.030V 312 329mA" printing EEPROM array index
 * "0.392o l 1200mAh" using sBatteryInfo.Milliohm
 *
 * Called with states STATE_SETUP_AND_READ_EEPROM, STATE_INITIAL_ESR_MEASUREMENT and STATE_STORE_TO_EEPROM
 */
void printBatteryValues() {
    char tString[6];

    sInLCDPrint = true; // disable printing by button handler
    uint8_t tMeasurementState = sMeasurementState; // Because sMeasurementState is volatile
    if (tMeasurementState != STATE_SETUP_AND_READ_EEPROM) {
        /***********************************************************************************
         * First row only for state STATE_INITIAL_ESR_MEASUREMENT or STATE_STORE_TO_EEPROM
         ***********************************************************************************/
        printVoltageNoLoadMillivolt();
        if (!sOnlyPlotterOutput) {
            // 4.094 V,  334 mA at 11.949 ohm, ESR=0.334 ohm, capacity=3501 mAh
            Serial.print(F(", "));
        }

        if (tMeasurementState == STATE_INITIAL_ESR_MEASUREMENT) {
            /*
             * Print down counter for STATE_INITIAL_ESR_MEASUREMENT
             * Count down only if we do not have a rapid voltage decrease
             */
            if ((sLastVoltageNoLoadMillivoltForPrintAndCountdown - sBatteryInfo.VoltageNoLoadMillivolt) < 3) {
                sNumbersOfESRChecksToGo--;
            }
            sLastVoltageNoLoadMillivoltForPrintAndCountdown = sBatteryInfo.VoltageNoLoadMillivolt;

            uint8_t tNumbersOfESRChecksToGo = sNumbersOfESRChecksToGo;
            if (!sOnlyPlotterOutput) {
                Serial.print(tNumbersOfESRChecksToGo);
                Serial.print(F(" s, ")); // seconds until discharging
            }
            if (tNumbersOfESRChecksToGo <= 10) {
#if defined(USE_LCD)
                myLCD.print(' '); // padding space for count
#endif
                tone(BUZZER_PIN, 2000, 40); // costs 1524 bytes code space
            }
#if defined(USE_LCD)
            myLCD.print(tNumbersOfESRChecksToGo);
            myLCD.print(' '); // trailing space for count (just in case mA are > 999)
#endif

        } else {
            /*
             * Print index counter for STATE_STORE_TO_EEPROM
             */
#if defined(USE_LCD)
            int tDeltaArrayIndex = ValuesForDeltaStorage.DeltaArrayIndex; // saves 8 bytes of program space
            if (ValuesForDeltaStorage.compressionIsActive) {
                tDeltaArrayIndex *= 2;
            }
// We start with array index -1, which indicates initialization of array :-)
            if (tDeltaArrayIndex < 10 && tDeltaArrayIndex >= 0) {
                myLCD.print(' '); // we have "-1" once, because we store values (and increment index) after print
            }
            if (tDeltaArrayIndex < 100) {
                myLCD.print(' '); // padding space :-)
            }
            myLCD.print(tDeltaArrayIndex);

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
            printMillisValueAsFloat(sCurrentLoadResistorHistory[0]);
            Serial.print(F(" ohm, "));
        }
#if defined(USE_LCD)
        myLCD.print(tString);
        myLCD.print(F("mA"));
#endif

    }

    /**********************
     * Start of second row
     **********************/
    /*
     * STATE_SETUP_AND_READ_EEPROM + STATE_STORE_TO_EEPROM: "0.061o l 1200mAh" using sBatteryInfo.Milliohm
     * STATE_INITIAL_ESR_MEASUREMENT:                       "0.061o l  0.128V" using current ESR from sESRHistory[0]
     */
//    bool tShowCapacity = (tMeasurementState == STATE_STORE_TO_EEPROM || tMeasurementState == STATE_SETUP_AND_READ_EEPROM);
    bool tShowCapacity = (tMeasurementState != STATE_INITIAL_ESR_MEASUREMENT);

    if (!sOnlyLoggerFunctionality) {
        uint32_t tMilliohm;
        if (tShowCapacity) {
            tMilliohm = sBatteryInfo.Milliohm;
        } else {
            tMilliohm = sESRHistory[0];
        }

        if (!sOnlyPlotterOutput) {
            Serial.print(F("ESR="));
            if (tMilliohm == __UINT16_MAX__ || sBatteryInfo.Milliampere == 0) {
                Serial.print(F("overflow, "));
            } else {
                printMillisValueAsFloat(tMilliohm);
                Serial.print(F(" ohm, "));
            }
        }
#if defined(USE_LCD)
        myLCD.setCursor(0, 1);
        if (tMilliohm == __UINT16_MAX__ || sBatteryInfo.Milliampere == 0) {
            myLCD.print(F("99.99")); // Overflow
        } else if (tMilliohm < 10000) {
            myLCD.print(((float) (tMilliohm)) / 1000, 3);
        } else {
            myLCD.print(((float) (tMilliohm)) / 1000, 2);
        }
        myLCD.print(F("\xF4 ")); // Ohm symbol

        myLCD.setCursor(7, 1); // This avoids problems with values >= 10 ohm
        myLCD.print(getDischargeModeAsCharacter());
#endif
    }

    /*
     * Print voltage difference or capacity
     */
    if (tShowCapacity) {
        /*
         * Print capacity
         */
        if (!sOnlyPlotterOutput) {
            Serial.print(F("capacity="));
            Serial.print(sBatteryInfo.CapacityMilliampereHour);
            Serial.print(F(" mAh"));
        }
#if defined(USE_LCD)
        sprintf_P(tString, PSTR("%5u"), sBatteryInfo.CapacityMilliampereHour);
        myLCD.print(tString);
        myLCD.print(F("mAh"));
#endif

    } else {
        if (!sOnlyLoggerFunctionality) {
            /*
             * Print voltage difference at load
             */
            uint16_t tESRDeltaMillivolt = sBatteryInfo.sESRDeltaMillivolt; // saves 4 bytes programming space
            if (!sOnlyPlotterOutput) {
                printMillisValueAsFloat(tESRDeltaMillivolt);
                Serial.print(F(" V "));
            }
#if defined(USE_LCD)
            myLCD.print(F("  ")); // leading spaces only for voltage
            LCDPrintAsFloatWith3Decimals(tESRDeltaMillivolt);
            myLCD.print(F("V"));
#endif
        }
    }

    printlnIfNotPlotterOutput();

    checkForDelayedButtorProcessing();
}

void printMillisValueAsFloat(uint16_t aValueInMillis) {
    Serial.print(((float) (aValueInMillis)) / 1000, 3);
}

void LCDPrintAsFloatWith3Decimals(uint16_t aValueInMillis) {
#if defined(USE_LCD)
    myLCD.print(((float) (aValueInMillis)) / 1000, 3);
#else
    (void) aValueInMillis;
#endif
}

void LCDPrintAsFloatWith2Decimals(uint16_t aValueInMillis) {
#if defined(USE_LCD)
    myLCD.print(((float) (aValueInMillis)) / 1000, 2);
#else
    (void) aValueInMillis;
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
 * Returns the real uncompressed delta
 *  7 / F is interpreted as 16 enabling values of 10 (16 -6) to 21 (16 +5) in 2 steps
 *  6 / E is interpreted as 5
 *  5 / D is interpreted as 4
 *  4 / C is interpreted as 3
 * -4 / 4 is interpreted as -5
 * -5 / 3 is interpreted as -6 enabling values of -12 in 2 steps
 * -6 / 2 is interpreted as -18 enabling values of -13 (-18 +5) to -24 (-18 -6) and -36 in 2 steps
 * -7 / 1 is interpreted as -30 enabling values of -25 (-30 +5) to -36 (-30 -6) in 2 steps
 * -8 / 0 is interpreted as -42 enabling values of -37 (-42 +5) to -48 (-42 -6) in 2 steps
 */
int8_t getDelta(uint8_t a4BitDelta) {
    int8_t tDelta;
    if (a4BitDelta == 15) {
        tDelta = 16;
    } else if (a4BitDelta == 2) {
        tDelta = -18;
    } else if (a4BitDelta == 1) {
        tDelta = -30;
    } else if (a4BitDelta == 0) {
        tDelta = -42;
    } else {
// Here values from 3 to 14 converted to -6 to 5
        tDelta = a4BitDelta - 9;
    }
    return tDelta;
}

/*
 * compressed delta
 * upper 4 bit store the first value (between -8 and 7), lower 4 bit store the second value
 * @param aDelta        The delta to process
 * @param *aDeltaTemp   Storage of the upper 4 bit delta, which cannot directly be written to EEPROM
 * @return clipped aDelta | aDelta which is stored
 */
int16_t storeDeltas(int16_t aDelta, uint8_t *aDeltaTemp, uint8_t *aEEPROMAddressToStoreValue) {
    if (!sOnlyPlotterOutput) {
        Serial.print(' ');
        Serial.print(aDelta);
    }

    if (!ValuesForDeltaStorage.compressionIsActive) {
// No compression, only clip to 8 bit range
        if (aDelta > __INT8_MAX__) {
            aDelta = __INT8_MAX__;
        } else if (aDelta < -128) {
            aDelta = -128;
        }
        int8_t tDelta = aDelta;
        eeprom_write_byte(aEEPROMAddressToStoreValue, tDelta);
        return aDelta;
    }

    /*
     * Compression here. Clip aDelta to the available range
     */
    int8_t tDelta;
    if (aDelta >= 10) {
        aDelta = 16; // The interpreted value == getDelta(tDelta)
        tDelta = 7;  // The coded value
    } else if (aDelta >= 5) {
        aDelta = 5; // The interpreted value == getDelta(tDelta)
        tDelta = 6; // The coded value

    } else if (aDelta <= -37) {
        aDelta = -42; // The interpreted value == getDelta(tDelta)
        tDelta = -8;  // The coded value
    } else if (aDelta <= -25) {
        aDelta = -30; // The interpreted value == getDelta(tDelta)
        tDelta = -7;  // The coded value
    } else if (aDelta <= -13) {
        aDelta = -18;
        tDelta = -6; // -> 1
    } else if (aDelta <= -6) {
        aDelta = -6; // The interpreted value == getDelta(tDelta)
        tDelta = -5; // The coded value
    } else {
// Here values from -5 to 4
        tDelta = aDelta + 1;
    }
    /*
     * convert delta to an unsigned value by adding 8 => -8 to 7 -> 0 to F
     * F is +7, 8 is 0, 0 is -8 tDelta is positive now :-)
     */
    tDelta += 8;

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
            tone(BUZZER_PIN, NOTE_C7, 20);
            delay(40);
            tone(BUZZER_PIN, NOTE_C6, 20);
            delay(40);
            tone(BUZZER_PIN, NOTE_C7, 20);
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
void storeBatteryValuesToEEPROM(uint16_t aVoltageNoLoadMillivolt, uint16_t aMilliampere, uint16_t aMilliohm) {
    if (ValuesForDeltaStorage.DeltaArrayIndex < 0) {
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Store initial values to EEPROM"));
        }

        clearEEPROMTo_FF(); // this may last one or two seconds

        /*
         * Initial values
         * Storing them in a local structure and storing this, costs 50 bytes code size
         * Storing them in a global structure and storing this, costs 30 bytes code size
         *
         * Initially set up structure for start values, also used for printing
         * and store it to EEPROM
         */
        StartValues.initialDischargingMillivolt = aVoltageNoLoadMillivolt;
        StartValues.initialDischargingMilliampere = aMilliampere;
        StartValues.initialDischargingMilliohm = aMilliohm;
        StartValues.compressionFlag = FLAG_NO_COMPRESSION;
        StartValues.DischargeCutoffLevel = sDischargeCutoffLevel;
        StartValues.LoadResistorMilliohm = sCurrentLoadResistorAverage;
        StartValues.CapacityMilliampereHour = 0; // Capacity is written at the end or computed while reading
        eeprom_write_block(&StartValues, &EEPROMStartValues, sizeof(EEPROMStartValues));

        /*
         * Initially set up structure for delta storage
         */
        ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt = aVoltageNoLoadMillivolt;
        ValuesForDeltaStorage.lastStoredMilliampere = aMilliampere;
        ValuesForDeltaStorage.lastStoredMilliohm = aMilliohm;
        ValuesForDeltaStorage.tempDeltaIsEmpty = true;
        ValuesForDeltaStorage.compressionIsActive = false;
        ValuesForDeltaStorage.DeltaArrayIndex = 0;

    } else {
        if (!ValuesForDeltaStorage.compressionIsActive) {
            /*
             * No compression -> use 8 bit deltas
             */
            if ((unsigned int) ValuesForDeltaStorage.DeltaArrayIndex < MAX_NUMBER_OF_SAMPLES) {

                if (!sOnlyPlotterOutput) {
                    Serial.print(F("Store 8 bit deltas:"));
                }
                /*
                 * Append value to delta values array
                 */
                int16_t tVoltageDelta = aVoltageNoLoadMillivolt - ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt;
                tVoltageDelta = storeDeltas(tVoltageDelta, &ValuesForDeltaStorage.tempVoltageDelta,
                        reinterpret_cast<uint8_t*>(&sMillivoltDeltaArrayEEPROM[ValuesForDeltaStorage.DeltaArrayIndex]));
                ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt += tVoltageDelta;

                int16_t tMilliampereDelta = aMilliampere - ValuesForDeltaStorage.lastStoredMilliampere;
                tMilliampereDelta = storeDeltas(tMilliampereDelta, &ValuesForDeltaStorage.tempMilliampereDelta,
                        reinterpret_cast<uint8_t*>(&sMilliampereDeltaArrayEEPROM[ValuesForDeltaStorage.DeltaArrayIndex]));
                ValuesForDeltaStorage.lastStoredMilliampere += tMilliampereDelta;

                int16_t tMilliohmDelta = aMilliohm - ValuesForDeltaStorage.lastStoredMilliohm;
                tMilliohmDelta = storeDeltas(tMilliohmDelta, &ValuesForDeltaStorage.tempMilliohmDelta,
                        reinterpret_cast<uint8_t*>(&sMilliohmDeltaArrayEEPROM[ValuesForDeltaStorage.DeltaArrayIndex]));
                ValuesForDeltaStorage.lastStoredMilliohm += tMilliohmDelta;

                printlnIfNotPlotterOutput();

                ValuesForDeltaStorage.DeltaArrayIndex++; // increase every sample
            } else {
                /*
                 * CONVERT
                 */
                if (!sOnlyPlotterOutput) {
                    Serial.println();
                    Serial.print(F("Convert "));
                    Serial.print(MAX_NUMBER_OF_SAMPLES);
                    Serial.println(F(" uncompressed stored values to compressed ones"));
                }

                // Start a new compressed storage
                ValuesForDeltaStorage.DeltaArrayIndex = 0;
                ValuesForDeltaStorage.compressionIsActive = true;

                /*
                 * Read all data and process them. This recursively calls storeBatteryValuesToEEPROM(), but we end up below in code for compressed format
                 */
                sDoPrintCaption = false; // for recursive call to printValuesForPlotter() below
                readAndProcessEEPROMData(true); // true -> convert data instead of printing them
                sDoPrintCaption = true;

                /*
                 * Clear rest of uncompressed EEPROM values
                 */
                for (unsigned int i = ValuesForDeltaStorage.DeltaArrayIndex; i < MAX_NUMBER_OF_SAMPLES; ++i) {
                    eeprom_update_byte(reinterpret_cast<uint8_t*>(&sMillivoltDeltaArrayEEPROM[i]), 0xFF);
                    eeprom_update_byte(reinterpret_cast<uint8_t*>(&sMilliampereDeltaArrayEEPROM[i]), 0xFF);
                    eeprom_update_byte(reinterpret_cast<uint8_t*>(&sMilliohmDeltaArrayEEPROM[i]), 0xFF);
                }

                /*
                 * Set compression flag
                 */
                StartValues.compressionFlag = FLAG_COMPRESSION;
                eeprom_write_byte(&EEPROMStartValues.compressionFlag, FLAG_COMPRESSION); // store compression flag in EEPROM
                if (!sOnlyPlotterOutput) {
                    Serial.println(F("Conversion done"));
                    Serial.println();
                }
            }
        }

        /*
         * Check again, in order to store also the value which triggered the conversion
         */
        if (ValuesForDeltaStorage.compressionIsActive) {
            /*
             * Store data in compressed format, 4 bit deltas
             */
            if ((unsigned int) ValuesForDeltaStorage.DeltaArrayIndex < MAX_NUMBER_OF_SAMPLES) {
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
                int16_t tVoltageDelta = aVoltageNoLoadMillivolt - ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt;
                tVoltageDelta = storeDeltas(tVoltageDelta, &ValuesForDeltaStorage.tempVoltageDelta,
                        reinterpret_cast<uint8_t*>(&sMillivoltDeltaArrayEEPROM[ValuesForDeltaStorage.DeltaArrayIndex]));
                ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt += tVoltageDelta;

                int16_t tMilliampereDelta = aMilliampere - ValuesForDeltaStorage.lastStoredMilliampere;
                tMilliampereDelta = storeDeltas(tMilliampereDelta, &ValuesForDeltaStorage.tempMilliampereDelta,
                        reinterpret_cast<uint8_t*>(&sMilliampereDeltaArrayEEPROM[ValuesForDeltaStorage.DeltaArrayIndex]));
                ValuesForDeltaStorage.lastStoredMilliampere += tMilliampereDelta;

                int16_t tMilliohmDelta = aMilliohm - ValuesForDeltaStorage.lastStoredMilliohm;
                tMilliohmDelta = storeDeltas(tMilliohmDelta, &ValuesForDeltaStorage.tempMilliohmDelta,
                        reinterpret_cast<uint8_t*>(&sMilliohmDeltaArrayEEPROM[ValuesForDeltaStorage.DeltaArrayIndex]));
                ValuesForDeltaStorage.lastStoredMilliohm += tMilliohmDelta;

                printlnIfNotPlotterOutput();

                if (ValuesForDeltaStorage.tempDeltaIsEmpty) {
                    ValuesForDeltaStorage.tempDeltaIsEmpty = false;
                } else {
                    // start two new 4 bit compressed values
                    ValuesForDeltaStorage.DeltaArrayIndex++; // increase every second sample
                    ValuesForDeltaStorage.tempDeltaIsEmpty = true;
                }
            }
        }
    }

    printValuesForPlotter(aVoltageNoLoadMillivolt, aMilliampere, aMilliohm, sDoPrintCaption);
    Serial.println();

}

void storeCapacityAndDischargeModeToEEPROM() {
    eeprom_write_word(&EEPROMStartValues.CapacityMilliampereHour, sBatteryInfo.CapacityMilliampereHour);
    eeprom_write_byte(&EEPROMStartValues.DischargeCutoffLevel, sDischargeCutoffLevel);
    if (!sOnlyPlotterOutput) {
// Print should be done after checkForDoublePress() in order to not disturb the double press detection
        Serial.print(F("Discharge mode "));
        Serial.print(getDischargeModeAsCharacter());
        Serial.print(F(" and capacity "));
        Serial.print(sBatteryInfo.CapacityMilliampereHour);
        Serial.println(F(" mAh stored"));
    }
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    myLCD.print(F("Capacity stored "));
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
}

/*
 * The reproduced ESR is likely to be noisy if the relation between the load resistor and the ESR is big.
 * E.g. for Li-ion we have an load resistor of 12.156 ohm and a voltage of 4.158 volt.
 * We then get an ESR of 0.109 ohm for 339 mA and 0.073 ohm for 340 mA :-(.
 */
void printValuesForPlotter(uint16_t aVoltageToPrint, uint16_t aMilliampereToPrint, uint16_t aMilliohmToPrint,
        bool aDoPrintCaption) {
#if defined(ARDUINO_2_0_PLOTTER_FORMAT)
Serial.print(F("Voltage:"));
Serial.print(aVoltageToPrint);
Serial.print(F(" Current:"));
Serial.print(aMilliampereToPrint);
Serial.print(F(" ESR:"));
Serial.print(aMilliohmToPrint);
if (aDoPrintSummary) {
    // Print updated plotter caption
    Serial.print(F(" Voltage="));
    printMillisValueAsFloat(StartValues.initialDischargingMillivolt);
    Serial.print(F("V->"));
    printMillisValueAsFloat(aVoltageToPrint);
    Serial.print(F("V__Current="));
    Serial.print(StartValues.initialDischargingMilliampere);
    Serial.print(F("mA->"));
    Serial.print(aMilliampereToPrint);
    Serial.print(F("mA__ESR="));
    Serial.print(StartValues.initialDischargingMilliohm);
    Serial.print(F("mohm->"));
    Serial.print(aMilliohmToPrint);
    Serial.print(F("mohm___LoadResistor="));
    printMillisValueAsFloat(StartValues.LoadResistorMilliohm);
    Serial.print(F("ohm__Capacity="));
    Serial.print(StartValues.CapacityMilliampereHour);
    Serial.print(F("mAh__Duration="));
    // We have 2 4bit values per storage byte
    uint16_t tDurationMinutes = (ValuesForDeltaStorage.DeltaArrayIndex)
    * (2 * NUMBER_OF_SAMPLES_PER_STORAGE)/ SECONDS_IN_ONE_MINUTE;
    Serial.print(tDurationMinutes / 60);
    Serial.print(F("h_"));
    Serial.print(tDurationMinutes % 60);
    Serial.print(F("min:aVoltageToPrint"));
}
#else
    if (aDoPrintCaption) {
// Print updated plotter caption
        Serial.print(F("Voltage="));
        printMillisValueAsFloat(StartValues.initialDischargingMillivolt);
        Serial.print(F("V->"));
        printMillisValueAsFloat(aVoltageToPrint);
        Serial.print(F("V:"));
        Serial.print(aVoltageToPrint);
        Serial.print(F(" Current="));
//        Serial.print(F("V Current="));
        Serial.print(StartValues.initialDischargingMilliampere);
        Serial.print(F("mA->"));
        Serial.print(aMilliampereToPrint);
        Serial.print(F("mA:"));
        Serial.print(aMilliampereToPrint);
        Serial.print(F(" ESR="));
//        Serial.print(F("mA ESR="));
        Serial.print(StartValues.initialDischargingMilliohm);
        Serial.print(F("mohm->"));
        Serial.print(aMilliohmToPrint);
        Serial.print(F("mohm:"));
        Serial.print(aMilliohmToPrint);
        Serial.print(F(" LoadResistor="));
//        Serial.print(F("mohm LoadResistor="));
        printMillisValueAsFloat(StartValues.LoadResistorMilliohm);
        Serial.print(F("ohm Capacity="));
        if (sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff != 0
                && sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff != sBatteryInfo.CapacityMilliampereHour) {
            Serial.print(sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff);
            Serial.print('_');
        }
        Serial.print(sBatteryInfo.CapacityMilliampereHour);
        Serial.print(F("mAh Duration"));
        uint16_t tDurationMinutes;
        if (StartValues.compressionFlag == FLAG_NO_COMPRESSION) {
// We have 1 8 bit delta value per storage byte
            tDurationMinutes = (ValuesForDeltaStorage.DeltaArrayIndex) * (NUMBER_OF_SECONDS_PER_STORAGE / SECONDS_IN_ONE_MINUTE);
        } else {
            Serial.print(F("_compr"));
// We have 2 4 bit delta values per storage byte
            tDurationMinutes = (ValuesForDeltaStorage.DeltaArrayIndex)
                    * (2 * NUMBER_OF_SECONDS_PER_STORAGE / SECONDS_IN_ONE_MINUTE);
        }
        Serial.print('=');
        Serial.print(tDurationMinutes / 60);
        Serial.print(F("h_"));
        Serial.print(tDurationMinutes % 60);
        Serial.print(F("min"));
    } else {
        Serial.print(aVoltageToPrint);
        Serial.print(' ');
        Serial.print(aMilliampereToPrint);
        Serial.print(' ');
        Serial.print(aMilliohmToPrint);
    }
#endif
}

/*
 * Copy EEPROM delta and start values to RAM
 */
void copyEEPROMDataToRam() {
    eeprom_read_block(sVoltageDeltaArray, reinterpret_cast<uint8_t*>(&sMillivoltDeltaArrayEEPROM),
    MAX_NUMBER_OF_SAMPLES);
    eeprom_read_block(sMilliampereDeltaArray, reinterpret_cast<uint8_t*>(&sMilliampereDeltaArrayEEPROM),
    MAX_NUMBER_OF_SAMPLES);
    eeprom_read_block(sMilliohmDeltaArray, reinterpret_cast<uint8_t*>(&sMilliohmDeltaArrayEEPROM),
    MAX_NUMBER_OF_SAMPLES);

    /*
     * Read start values data for later printing
     */
    eeprom_read_block(&StartValues, &EEPROMStartValues, sizeof(EEPROMStartValues));
}

#define CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE            0
#define CAPACITY_STARTED                1       // Current voltage is below or equal NominalFullVoltageMillivolt and higher or equal SwitchOffVoltageMillivoltHigh
#define CAPACITY_COMPLETED              2       // Current voltage is below SwitchOffVoltageMillivoltHigh
/*
 * Reads EEPROM delta values arrays
 * - print data for plotter and compute ESR on the fly from voltage, current and load resistor
 * - compute capacity from current (if defined SUPPORT_CAPACITY_RESTORE)
 * - restore battery type and capacity accumulator as well as mAh
 * - Capacity is stored in sBatteryInfo.CapacityMilliampereHour and sBatteryInfo.CapacityAccumulator
 */
void readAndProcessEEPROMData(bool aDoConvertInsteadOfPrint) {
    /*
     * First copy EEPROM delta and start values to RAM for later printing
     */
    copyEEPROMDataToRam();
    bool tIsCompressed = (StartValues.compressionFlag != FLAG_NO_COMPRESSION);

// search last non 0xFF (not cleared) value
    int tLastNonZeroIndex;
    for (tLastNonZeroIndex = (MAX_NUMBER_OF_SAMPLES - 1); tLastNonZeroIndex >= 0; --tLastNonZeroIndex) {
        if (sVoltageDeltaArray[tLastNonZeroIndex] != 0xFF || sMilliampereDeltaArray[tLastNonZeroIndex] != 0xFF
                || sMilliohmDeltaArray[tLastNonZeroIndex] != 0xFF) {
            break;
        }
    }
    tLastNonZeroIndex++; // Convert from 0 to MAX_NUMBER_OF_SAMPLES-1 to ValuesForDeltaStorage.DeltaArrayIndex to 0 to MAX_NUMBER_OF_SAMPLES

    sDischargeCutoffLevel = StartValues.DischargeCutoffLevel;
    uint16_t tVoltage = StartValues.initialDischargingMillivolt;
    sBatteryInfo.TypeIndex = getBatteryTypeIndex(tVoltage);
    uint8_t tCapacityMilliampereHourStandardValueState;
    /*
     * Check if start voltage > voltage for standard capacity computation
     * Assume, that voltage is not rising, so check at first value is sufficient
     */
    if (tVoltage >= BatteryTypeInfoArray[sBatteryInfo.TypeIndex].NominalFullVoltageMillivolt) {
        sBatteryInfo.isStandardCapacity = true;
        tCapacityMilliampereHourStandardValueState = CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE;
    } else {
        sBatteryInfo.isStandardCapacity = false;
        tCapacityMilliampereHourStandardValueState = CAPACITY_STARTED;
    }

    uint16_t tMilliampere = StartValues.initialDischargingMilliampere;
    uint16_t tMilliohm = StartValues.initialDischargingMilliohm;
    sBatteryInfo.Milliohm = tMilliohm; // displayed in summary print
    sBatteryInfo.Milliampere = tMilliampere; // To avoid overflow detection for Milliohm print
    sBatteryInfo.CapacityMilliampereHour = StartValues.CapacityMilliampereHour; // required for printing and append to EEPROM functionality
    sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff = 0;

// Required for conversion
    ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt = tVoltage;
    ValuesForDeltaStorage.lastStoredMilliampere = tMilliampere;
    ValuesForDeltaStorage.lastStoredMilliohm = tMilliohm;

    if (!sOnlyPlotterOutput) {
        Serial.println();
// We have always the first one as uncompressed value
        if (tIsCompressed) {
            Serial.print(tLastNonZeroIndex * 2);
            Serial.print(' ');
        } else {
            Serial.print(tLastNonZeroIndex + 1);
            Serial.print(F(" un"));
        }
        Serial.print(F("compressed EEPROM values found for type="));
        Serial.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].TypeName);
        Serial.print(F(" and discharge mode="));
        Serial.println(getDischargeModeAsCharacter());
    }

    uint32_t tCapacityAccumulator = tMilliampere;
    uint32_t tCapacityAccumulatorStartForStandardValue = 0;

    if (!aDoConvertInsteadOfPrint) {
        /*
         * Print the initial value and no caption to plotter
         */
        printValuesForPlotter(tVoltage, tMilliampere, tMilliohm, false);
        Serial.println();

// Required for appending to compressed values
        ValuesForDeltaStorage.DeltaArrayIndex = tLastNonZeroIndex;
        ValuesForDeltaStorage.compressionIsActive = tIsCompressed;
    }

// DeltaArrayIndex can be from 0 to MAX_NUMBER_OF_SAMPLES
    for (int i = 0; i < tLastNonZeroIndex; ++i) {

        if (!tIsCompressed) {
            /***************
             * Uncompressed
             ***************/
            tVoltage += (int8_t) sVoltageDeltaArray[i];
            tMilliampere += (int8_t) sMilliampereDeltaArray[i];
            tMilliohm += (int8_t) sMilliohmDeltaArray[i];
            if (aDoConvertInsteadOfPrint) {
                /*
                 * Convert uncompressed values here
                 */
                storeBatteryValuesToEEPROM(tVoltage, tMilliampere, tMilliohm);
//                Serial.print(F("Stored result "));
//                printValuesForPlotter(ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt, ValuesForDeltaStorage.lastStoredMilliampere, ValuesForDeltaStorage.lastStoredMilliohm, false);
            }
            tCapacityAccumulator += tMilliampere; // putting this into printValuesForPlotter() increases program size
#if defined(DEBUG)
            if (!sOnlyPlotterOutput) {
                Serial.print(F("EEPROM values="));
                Serial.print((int8_t) sVoltageDeltaArray[i]);
                Serial.print(' ');
                Serial.print((int8_t) sMilliampereDeltaArray[i]);
                Serial.print(' ');
                Serial.print((int8_t) sMilliohmDeltaArray[i]);
//                Serial.print(F(" CapAccu="));
//                Serial.print(tCapacityAccumulator);
                Serial.println();
            }
#endif

        } else {
            /*************
             * Compressed
             *************/
            /*
             * Process first part of compressed data
             */
            uint8_t t4BitVoltageDelta = sVoltageDeltaArray[i];
            tVoltage += getDelta(t4BitVoltageDelta >> 4);

            uint8_t t4BitMilliampereDelta = sMilliampereDeltaArray[i];
            tMilliampere += getDelta(t4BitMilliampereDelta >> 4);

            uint8_t t4BitMilliohmDelta = sMilliohmDeltaArray[i];
            tMilliohm += getDelta(t4BitMilliohmDelta >> 4);

            tCapacityAccumulator += tMilliampere; // putting this into printValuesForPlotter() increases program size
            /*
             * Print first part of expanded values
             */
            if (!sOnlyPlotterOutput || tLastNonZeroIndex < (MAX_VALUES_DISPLAYED_IN_PLOTTER / 2)) {
                /*
                 *  Skip every second value, if we have more than 500 uncompressed (250 compressed) values,
                 *  to fit the graph into the plotter display.
                 *  Skip first part of compressed data, since we printed initial value before loop
                 */
                printValuesForPlotter(tVoltage, tMilliampere, tMilliohm, false);
                Serial.println();
            }

            /*
             * Process second part of compressed data
             */
            tVoltage += getDelta(t4BitVoltageDelta & 0x0F);
            tMilliampere += getDelta(t4BitMilliampereDelta & 0x0F);
            tMilliohm += getDelta(t4BitMilliohmDelta & 0x0F);

            /*
             * Restoring capacity value from stored data, which have a bigger sample interval than measured data.
             * The observed delta was around 1%, so we can use this as a fallback, if no capacity data was stored e.g. in case of a sudden power down.
             * Increases program size by 184 bytes.
             */
            tCapacityAccumulator += tMilliampere;
#if defined(DEBUG)
            if (!sOnlyPlotterOutput) {
                Serial.print(F("EEPROM values=0x"));
                Serial.print(sVoltageDeltaArray[i], HEX);
                Serial.print(F(" 0x"));
                Serial.print(sMilliampereDeltaArray[i], HEX);
                Serial.print(F(" 0x"));
                Serial.print(sMilliohmDeltaArray[i], HEX);
//                Serial.print(F(" CapAccu="));
//                Serial.print(tCapacityAccumulator);
                Serial.println();
            }
#endif
        }

        if (!aDoConvertInsteadOfPrint) {

            uint16_t tVoltageForPrint = tVoltage; // To print markers for start and end of standard capacity
            uint8_t tPrintDelayed = 0; // to append text at values print output

            /*
             * Get "standard" capacity from NominalFullVoltageMillivolt to SwitchOffVoltageMillivoltHigh
             */
            if (tCapacityMilliampereHourStandardValueState == CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE
                    && tVoltage <= BatteryTypeInfoArray[sBatteryInfo.TypeIndex].NominalFullVoltageMillivolt) {
                // Store initial capacity at nominal full voltage to subtract it later
                tCapacityMilliampereHourStandardValueState = CAPACITY_STARTED;
                tCapacityAccumulatorStartForStandardValue = tCapacityAccumulator;
                tPrintDelayed = 1; // print text after print of values
                if (sOnlyPlotterOutput) {
                    tVoltageForPrint += 50; // modify voltage before print of values
                }

            } else if (tCapacityMilliampereHourStandardValueState == CAPACITY_STARTED
                    && tVoltage < BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivoltHigh) {
                tCapacityMilliampereHourStandardValueState = CAPACITY_COMPLETED;
                sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff = (tCapacityAccumulator
                        - tCapacityAccumulatorStartForStandardValue) / NUMBER_OF_STORAGES_PER_HOUR; // -> tCapacityAccumulator / 60
                if (i != tLastNonZeroIndex - 1) { // do not modify last value line containing caption
                    tPrintDelayed = 2; // print text after print of values
                    if (sOnlyPlotterOutput) {
                        tVoltageForPrint += 50; // modify voltage before print of values
                    }
                }
            }

            /*
             * Print (the second uncompressed) values
             * At last, print the caption with values from the end of the measurement cycle to plotter
             */
            printValuesForPlotter(tVoltageForPrint, tMilliampere, tMilliohm, (i == tLastNonZeroIndex - 1));
            /*
             * append
             */
            if (tPrintDelayed == 1) {
                if (!sOnlyPlotterOutput) {
                    Serial.print(F(" - Capacity on top of standard value="));
                    Serial.print(tCapacityAccumulator / NUMBER_OF_STORAGES_PER_HOUR);
                    Serial.print(F(" mAh"));
                }
            } else if (tPrintDelayed == 2) {
                if (!sOnlyPlotterOutput) {
                    Serial.print(F(" - "));
                    if (sBatteryInfo.isStandardCapacity) {
                        Serial.print(F("Standard "));
                    }
                    Serial.print(F("capacity at high cutoff="));
                    Serial.print(sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff);
                    Serial.print(F(" mAh"));
                }
            }
            Serial.println();
        }
    }

    /*
     * Loop was processed, handle capacity and LCD display now
     */
    if (!aDoConvertInsteadOfPrint) {

        uint16_t tCurrentCapacityMilliampereHourComputed = tCapacityAccumulator / NUMBER_OF_STORAGES_PER_HOUR;

        if (!sOnlyPlotterOutput) {
            if (sBatteryInfo.CapacityMilliampereHour == 0) {
                Serial.print(F("No capacity was stored, so use computed capacity of "));
                Serial.print(tCurrentCapacityMilliampereHourComputed);
            } else {
                /*
                 * The observed delta was around 1% :-)
                 */
                int16_t tCurrentCapacityMilliampereHourDelta = sBatteryInfo.CapacityMilliampereHour
                        - tCurrentCapacityMilliampereHourComputed;
                Serial.print(F("Stored minus computed capacity="));
                Serial.print(tCurrentCapacityMilliampereHourDelta);
            }
            Serial.println(F(" mAh"));

            /*
             * Print Standard capacity a between NominalFullVoltageMillivolt and  SwitchOffVoltageMillivoltHigh,
             * if we have both values.
             */
            if (sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff != 0
                    && sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff != tCurrentCapacityMilliampereHourComputed) {
                if (sBatteryInfo.isStandardCapacity) {
                    Serial.print(F("Standard "));
                }
                Serial.print(F("computed capacity between "));
                if (sBatteryInfo.isStandardCapacity) {
                    Serial.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].NominalFullVoltageMillivolt);
                } else {
                    Serial.print(StartValues.initialDischargingMillivolt);
                }
                Serial.print(F(" mV and "));
                Serial.print(BatteryTypeInfoArray[sBatteryInfo.TypeIndex].SwitchOffVoltageMillivoltHigh);
                Serial.print(F(" mV="));
                Serial.print(sBatteryInfo.CapacityMilliampereHourValueAtHighCutoff);
                Serial.println(F(" mAh"));
            }
        } // if (!sOnlyPlotterOutput)

        if (sBatteryInfo.CapacityMilliampereHour == 0) {
            sBatteryInfo.CapacityMilliampereHour = tCurrentCapacityMilliampereHourComputed;
        }

// restore capacity accumulator
        sBatteryInfo.CapacityAccumulator = sBatteryInfo.CapacityMilliampereHour
                * ((3600L * MILLIS_IN_ONE_SECOND) / SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS);

//        if (!sOnlyPlotterOutput) {
//            Serial.print(F("EEPROM values: "));
//        }

        /*
         * Store current values in structure for delta storage for append functionality
         */
        ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt = tVoltage;
        ValuesForDeltaStorage.lastStoredMilliampere = tMilliampere;
        ValuesForDeltaStorage.lastStoredMilliohm = tMilliohm;
        ValuesForDeltaStorage.tempDeltaIsEmpty = true;
    }
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
#else
    (void) aLineNumber;
#endif
}
