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
 * Version 4.0 - 2/2024
 *    Use capacity between NominalFullVoltageMillivolt and CutoffVoltageMillivoltHigh as standard capacity to enable better comparison.
 *    If powered by USB plotter pin logic is reversed, i.e. plotter output is enabled if NOT connected to ground.
 *    In state detecting battery, you can toggle cut off voltage between high, low and zero (0.1 V) with stop button.
 *    Fix bug for appending to compressed data.
 *    Synchronizing of LCD access for button handler, avoiding corrupted display content.
 *    Print improvements.
 *    Support for storage period of 120 s.
 *    Compression improved for rapidly descending voltage.
 *    Moving seldom used function of pin 10 to pin A5.
 *    New Logger mode with separate shunt enabled by pin 10.
 *    Store data in an array of structure instead in 3 arrays.
 *
 * Version 3.2.1 - 11/2023
 *    BUTTON_IS_ACTIVE_HIGH is not default any more
 * Version 3.2 - 10/2023
 *    Cut off LCD message improved
 * Version 3.1 - 3/2023
 *    Fixed "conversion does not clear rest of EEPROM" bug
 * Version 3.0 - 12/2022
 *    Improved compression
 * Version 2.3 - 10/2022
 *    Increase no load settle time especially for NiMh batteries
 *    Attention tones
 * Version 2.2 - 8/2022
 *    ESR > 64 bug fixed.
 *    Display of changes on pin CUTOFF_LEVEL_PIN
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
//#define TRACE
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
 *
 * Pin 2 / INT0 is used for Start/Stop button
 * Pin 3 to 8 are used for parallel LCD connection
 */
#define ADC_CHANNEL_VOLTAGE          0 // A0 for voltage measurement
#define ADC_CHANNEL_CURRENT          1 // A1 for current measurement
#define VOLTAGE_RANGE_EXTENSION_PIN A2 // This pin is low to extend the voltage range from 2.2 volt to 4.4 volt
#define LOAD_HIGH_PIN               A3 // This pin is high to switch on the high load (3 ohm)
// A4 + A5, the hardware I2C pins on Arduino, are used for Serial LCD
#define ADC_CHANNEL_LOGGER_CURRENT   4 // A4 for current measurement for Logger
#define BUZZER_PIN                  A5
// Mode pins
#define ONLY_PLOTTER_OUTPUT_PIN      9 // If powered by Li-ion, verbose output to Arduino Serial Monitor is disabled, if connected to ground. This is intended for Arduino Plotter mode.
#define ONLY_LOGGER_MODE_PIN        10 // If connected to ground, current is measured at the shunt at A4 and voltage still at A0.
// If powered by USB verbose verbose output to Arduino Serial Monitor is disabled, if NOT connected to ground.
#define CUTOFF_LEVEL_PIN            11 // If connected to ground, "cut off is low" is displayed and discharge ends at a lower voltage. E.g. Li-ion discharge ends at 3000 mV instead of 3500 mV
#define LOAD_LOW_PIN                12 // This pin is high to switch on the low load (10 ohm). A4 is occupied by I2C for serial LCD display.

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

bool sOnlyLoggerFunctionality;           // contains the (inverted) value of the pin ONLY_LOGGER_MODE_PIN

#define CUTOFF_LEVEL_HIGH       0 // Switch off current percentage is 50% for logger. Is default case
#define CUTOFF_LEVEL_LOW        1 // 25% for logger.
#define CUTOFF_LEVEL_ZERO       2 // 12% for logger. End discharging at 0.05 volt (DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT)
uint8_t sCutoffLevel; // One of CUTOFF_LEVEL_HIGH, CUTOFF_LEVEL_LOW and CUTOFF_LEVEL_ZERO. Starts with the (inverted) value of the pin CUTOFF_LEVEL_PIN
#define DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT   50  // 50 mV
#define NO_BATTERY_MILLIVOLT                    (DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT / 2)   // 25 mV
#define NO_LOGGER_MILLAMPERE                    12
/*
 * Cutoff detection is done by comparing current mA value with low pass mA value
 * in sLastMilliampereLowPassFiltered5 shifted by 1 or 2 or 3 (divide by 2 or 4 or 8) corresponding to 50% or 25% or 12.5%
 */
uint16_t sLastMilliampereLowPassFiltered5;           // For sCutoffMilliamperePercent
bool sLastValueOfCutoffLevelPin;                    // To support changing between normal and low by using pin CUTOFF_LEVEL_PIN

/*
 * External circuit definitions
 */
#define LOGGER_SHUNT_RESISTOR_MILLIOHM          200L  // 0.2 ohm -> Resolution of 5 mA
#define ESR_SHUNT_RESISTOR_MILLIOHM             2000L  // 2 ohm
#define LOAD_LOW_MILLIOHM                       (1000 + ESR_SHUNT_RESISTOR_MILLIOHM) // Additional 1 ohm
#define LOAD_HIGH_MILLIOHM                      (10 * 1000 + ESR_SHUNT_RESISTOR_MILLIOHM) // Additional 10 ohm
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
// redefine the two pins, because A4 + A5 is used by I2C
#undef ONLY_PLOTTER_OUTPUT_PIN
#undef ADC_CHANNEL_LOGGER_CURRENT
#define ONLY_PLOTTER_OUTPUT_PIN             8 // This pin is high to switch on the high load (3 ohm)
#define ADC_CHANNEL_LOGGER_CURRENT          6 // A6 available on Nano, not on Uno for current measurement for Logger
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
#define NUMBER_OF_INITIAL_SAMPLES               4 // 4 seconds (-2 for initial display of append message) before starting discharge and storing, to have time to just test for ESR of battery.
#define SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS   500 // The time of the activated load for one sample.
#define NUMBER_OF_SAMPLES_PER_STORAGE             5 // 1 minute, if we have 1 sample per second
#else
#define NUMBER_OF_INITIAL_SAMPLES               30 // Before starting discharge and storing, to have time to just test for ESR of battery. 30 seconds with SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS as 1000.
#define SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS   MILLIS_IN_ONE_SECOND // 1 s. The time of the activated load for one sample.
#  if !defined(NUMBER_OF_SAMPLES_PER_STORAGE)
#define NUMBER_OF_SAMPLES_PER_STORAGE           SECONDS_IN_ONE_MINUTE // 60, if we have 1 sample per second (SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS)
//#define NUMBER_OF_SAMPLES_PER_STORAGE           (2 * SECONDS_IN_ONE_MINUTE) // 120, if we have 1 sample per second (SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS)
#  endif
#endif

#define LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT 769L // for ADC_PRESCALE32 and 20 ms (50 Hz)
#define LOGGER_SAMPLE_PERIOD_MILLIS             50 // 20 Hz.
#define LOGGER_SAMPLE_FREQUENCY_HZ              (MILLIS_IN_ONE_SECOND / LOGGER_SAMPLE_PERIOD_MILLIS)
#define LOGGER_SAMPLES_PER_MINUTE               (SECONDS_IN_ONE_MINUTE * LOGGER_SAMPLE_FREQUENCY_HZ * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT)  // = 230.400 every minute for 10 HZ

#define NUMBER_OF_SECONDS_PER_STORAGE           NUMBER_OF_SAMPLES_PER_STORAGE // NUMBER_OF_SAMPLES_PER_STORAGE, if we have 1 sample per second
#define NUMBER_OF_STORAGES_PER_HOUR             (3600L / NUMBER_OF_SECONDS_PER_STORAGE)

#define MAX_VALUES_DISPLAYED_IN_PLOTTER         500 // The Arduino 1.8 Plotter displays 500 values before scrolling
#define BATTERY_OR_VOLTAGE_DETECTION_PERIOD_MILLIS         (MILLIS_IN_ONE_SECOND / 2) // 500 ms
#define BATTERY_DETECTION_MINIMAL_MILLIVOLT     50

/*
 * Values for different battery types
 */
struct BatteryTypeInfoStruct {
    const char TypeName[11];
    uint16_t DetectionThresholdVoltageMillivolt; // Type is detected if voltage is below this threshold
    uint16_t NominalFullVoltageMillivolt;       // The voltage to start the "standard" capacity computation
    uint16_t CutoffVoltageMillivoltHigh; // The voltage to stop the "standard" capacity computation. Cut off happens below this voltage
    uint16_t CutoffVoltageMillivoltLow;
    uint8_t LoadType;                           // High (3 Ohm) or low (12 Ohm)
    uint16_t LoadSwitchSettleTimeMillis; // Time for voltage to settle after load switch was disabled => time of NoLoad during one sample
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
struct BatteryOrLoggerInfoStruct {
    union VoltagesUnion {
        struct BatteryVoltagesStruct {
            uint16_t NoLoadMillivolt;
            uint16_t LoadMillivolt;
        } Battery;
        struct LoggerVoltagesStruct {
            // The first value is stored in EEPROM
            uint16_t MinimumMillivolt;
            uint16_t AverageMillivolt;
            uint16_t MaximumMillivolt;
        } Logger;
    } Voltages;
    uint16_t Milliampere;
    uint16_t ESRMilliohm; // Average of last 60 values. ESR - Equivalent Series Resistor | internal battery resistance.
    uint16_t sESRDeltaMillivolt = 0; // only displayed at initial ESR testing
    uint32_t CapacityAccumulator;
    uint16_t CapacityMilliampereHour;

    // not used for logger
    uint16_t CapacityMilliampereHourValueAtHighCutoff;
    bool isStandardCapacity;

    uint8_t LoadState; // NO_LOAD | LOW_LOAD 12 ohm | HIGH_LOAD 3 ohm
    uint8_t BatteryTypeIndex;
} sMeasurementInfo;

uint16_t sLastVoltageNoLoadMillivoltForPrintAndCountdown;
bool sBatteryOrVoltageAndCurrentWasDetected = false; // set by detectBatteryOrLoggerVoltageAndCurrentLCD() if detection was successful
bool sButtonUsageMessageWasPrinted = false;
bool sVoltageNoLoadIsDisplayedOnLCD = false;

bool sVoltageRangeIsLow;                        // true for 2.2 V range

uint8_t sLoggerADCVoltageReference = INTERNAL;  // INTERNAL or DEFAULT
uint16_t sLoggerMaximumRawVoltage;
uint16_t sLoggerMinimumRawVoltage;
uint16_t sLogger1SecondRawSampleCount;          // is 20 for 20 HZ
uint32_t sLogger1SecondRawVoltageAccumulator;   // normalized for 4.4 V range
uint32_t sLogger1SecondRawCurrentAccumulator;   // 16 bit is only OK up to 50 Hz
uint16_t sLogger1MinuteRawSampleCount;          // is 3000 every minute for 50 HZ
uint32_t sLogger1MinuteRawVoltageAccumulator8ShiftRight; // Unshifted maximum is 5000 * 769 * 20 * 60 = 6.921 billion => overflow at 12.2 Volt
uint32_t sLogger1MinuteRawCurrentAccumulator;   // Unshifted maximum is 1023 * 769 * 20 * 60 = 1.416 billion
/*
 * Tester state machine
 */
#define STATE_SETUP_AND_READ_EEPROM         0
#define STATE_DETECTING_BATTERY_OR_VOLTAGE  1 // Check if battery is inserted and determine type or external voltage is connected to start with logging
#define STATE_INITIAL_SAMPLES               2 // Only voltage (and ESR) measurement every n seconds for NUMBER_OF_INITIAL_SAMPLES samples
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
unsigned long sLastStateDetectingBatteryOrVoltageBeepMillis;
unsigned long sLastStateStoppedBeepMillis;

unsigned long sSampleCountForStoring;
unsigned long sLastMillisOfSample = 0;
unsigned long sLastMillisOfLoggerSample = 0;
unsigned long sLastMillisOfBatteryOrVoltageDetection = 0;
uint8_t sNumbersOfInitialSamplesToGo;

/*
 * Current value is in sCurrentLoadResistorHistory[0]. Used for computing and storing the average.
 */
#define HISTORY_SIZE_FOR_LOAD_RESISTOR_AVERAGE 16
uint16_t sCurrentLoadResistorHistory[HISTORY_SIZE_FOR_LOAD_RESISTOR_AVERAGE];
uint16_t sCurrentLoadResistorAverage;

/*
 * Current value is in sESRHistory[0]. The average is stored in sMeasurementInfo.ESRMilliohm.
 * For 9V and 60 mA we have a resolution of 0.3 ohm so we need an average.
 */
#define HISTORY_SIZE_FOR_ESR_AVERAGE    60
uint16_t sESRHistory[HISTORY_SIZE_FOR_ESR_AVERAGE];

/*
 * EEPROM store, It seems that EEPROM is allocated top down and in called or referenced sequence
 * https://arduino.stackexchange.com/questions/93873/how-eemem-maps-the-variables-avr-eeprom-h
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
    uint8_t CutoffLevel;

    uint16_t CapacityMilliampereHour; // is set at end of measurement or by store button
};
EEPROMStartValuesStruct StartValues;
EEMEM EEPROMStartValuesStruct EEPROMStartValues;

#if defined(TEST)
#define MAX_NUMBER_OF_SAMPLES   9
#else
// EEPROM size for values is (1024 - sizeof(EEPROMStartValues)) / 3 = 1012 / 3 = 337.3
#define MAX_NUMBER_OF_SAMPLES      (E2END - sizeof(EEPROMStartValuesStruct)) / 3 // 337 + 1 since we always have the initial value. 5.6 h / 11.2 h for 1 minute sample rate
#endif
union EEPROMDataUnion {
    struct {
        int8_t DeltaMillivolt; // one 8 bit delta
        int8_t DeltaMilliampere;
        int8_t DeltaESRMilliohm;
    } uncompressed;
    struct {
        uint8_t DeltaMillivolt; // contains 2 4 bit deltas
        uint8_t DeltaMilliampere;
        uint8_t DeltaESRMilliohm;
    } compressed;
};
EEMEM EEPROMDataUnion EEPROMDataArray[MAX_NUMBER_OF_SAMPLES];

/*
 * Every compressed array byte contains two 4 bit values
 * The upper 4 bit store the first value, the lower 4 bit store the second value
 * 8 is added to the 4 bit integer (range from -8 and 7) to get positive values for storage
 */
struct ValuesForDeltaStorageStruct {
    uint16_t lastStoredMilliampere;
    uint16_t lastStoredVoltageNoLoadMillivolt;
    uint16_t lastStoredMilliohm;
    EEPROMDataUnion tempDeltas;
    bool tempDeltaIsEmpty;
    bool compressionIsActive;
    int DeltaArrayIndex; // The index of the next values to be written. -1 to signal, that start values must be written.
} ValuesForDeltaStorage;

bool sDoPrintCaption = true; // Value used for (recursive) call to printValuesForPlotter().

void getBatteryVoltageMillivolt();
void addToCapacity();
uint16_t getBatteryRawVoltage();
void detectBatteryOrLoggerVoltageAndCurrentLCD();
void clearLogger1SecondAccumulator();
void clearLogger1MinuteAccumulator();
void getLoggerCurrent();
void getLogger1SecondValues();
void getLogger1MinuteValues();
void accumulateLoggerValues();

void getBatteryCurrent();
void getBatteryValues();
void checkAndHandleStopConditionLCD();
bool checkForVoltageOrCurrentRemoved();
void playEndTone();
void playAttentionTone();
void setLoad(uint8_t aNewLoadState);
void printStoredDataLCD();
void printMilliampere4DigitsLCD();
void printVoltageNoLoadMillivoltWithTrailingSpaceLCD();
void printMeasurementValuesLCD();
void printValuesForPlotter(uint16_t aVoltageToPrint, uint16_t aMilliampereToPrint, uint16_t aMilliohmToPrint, bool aDoPrintCaption);
void printMillisValueAsFloat(uint16_t aValueInMillis);

void dumpEEPROM(uint8_t *aEEPROMAdress, uint8_t aNumberOf16ByteBlocks);
void storeBatteryValuesToEEPROM(uint16_t aVoltageNoLoadMillivolt, uint16_t aMilliampere, uint16_t aMilliohm);
void storeCapacityAndCutoffLevelToEEPROM_LCD();
void readAndProcessEEPROMData(bool aDoConvertInsteadOfPrint);

void delayAndCheckForButtonPress();
void printButtonUsageMessageLCD();
char getCutoffLevelAsCharacter();
void printCutoff();
void printlnIfNotPlotterOutput();
void printStateString(uint8_t aState);

void switchToStateDetectingBatteryOrVoltage();
void switchToStateInitialSamples();
void switchToStateStoreToEEPROM();
void switchToStateStoppedLCD(char aReasonCharacter);

void TogglePin(uint8_t aPinNr);

#if defined(USE_LCD)
void LCDPrintAsFloatWith2Decimals(uint16_t aValueInMillis);
void LCDPrintAsFloatWith3Decimals(uint16_t aValueInMillis);
void LCDPrintVCC();
void LCDClearLine(uint8_t aLineNumber);
#endif

// Helper macro for getting a macro definition as string
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

/*
 * Program starts here
 */
#if defined(TRACE) && !defined(LOCAL_TRACE)
#define LOCAL_TRACE
#else
//#define LOCAL_TRACE // This enables TRACE output only for this file
#endif
#if defined(DEBUG) && !defined(LOCAL_DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file
#endif

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LOAD_HIGH_PIN, OUTPUT);
    pinMode(LOAD_LOW_PIN, OUTPUT);
    pinMode(ONLY_PLOTTER_OUTPUT_PIN, INPUT_PULLUP);
    pinMode(CUTOFF_LEVEL_PIN, INPUT_PULLUP);
    pinMode(ONLY_LOGGER_MODE_PIN, INPUT_PULLUP);
    setLoad(NO_LOAD);

    // Pin settings for sVoltageRangeIsLow = false;
    pinMode(VOLTAGE_RANGE_EXTENSION_PIN, OUTPUT);
    digitalWrite(VOLTAGE_RANGE_EXTENSION_PIN, LOW);

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif

    sOnlyLoggerFunctionality = !digitalRead(ONLY_LOGGER_MODE_PIN);

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
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS / 2);

    myLCD.setCursor(0, 1);
    if (sOnlyLoggerFunctionality) {
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Only logger mode"));
        }
        myLCD.print(F("Only logger mode"));
    }
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
    LCDClearLine(1); // Clear line "Only logger mode"

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
     * get sCutoffLevel for later appending
     */
    readAndProcessEEPROMData(false);
    printStoredDataLCD();

    printlnIfNotPlotterOutput(); // end of stored data

    if (sOnlyLoggerFunctionality) {
        sMeasurementInfo.ESRMilliohm = 0; // not used in logger function
    }

    /*
     * Read value to variable in order to force printing triggered by value change :-)
     */
    sLastValueOfCutoffLevelPin = digitalRead(CUTOFF_LEVEL_PIN);

    /*
     * If battery is still inserted, keep cut off level. I.e. measurement is easy to be continued.
     * If battery was removed, cut off level can be chosen by pressing stop button.
     */
    getBatteryVoltageMillivolt();
    if (sMeasurementInfo.Voltages.Battery.NoLoadMillivolt < NO_BATTERY_MILLIVOLT) {
        // Battery is removed here, so start with mode determined by pin
        sCutoffLevel = !sLastValueOfCutoffLevelPin;
        if (sOnlyLoggerFunctionality && sCutoffLevel == CUTOFF_LEVEL_HIGH) {
            // Pin not connected leads to cut off level zero (instead of high) for logger
            sCutoffLevel = CUTOFF_LEVEL_ZERO;
        }
        StartValues.CutoffLevel = sCutoffLevel; // Required, in order to keep level during conversion to compressed.
    }
    printCutoff(); // print actual cut off level

    if (sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff != 0) {
        /*
         * Display standard capacity info
         */
        if (!sOnlyPlotterOutput) {
            if (sMeasurementInfo.isStandardCapacity) {
                Serial.print(F("Standard "));
            }
            Serial.print(F("capacity="));
            Serial.print(sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff);
            Serial.println(F(" mAh"));
        }
#if defined(USE_LCD)
        myLCD.setCursor(7, 1);
        if (sMeasurementInfo.isStandardCapacity) {
            // replace h, l or z by s
            myLCD.print('s');
        } else if (sMeasurementInfo.CapacityMilliampereHour == sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff) {
            // keep h, l or z
            myLCD.setCursor(8, 1);
        } else {
            // remove h, l or z, because it is NOT standard capacity and not total capacity
            myLCD.print(' ');
        }
        char tString[6];
        sprintf_P(tString, PSTR("%5u"), sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff);
        myLCD.print(tString);
        myLCD.print(F("mAh"));
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
    }

    switchToStateDetectingBatteryOrVoltage();
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
        switchToStateStoppedLCD('U');
    }

    /*
     * Handle battery detection
     */
    if (sMeasurementState == STATE_DETECTING_BATTERY_OR_VOLTAGE) {
        if (millis() - sLastMillisOfBatteryOrVoltageDetection >= BATTERY_OR_VOLTAGE_DETECTION_PERIOD_MILLIS) {

            sLastMillisOfBatteryOrVoltageDetection = millis();
            /*
             * Check if battery was inserted or voltage connected
             */
            detectBatteryOrLoggerVoltageAndCurrentLCD();
            // we waited up to 2 seconds in detectBatteryOrLoggerVoltageAndCurrentLCD(), so must check if mode has not changed by button press
            if (sMeasurementState == STATE_DETECTING_BATTERY_OR_VOLTAGE) {
                if (sBatteryOrVoltageAndCurrentWasDetected) {
                    /*
                     * Successfully detected here
                     */
                    /*
                     * show values detected
                     */
                    printMeasurementValuesLCD();

#if defined(USE_LCD)
                    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
                    switchToStateInitialSamples();
                    // If found, print button usage once at start of InitialESRMeasurement
                    if (!sButtonUsageMessageWasPrinted) {
                        printButtonUsageMessageLCD();
                        sButtonUsageMessageWasPrinted = true;
                        sVoltageNoLoadIsDisplayedOnLCD = false;
                    }
                    if (sOnlyLoggerFunctionality) {
#if defined(USE_LCD)
                        LCDClearLine(1); // Clear line "append to EEPROM"
#endif
                        /*
                         * Initialize cutoff reference value
                         */
                        sLastMilliampereLowPassFiltered5 = sMeasurementInfo.Milliampere;
                        clearLogger1SecondAccumulator();
                        clearLogger1MinuteAccumulator();
                    } else {
                        // set load for the first call of getBatteryValues() to measure the current
                        setLoad(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].LoadType);
                    }

                } else {
                    /*
                     * Not detected here
                     * Print VCC voltage, but not if voltage was detected or battery was inserted before,
                     * to not overwrite battery/logger voltage printed in state STATE_INITIAL_SAMPLES
                     *          "0123456789012345"
                     * All:     "<Volt ><Message>" message starting at 6 with at least one space
                     * Logger:  "3.98V  No U or I"
                     * Logger:  "1.500V       0mA", if we have only voltage attached
                     * Logger:  "3.98V       22mA", if we have only current attached
                     * Battery: "3.98V   No batt."
                     * Battery: "1.500V  No batt.", if we displayed battery voltage before.
                     */
                    bool tNoLoadVoltageOrCurrentPrinted = false; // for printlnIfNotPlotterOutput() below
                    if (sMeasurementInfo.Voltages.Battery.NoLoadMillivolt > NO_BATTERY_MILLIVOLT) {
                        printVoltageNoLoadMillivoltWithTrailingSpaceLCD();
                        tNoLoadVoltageOrCurrentPrinted = true;
                    } else if (!sVoltageNoLoadIsDisplayedOnLCD) {
#if defined(USE_LCD)
                        LCDPrintVCC();
#endif
                    }

                    if (tNoLoadVoltageOrCurrentPrinted || sMeasurementInfo.Milliampere > 0) {
                        /*
                         * Here U or I is valid, but not both
                         */
#if defined(USE_LCD)
                        myLCD.setCursor(7, 0); // Clear eventual cut off message
                        myLCD.print(F("   "));
#endif
                        printMilliampere4DigitsLCD();
                        tNoLoadVoltageOrCurrentPrinted = true;
                    }

                    if (tNoLoadVoltageOrCurrentPrinted) {
                        printlnIfNotPlotterOutput();
                    }
                    /*
                     * if not connected to USB, check for attention every minute
                     */
                    if (!isVCCUSBPowered() && millis() - sLastStateDetectingBatteryOrVoltageBeepMillis >= STATE_BATTERY_DETECTION_ATTENTION_PERIOD_MILLIS) {
                        sLastStateDetectingBatteryOrVoltageBeepMillis = millis();
                        playAttentionTone();
                    }
                }
            }
        }
        /*
         * End of battery or voltage detection handling
         */

    } else {
        /*
         * STATE_INITIAL_SAMPLES or STATE_STORE_TO_EEPROM or STATE_STOPPED here
         */
        if (sOnlyLoggerFunctionality && ((unsigned) (millis() - sLastMillisOfLoggerSample) >= LOGGER_SAMPLE_PERIOD_MILLIS)) {
            sLastMillisOfLoggerSample = millis();
            /*
             * Accumulate logger values
             */
            accumulateLoggerValues();
        }

// For discharging, add LoadSwitchSettleTimeMillis to the second
        if ((sOnlyLoggerFunctionality && sLogger1SecondRawSampleCount == LOGGER_SAMPLE_FREQUENCY_HZ)
                || (!sOnlyLoggerFunctionality
                        && (unsigned) (millis() - sLastMillisOfSample)
                                >= (SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS
                                        + BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].LoadSwitchSettleTimeMillis))) {
            /*
             * Here battery is inserted or voltage detected and sample period (one second) expired
             * sMeasurementState is STATE_INITIAL_SAMPLES or STATE_STORE_TO_EEPROM or STATE_STOPPED
             * Do all this every second (of battery load)
             */
            sLastMillisOfSample = millis();

            /*
             * Get values
             */
            if (sOnlyLoggerFunctionality) {
                getLogger1SecondValues();
            } else {
                getBatteryValues();
            }

            if (sMeasurementState == STATE_STOPPED) {
                /*
                 * STATE_STOPPED here
                 */
                if (abs(
                        sLastVoltageNoLoadMillivoltForPrintAndCountdown
                        - sMeasurementInfo.Voltages.Battery.NoLoadMillivolt) > 5) {
                    printVoltageNoLoadMillivoltWithTrailingSpaceLCD();
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
                /*
                 * STATE_INITIAL_SAMPLES or STATE_STORE_TO_EEPROM here
                 * Check stop or remove conditions and display values
                 */
                if (checkForVoltageOrCurrentRemoved()) {
                    switchToStateDetectingBatteryOrVoltage(); // switch back to start and do not overwrite already displayed values
                } else {
                    // Here not stopped or just removed -> Print measurement values
                    addToCapacity();
                    printMeasurementValuesLCD();
                    if (sOnlyLoggerFunctionality) {
                        checkAndHandleStopConditionLCD(); // Check every second if current drops. For battery, we want to store the value of the stop condition in EEPROM first!
                    }
                }
            }

            /*
             * Check for end of STATE_INITIAL_SAMPLES
             */
            if (sMeasurementState == STATE_INITIAL_SAMPLES && sNumbersOfInitialSamplesToGo == 0) {
                if (!sOnlyLoggerFunctionality
                        && sMeasurementInfo.Voltages.Battery.NoLoadMillivolt
                                < BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].NominalFullVoltageMillivolt) {
                    if (!sOnlyPlotterOutput) {
                        Serial.print(F("Start voltage "));
                        Serial.print(sMeasurementInfo.Voltages.Battery.NoLoadMillivolt);
                        Serial.print(F(" V is below NominalFullVoltageMillivolt of "));
                        Serial.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].NominalFullVoltageMillivolt);
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
                if (sMeasurementState == STATE_INITIAL_SAMPLES) {
                    // Force new data set
                    ValuesForDeltaStorage.DeltaArrayIndex = -1;
                    sMeasurementInfo.CapacityAccumulator = 0;

                    // Must reset this values here, because values are displayed before computed again from CapacityAccumulator
                    sMeasurementInfo.CapacityMilliampereHour = 0;
                    sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff = 0;

                    memset(sCurrentLoadResistorHistory, 0, sizeof(sCurrentLoadResistorHistory)); // Clear history array

                    switchToStateStoreToEEPROM();
                    sSampleCountForStoring = NUMBER_OF_SAMPLES_PER_STORAGE; // store first value immediately
                }
                /*
                 * end of state STATE_INITIAL_SAMPLES
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

                    if (sOnlyLoggerFunctionality) {
                        getLogger1MinuteValues(); // get the smoother values here for storing to EEPROM
                    }
                    if (sMeasurementState != STATE_STOPPED) {
                        /*
                         * Store to EEPROM
                         */
                        storeBatteryValuesToEEPROM(sMeasurementInfo.Voltages.Battery.NoLoadMillivolt, sMeasurementInfo.Milliampere,
                                sMeasurementInfo.ESRMilliohm);
                        if (ValuesForDeltaStorage.DeltaArrayIndex == MAX_NUMBER_OF_SAMPLES
                                && ValuesForDeltaStorage.compressionIsActive) {
                            /*
                             * Print message and stop, if compressed buffer is full
                             */
                            if (!sOnlyPlotterOutput) {
                                Serial.println(F("EEPROM delta values array full -> stop measurement"));
                            }
                            switchToStateStoppedLCD('F');

                        } else if (!sOnlyLoggerFunctionality && ValuesForDeltaStorage.tempDeltaIsEmpty) {
                            /*
                             * For battery, we want to store the value of the stop condition in EEPROM first!
                             */
                            checkAndHandleStopConditionLCD();
                        }
                    }
                }
                /*
                 * End of handle storing to EEPROM
                 */
            }
        }
    } // end of handling each second

    if (!sOnlyLoggerFunctionality) {
        delay(100);

        /*
         * Check for plotter mode pin change
         * Is not really required for logger and sets reference to default,
         * which requires additional 8 ms for logger to switch back to 4.4 V range
         */
        sOnlyPlotterOutput = !digitalRead(ONLY_PLOTTER_OUTPUT_PIN);
        if (isVCCUSBPowered()) {
            sOnlyPlotterOutput = !sOnlyPlotterOutput; // reversed behavior if powered by USB
        }
    }

    /*
     * Check for cut off level pin change
     */
    bool tValueOfCutoffLevelPin = digitalRead(CUTOFF_LEVEL_PIN);
    if (sLastValueOfCutoffLevelPin != tValueOfCutoffLevelPin) {
        sLastValueOfCutoffLevelPin = tValueOfCutoffLevelPin;
        sCutoffLevel = !sLastValueOfCutoffLevelPin;
        // Update EEPROM value for right start value for next appending
        if (StartValues.CutoffLevel != sCutoffLevel) {
            StartValues.CutoffLevel = sCutoffLevel;
            eeprom_update_byte(&EEPROMStartValues.CutoffLevel, sCutoffLevel);
        }
        printCutoff();
    }
}

void printStateString(uint8_t aState) {
    if (!sOnlyPlotterOutput) {
        if (aState == STATE_DETECTING_BATTERY_OR_VOLTAGE) {
            Serial.print(F("DETECTING BATTERY_OR_VOLTAGE"));
        } else if (aState == STATE_INITIAL_SAMPLES) {
            Serial.print(F("INITIAL SAMPLES"));
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

void switchToStateDetectingBatteryOrVoltage() {
    sMeasurementState = STATE_DETECTING_BATTERY_OR_VOLTAGE;
    printSwitchStateString();
    printlnIfNotPlotterOutput();
    sLastStateDetectingBatteryOrVoltageBeepMillis = millis();
}

void switchToStateInitialSamples() {
    sMeasurementState = STATE_INITIAL_SAMPLES;
    printSwitchStateString();
    printlnIfNotPlotterOutput();
    sNumbersOfInitialSamplesToGo = NUMBER_OF_INITIAL_SAMPLES;
    memset(sESRHistory, 0, sizeof(sESRHistory));
}

void switchToStateStoreToEEPROM() {
    sMeasurementState = STATE_STORE_TO_EEPROM;
    printSwitchStateString();
    printlnIfNotPlotterOutput();
// Required for right appending to EEPROM, so do it here
    ValuesForDeltaStorage.tempDeltaIsEmpty = true;
    eeprom_update_byte(&EEPROMStartValues.CutoffLevel, sCutoffLevel);
}

/*
 * aWriteToLCD default is true.
 * @param aReasonCharacter, '-' for terminating condition met (regular end of measurement), U for VCC undervoltage, F for EEPROM full,
 *                           D for button double press, B for button press.
 */
void switchToStateStoppedLCD(char aReasonCharacter) {
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
            storeCapacityAndCutoffLevelToEEPROM_LCD(); // Store capacity and cut off level
        }

#if defined(USE_LCD)
        myLCD.setCursor(0, 0);
        myLCD.print(F("       Stopped "));
        myLCD.print(aReasonCharacter);
#endif
        sLastVoltageNoLoadMillivoltForPrintAndCountdown = 0; // to force display of NoLoad voltage
    }
}

char getCutoffLevelAsCharacter() {
    if (sCutoffLevel == CUTOFF_LEVEL_ZERO) {
        return 'z';
    } else if (sCutoffLevel == CUTOFF_LEVEL_LOW) {
        return 'l';
    } else { // CUTOFF_LEVEL_HIGH
        return 'h';
    }
}

#if defined(USE_LCD)
void LCDClearLine(uint8_t aLineNumber) {
    myLCD.setCursor(0, aLineNumber);
    myLCD.print("                    ");
    myLCD.setCursor(0, aLineNumber);
}

void LCDPrintAsFloatWith3Decimals(uint16_t aValueInMillis) {
    myLCD.print(((float) (aValueInMillis)) / 1000, 3);
}

void LCDPrintAsFloatWith2Decimals(uint16_t aValueInMillis) {
    myLCD.print(((float) (aValueInMillis)) / 1000, 2);
}

void LCDPrintVCC() {
    myLCD.setCursor(0, 0);
    sVCCVoltageMillivolt = getVCCVoltageMillivolt();
    myLCD.print(sVCCVoltageMillivolt / 1000.0, 2);
    myLCD.print(F("V  ")); // 2 spaces to overwrite "Cut off"
}

void LCDPrintCutoff() {
    myLCD.setCursor(0, 0);
    if (sOnlyLoggerFunctionality) {
        myLCD.print(F("Cut off is "));
        myLCD.print(100 >> (sCutoffLevel + 1));
        myLCD.print(F("% I"));
    } else {
        auto tCutoffLevel = sCutoffLevel;
        if (tCutoffLevel == CUTOFF_LEVEL_ZERO) {
            myLCD.print(F("Cut off is 50 mV"));
        } else {
            if (sMeasurementInfo.BatteryTypeIndex == TYPE_INDEX_NO_BATTERY) {
                // Long text without voltage
                myLCD.print(F("Cut off is "));
                if (tCutoffLevel == CUTOFF_LEVEL_LOW) {
                    myLCD.print(F("low  "));
                } else { // CUTOFF_LEVEL_HIGH
                    myLCD.print(F("high "));
                }
            } else {
                // Short text with voltage e.g. "Cutoff high 3.5V"
                myLCD.print(F("Cutoff "));
                uint16_t tSwitchOffVoltageMillivolt;
                if (tCutoffLevel == CUTOFF_LEVEL_LOW) {
                    myLCD.print(F("low "));
                    tSwitchOffVoltageMillivolt = BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].CutoffVoltageMillivoltLow;
                } else { // CUTOFF_LEVEL_HIGH
                    myLCD.print(F("high"));
                    tSwitchOffVoltageMillivolt = BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].CutoffVoltageMillivoltHigh;
                }
                if (tSwitchOffVoltageMillivolt < 10000) {
                    myLCD.print(' ');
                }
                myLCD.print(((float) tSwitchOffVoltageMillivolt) / 1000, 1);
                myLCD.print('V');
            }
        }
    }
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
    if (sMeasurementState == STATE_DETECTING_BATTERY_OR_VOLTAGE) {
        LCDPrintVCC(); // to overwrite start of message
    }
}
#endif

/*
 * Prints state of cut off level
 */
void printCutoff() {
    if (!sOnlyPlotterOutput) {
        if (sOnlyLoggerFunctionality) {
            Serial.print(F("End logging below "));
            Serial.print(100 >> (sCutoffLevel + 1));
            Serial.println(F(" % of last current"));

        } else {
            auto tCutoffLevel = sCutoffLevel;
            if (tCutoffLevel == CUTOFF_LEVEL_ZERO) {
                Serial.println(F("Cut off at 50 mV"));
            } else {
                if (sMeasurementInfo.BatteryTypeIndex == TYPE_INDEX_NO_BATTERY) {
                    if (tCutoffLevel == CUTOFF_LEVEL_LOW) {
                        Serial.println(F("Cut off at low voltage. e.g. 3000 mV for Li-ion"));
                    } else { // CUTOFF_LEVEL_HIGH
                        Serial.println(F("Cut off at high voltage. e.g. 3450 mV for Li-ion"));
                    }
                } else {
                    if (tCutoffLevel == CUTOFF_LEVEL_LOW) {
                        Serial.print(F("Cut off at low voltage "));
                        Serial.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].CutoffVoltageMillivoltLow);
                    } else { // CUTOFF_LEVEL_HIGH
                        Serial.print(F("Cut off at high voltage "));
                        Serial.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].CutoffVoltageMillivoltHigh);
                    }
                    Serial.println(F(" mV"));
                }
            }
        }
    }
#if defined(USE_LCD)
    LCDPrintCutoff();
#endif
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
void printButtonUsageMessageLCD() {
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
 * Check for removed battery or logger voltage or current
 * @return true if Voltage is low or current is zero
 */
bool checkForVoltageOrCurrentRemoved() {

// check only if battery was inserted before
    if (sBatteryOrVoltageAndCurrentWasDetected
            && (sMeasurementInfo.Voltages.Battery.NoLoadMillivolt < NO_BATTERY_MILLIVOLT || sMeasurementInfo.Milliampere == 0)) {
        if (!sOnlyPlotterOutput) {
            Serial.print(F("Battery or voltage removing detected. U="));
            Serial.print(sMeasurementInfo.Voltages.Battery.NoLoadMillivolt);
            Serial.print(F(" mV I="));
            Serial.print(sMeasurementInfo.Milliampere);
            Serial.println(F(" mA"));
        }
        sBatteryOrVoltageAndCurrentWasDetected = false;
        return true;
    }
    return false;
}

/*
 * !!!Called in ISR Context!!!
 * !!! We can be called recursively, i.e. while waiting for 2 seconds we can be called for double press !!!
 * Because interrupts are still enabled, millis() is working here :-)
 *
 * Ignore all presses in mode STATE_SETUP_AND_READ_EEPROM and STATE_DETECTING_BATTERY_OR_VOLTAGE
 * Double click in 2 seconds stop measurement. -> Goes to state STATE_STOPPED
 *
 * Single press:
 * In mode STATE_DETECTING_BATTERY_OR_VOLTAGE:
 *      Cycle cut off level
 * In mode STATE_STOPPED:
 *      Switch to state STATE_DETECTING_BATTERY_OR_VOLTAGE
 * In mode STATE_INITIAL_SAMPLES:
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

    bool tIsDoublePress = startStopButton0AtPin2.checkForDoublePress(2 * EASY_BUTTON_DOUBLE_PRESS_DEFAULT_MILLIS);
    if (!sOnlyPlotterOutput) {
// Print should be done after checkForDoublePress() in order to not disturb the double press detection
        Serial.print(F("Button pressed, state="));
        printStateString(sMeasurementState);
        Serial.println();
    }

    if (tIsDoublePress && sMeasurementState != STATE_DETECTING_BATTERY_OR_VOLTAGE) {
        /*
         * Double press detected!
         * Go to STATE_STOPPED
         */
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Double press detected"));
        }
        switchToStateStoppedLCD('D');

    } else {
        /*
         * Single press here
         * Attention, this press can be the first press of a double press,
         * so we must wait 2 seconds and check for double press before processing single press
         */

        if (sMeasurementState == STATE_DETECTING_BATTERY_OR_VOLTAGE) {
            // Toggle cut off to normal, to low and to zero
            sCutoffLevel++;
            if (sCutoffLevel > CUTOFF_LEVEL_ZERO) {
                sCutoffLevel = CUTOFF_LEVEL_HIGH;
            }
            printCutoff();

        } else if (sMeasurementState == STATE_STORE_TO_EEPROM) {
            switchToStateStoppedLCD('B'); // no check for double press required here :-)

        } else {

            /*
             * Only state STATE_INITIAL_SAMPLES and STATE_STOPPED left here!
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
                sVoltageNoLoadIsDisplayedOnLCD = false;
#endif

            } else {
                // STATE_INITIAL_SAMPLES here
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
                switchToStateDetectingBatteryOrVoltage();

            } else if (sMeasurementState == STATE_INITIAL_SAMPLES) {
                // No stop requested during 2 seconds wait -> append to EEPROM
                switchToStateStoreToEEPROM();
                /*
                 * Store next sample in 60 seconds, because we assume double press directly after entering state STATE_INITIAL_SAMPLES
                 * Otherwise we would start the appended data with a short sampling period.
                 */
                sSampleCountForStoring = 0;
            }
        }
#if defined(LOCAL_DEBUG)
        if (!sOnlyPlotterOutput) {
            Serial.print(F("New state="));
            printStateString(sMeasurementState);
            Serial.println();
        }
#endif
    }
}

void setLoad(uint8_t aNewLoadState) {
    if (sMeasurementInfo.LoadState != aNewLoadState) {
        sMeasurementInfo.LoadState = aNewLoadState;

#if defined(LOCAL_TRACE)
        Serial.print(F("Set load to "));
#endif

        if (aNewLoadState == NO_LOAD) {
#if defined(LOCAL_TRACE)
            Serial.println(F("off"));
#endif
            digitalWrite(LOAD_LOW_PIN, LOW);    // disable 12 ohm load
            digitalWrite(LOAD_HIGH_PIN, LOW);    // disable 3 ohm load
        } else if (aNewLoadState == LOW_LOAD) {
#if defined(LOCAL_TRACE)
            Serial.println(F("low"));
#endif
            digitalWrite(LOAD_LOW_PIN, HIGH);    // enable 12 ohm load
            digitalWrite(LOAD_HIGH_PIN, LOW);    // disable 3 ohm load
        } else {
#if defined(LOCAL_TRACE)
            Serial.println(F("high"));
#endif
            digitalWrite(LOAD_LOW_PIN, LOW);    // disable 12 ohm load
            digitalWrite(LOAD_HIGH_PIN, HIGH);    // enable 3 ohm load
        }
    }
}

void getBatteryVoltageMillivolt() {

    uint16_t tInputVoltageRaw = getBatteryRawVoltage();
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

    if (sMeasurementInfo.LoadState == NO_LOAD) {
        sMeasurementInfo.Voltages.Battery.NoLoadMillivolt = tCurrentBatteryVoltageMillivolt;
    } else {
        sMeasurementInfo.Voltages.Battery.LoadMillivolt = tCurrentBatteryVoltageMillivolt;
    }

#if defined(LOCAL_DEBUG)
    if (!sOnlyPlotterOutput) {
        Serial.print(tCurrentBatteryVoltageMillivolt);
        Serial.println(F(" mV"));
    }
#endif
}

void setToLowVoltageRange() {
    sVoltageRangeIsLow = true;
    pinMode(VOLTAGE_RANGE_EXTENSION_PIN, INPUT);
    digitalWrite(VOLTAGE_RANGE_EXTENSION_PIN, LOW);
#if defined(LOCAL_DEBUG)
    if (!sOnlyPlotterOutput) {
        Serial.println(F(" -> switch to 2.2 V range"));
    }
#endif
}

void setToHighVoltageRange() {
    sVoltageRangeIsLow = false;
    pinMode(VOLTAGE_RANGE_EXTENSION_PIN, OUTPUT);
    digitalWrite(VOLTAGE_RANGE_EXTENSION_PIN, LOW);    // required???
#if defined(LOCAL_DEBUG)
    if (!sOnlyPlotterOutput) {
        Serial.println(F(" -> switch to 4.4 V range"));
    }
#endif
}
/*
 * Sets NoLoadMillivolt or LoadMillivolt
 * Provides automatic range switch between 2.2, 4.4 and 14 (up to 20 with 5V VCC) volt range
 * The ranges are realized by a divider with 100 kOhm and 100 kOhm -> 2.2 V range and a divider with 100 kOhm and 33.333 kOhm -> 4.4 v range
 * The 14 (20) volt range is realized by using the 4.4 volt range with VCC (of at least 3.5 volt) as reference.
 * With 5 volt VCC this range goes up to 20 volt resulting in a raw value of 4651
 * Does not affect the loads
 */
uint16_t getBatteryRawVoltage() {
    uint16_t tInputVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
    /*
     * Automatic range
     */
    if (sVoltageRangeIsLow && tInputVoltageRaw >= 0x3F0) { // 1008
    // switch to higher voltage range by activating the range extension resistor at pin A2
#if defined(LOCAL_DEBUG)
        if (!sOnlyPlotterOutput) {
            Serial.print(F("VoltageRaw="));
            Serial.print(tInputVoltageRaw);
        }
#endif
        setToHighVoltageRange(); // 4.4 V range
        // no wait, since last reading was same channel, same reference
        tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
    }
    if (!sVoltageRangeIsLow) {
        if (tInputVoltageRaw < (((0x3F0L * ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE) / ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE) - 0x10)) {
            // switch to lower voltage range at values below 488 by deactivating the range extension resistor at pin A2
#if defined(LOCAL_DEBUG)
            if (!sOnlyPlotterOutput) {
                Serial.print(F("VoltageRaw="));
                Serial.print(tInputVoltageRaw);
            }
#endif
            setToLowVoltageRange(); // 2.2 V range
            tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_VOLTAGE, INTERNAL);
        } else if (tInputVoltageRaw >= 0x3F0) {
            /*
             * Here we have 17 mV resolution
             * which leads to e.g. 0.3 ohm resolution at 9V and 60 mA
             */
#if defined(LOCAL_DEBUG)
            if (!sOnlyPlotterOutput) {
                Serial.print(F("Switch to "));
                Serial.print((sVCCVoltageMillivolt * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE) / 1000.0, 3);
                Serial.println(F(" V range"));
            }
#endif
// switch to highest voltage range by using VCC as reference
            uint16_t tReadoutFor1_1Reference = waitAndReadADCChannelWithReference(ADC_1_1_VOLT_CHANNEL_MUX, DEFAULT); // 225 at 5 volt VCC
            tInputVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_VOLTAGE, DEFAULT);
#if defined(LOCAL_TRACE)
            Serial.print(tInputVoltageRaw);
            Serial.print(F(" / "));
            Serial.println(tReadoutFor1_1Reference);
#endif
            // Adjust tInputVoltageRaw to a virtual 12.5 bit range based at voltage range high
            tInputVoltageRaw = (tInputVoltageRaw * 1023L) / tReadoutFor1_1Reference;
        }
    }
#if defined(LOCAL_TRACE)
    Serial.print(F("VoltageRaw="));
    Serial.print(tInputVoltageRaw);
    Serial.print(F(" isLow="));
    Serial.println(sVoltageRangeIsLow);
#endif
    return tInputVoltageRaw;
}

void addToCapacity() {
    if (sMeasurementInfo.Milliampere > 1) {
        // Capacity computation
        sMeasurementInfo.CapacityAccumulator += sMeasurementInfo.Milliampere;
        sMeasurementInfo.CapacityMilliampereHour = sMeasurementInfo.CapacityAccumulator
                / ((3600L * MILLIS_IN_ONE_SECOND) / SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS); // = / 3600 for 1 s sample period
    }
}

void clearLogger1SecondAccumulator() {
    sLogger1SecondRawVoltageAccumulator = 0;
    sLogger1SecondRawCurrentAccumulator = 0;
    sLogger1SecondRawSampleCount = 0;
    sLoggerMinimumRawVoltage = 0xFFFF;
    sLoggerMaximumRawVoltage = 0;
}

void clearLogger1MinuteAccumulator() {
    sLogger1MinuteRawVoltageAccumulator8ShiftRight = 0;
    sLogger1MinuteRawCurrentAccumulator = 0;
    sLogger1MinuteRawSampleCount = 0;
    // start every minute with new range selection
    setToLowVoltageRange();
    sLoggerADCVoltageReference = INTERNAL;
    sMeasurementInfo.Voltages.Logger.MaximumMillivolt = 0xFFFF;
    sMeasurementInfo.Voltages.Logger.MinimumMillivolt = 0;
}

/*
 * Compute milliampere and voltage from accumulator values and sample count
 * !!! We must be called only if sLogger1MinuteRawSampleCount == LOGGER_SAMPLE_FREQUENCY_HZ !!!
 */
void getLogger1SecondValues() {
    /*
     * Accumulate for minute
     */
    sLogger1MinuteRawVoltageAccumulator8ShiftRight += sLogger1SecondRawVoltageAccumulator >> 8;
    sLogger1MinuteRawSampleCount += LOGGER_SAMPLE_FREQUENCY_HZ;

    /*
     * Compute Milliampere and avoid overflow
     */
    sMeasurementInfo.Milliampere = ((((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) / 1023L) * sLogger1SecondRawCurrentAccumulator)
            / (LOGGER_SHUNT_RESISTOR_MILLIOHM * LOGGER_SAMPLE_FREQUENCY_HZ * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT));

    sLastMilliampereLowPassFiltered5 += ((sLastMilliampereLowPassFiltered5 - sLastMilliampereLowPassFiltered5) + (1 << 4)) >> 5; // 2.5 us, alpha = 1/32 0.03125, cutoff frequency 5.13 Hz @1kHz

    /*
     * Compute voltage and avoid overflow
     * >> 8 and * 4 in divisor are a fast and short way to divide by 1024
     */
    sMeasurementInfo.Voltages.Logger.AverageMillivolt =
            ((ADC_INTERNAL_REFERENCE_MILLIVOLT * (sLogger1SecondRawVoltageAccumulator >> 8))
                    / ((LOGGER_SAMPLE_FREQUENCY_HZ * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT * 4)
                            / ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE));
//
//    sMeasurementInfo.Voltages.Logger.MaximumMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE)
//            * (uint32_t) sLoggerMaximumRawVoltage) / 1023L);
    // divide by 1024 in short and fast. ((... >> 8) >> 2) does not work, it will be first converted to >> 10 and the compiled into a loop
    sMeasurementInfo.Voltages.Logger.MaximumMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE)
            * (uint32_t) sLoggerMaximumRawVoltage) >> 8);
    sMeasurementInfo.Voltages.Logger.MaximumMillivolt >>= 2;

    sMeasurementInfo.Voltages.Logger.MinimumMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE)
            * (uint32_t) sLoggerMinimumRawVoltage) >> 8);
    sMeasurementInfo.Voltages.Logger.MinimumMillivolt >>= 2;

#if defined(LOCAL_TRACE)
    Serial.print(F("cnt="));
    Serial.print(sLogger1SecondRawSampleCount);
    Serial.print(F(" Iacc="));
    Serial.print(sLogger1SecondRawCurrentAccumulator);
    Serial.print(' ');
    Serial.print(sMeasurementInfo.Milliampere);
    Serial.print(F(" mA, Uacc="));
    Serial.print(sLogger1SecondRawVoltageAccumulator);
    Serial.print(' ');
    Serial.print(sMeasurementInfo.Voltages.Battery.NoLoadMillivolt);
    Serial.print(F(" mV l="));
    Serial.println(sVoltageRangeIsLow);
#endif

    clearLogger1SecondAccumulator();
}
void getLogger1MinuteValues() {
    // avoid overflow
    sMeasurementInfo.Milliampere = ((((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) / 1023L)
            * (sLogger1MinuteRawCurrentAccumulator / LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT))
            / (sLogger1MinuteRawSampleCount * LOGGER_SHUNT_RESISTOR_MILLIOHM));

    /*
     * Compute voltage and avoid overflow
     * Instead of (sLogger1MinuteRawVoltageAccumulator8ShiftRight << 8) / 1024 which would give overflow
     * we do (sLogger1MinuteRawVoltageAccumulator8ShiftRight >> 8) and divide the divisor by 2^6 (64)
     */
    sMeasurementInfo.Voltages.Logger.AverageMillivolt = (ADC_INTERNAL_REFERENCE_MILLIVOLT
            * (sLogger1MinuteRawVoltageAccumulator8ShiftRight >> 8))
            / (((uint32_t) sLogger1MinuteRawSampleCount * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT)
                    / (ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE * 64));

#if defined(LOCAL_TRACE)
    Serial.print(F("cnt="));
    Serial.print(sLogger1MinuteRawSampleCount);
    Serial.print(F(" Iacc="));
    Serial.print(sLogger1MinuteRawCurrentAccumulator);
    Serial.print(' ');
    Serial.print(sMeasurementInfo.Milliampere);
    Serial.print(F(" mA, Uacc="));
    Serial.print(sLogger1MinuteRawVoltageAccumulator8ShiftRight);
    Serial.print(' ');
    Serial.println(sMeasurementInfo.Voltages.Battery.NoLoadMillivolt);
    Serial.print(F(" mV l="));
    Serial.println(sVoltageRangeIsLow);
#endif

    clearLogger1MinuteAccumulator();
}

/*
 * Maximal current for a 0.2 ohm shunt resistor is 5.5 A, and resolution is 5.4 mA.
 */
void getLoggerCurrent() {
    uint32_t tRawCurrentValue = readADCChannelMultiSamplesWithReferenceAndPrescaler(ADC_CHANNEL_LOGGER_CURRENT, INTERNAL,
    ADC_PRESCALE32, LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT);
    /*
     * Compute Milliampere and avoid overflow
     */
    sMeasurementInfo.Milliampere = ((((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) / 1023L) * tRawCurrentValue)
            / (LOGGER_SHUNT_RESISTOR_MILLIOHM * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT));
}

/*
 * Sampling takes 40.5 ms. For Voltage > 4.4V it takes 48.5 ms
 */
void accumulateLoggerValues() {
    if (sMeasurementState != STATE_DETECTING_BATTERY_OR_VOLTAGE) {
        /*
         * Read 769 current values in 20ms if not stopped
         */
        digitalWrite(LED_BUILTIN, HIGH);
// switch channel and reference
#if defined(LOCAL_TRACE)
//    uint8_t tOldADMUX =
#endif
        checkAndWaitForReferenceAndChannelToSwitch(ADC_CHANNEL_LOGGER_CURRENT, INTERNAL); //
        /*
         * maximum value of readADCChannelWithReferenceAndPrescalerMultiSamples(...769) is 800 000
         * So we can have 5 k of it in a 32 bit integer
         * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT = 769 for ADC_PRESCALE32 and 20 ms (50 Hz)
         */
        uint32_t tRawCurrentValue = readADCChannelMultiSamples(ADC_PRESCALE32, LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT);
        digitalWrite(LED_BUILTIN, LOW);
        sLogger1SecondRawCurrentAccumulator += tRawCurrentValue;
        sLogger1MinuteRawCurrentAccumulator += tRawCurrentValue;
#if defined(LOCAL_TRACE)
        if (!sOnlyPlotterOutput) {
//            Serial.print(F("OldADMUX=0x"));
//            Serial.print(tOldADMUX, HEX);
            Serial.print(F(" AVGRawCurrent="));
            Serial.println(tRawCurrentValue / LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT);
        }
#endif
    }

    /*
     * Read voltage always
     * Set external voltage divider and ADC reference.
     * For voltages > 4.4 V, reference must be switched to VCC and back at next current measurement.
     * tRawVoltageValue is normalized for a value of 1023 at 4.4 V.
     */
    waitAndReadADCChannelWithReference(ADC_CHANNEL_VOLTAGE, sLoggerADCVoltageReference);

    digitalWrite(LED_BUILTIN, HIGH);
    uint32_t tRawVoltageValue = 0;
    uint16_t tInputMinimumRawVoltage = 0xFFFF;
    uint16_t tInputMaximumRawVoltage = 0;
    ADCSRB = 0; // Free running mode. Only active if ADATE is set to 1.
    // ADSC-StartConversion ADATE-AutoTriggerEnable ADIF-Reset Interrupt Flag
    ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIF) | ADC_PRESCALE32);

    for (uint16_t i = 0; i < LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT; i++) {
        /*
         * wait for free running conversion to finish.
         * Do not wait for ADSC here, since ADSC is only low for 1 ADC Clock cycle on free running conversion.
         */
        loop_until_bit_is_set(ADCSRA, ADIF);

        ADCSRA |= _BV(ADIF); // clear bit to enable recognizing next conversion has finished
        // Add value
        uint16_t tInputRawVoltage = ADCL | (ADCH << 8);
        if (tInputRawVoltage >= 0x3F0) { // 1008
            if (sVoltageRangeIsLow) {
                // switch to higher voltage range by activating the range extension resistor at pin A2
                setToHighVoltageRange();
            } else if (sLoggerADCVoltageReference == INTERNAL) {
                // here we have the 4.4 V range and must switch to VCC reference
                sLoggerADCVoltageReference = DEFAULT;
                checkAndWaitForReferenceAndChannelToSwitch(ADC_CHANNEL_VOLTAGE, DEFAULT);
#if defined(LOCAL_DEBUG)
                if (!sOnlyPlotterOutput) {
                    Serial.print(F("Switch to "));
                    Serial.print((sVCCVoltageMillivolt * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE) / 1000.0, 3);
                    Serial.println(F(" V range"));
                }
#endif
            } else {
                // Voltage once too high for 4.4 V divider and VCC as reference
#if defined(LOCAL_DEBUG)
                if (!sOnlyPlotterOutput) {
                    Serial.println(F("overvoltage"));
                }
#endif
                tRawVoltageValue = 0x3FF * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT;
                break;
            }
            // Start a full new loop
            i = 0;
            tRawVoltageValue = 0;
            tInputRawVoltage = 0;
        }
        tRawVoltageValue += tInputRawVoltage;
        if (tInputMinimumRawVoltage > tInputRawVoltage) {
            tInputMinimumRawVoltage = tInputRawVoltage;
        }
        if (tInputMaximumRawVoltage < tInputRawVoltage) {
            tInputMaximumRawVoltage = tInputRawVoltage;
        }
    }
    ADCSRA &= ~_BV(ADATE); // Disable auto-triggering (free running mode)

    // normalize to 1023 at 4.4 V
    if (sVoltageRangeIsLow) {
        tRawVoltageValue /= (ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE / ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE);
        tInputMaximumRawVoltage /= (ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE / ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE);
        tInputMinimumRawVoltage /= (ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE / ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE);
    } else if (sLoggerADCVoltageReference == DEFAULT) {
        // Adjust tInputVoltageRaw to a virtual 12.5 bit range -> maximum value is 5000 for 20 V
        uint16_t tReadoutFor1_1Reference = waitAndReadADCChannelWithReference(ADC_1_1_VOLT_CHANNEL_MUX, DEFAULT); // 225 at 5 volt VCC
        tRawVoltageValue = (tRawVoltageValue * 1023L) / tReadoutFor1_1Reference;
        tInputMaximumRawVoltage = (tInputMaximumRawVoltage * 1023L) / tReadoutFor1_1Reference;
        tInputMinimumRawVoltage = (tInputMinimumRawVoltage * 1023L) / tReadoutFor1_1Reference;
    }

    sLogger1SecondRawVoltageAccumulator += tRawVoltageValue;
    if (sLoggerMaximumRawVoltage < tInputMaximumRawVoltage) {
        sLoggerMaximumRawVoltage = tInputMaximumRawVoltage;
    }
    if (sLoggerMinimumRawVoltage > tInputMinimumRawVoltage) {
        sLoggerMinimumRawVoltage = tInputMinimumRawVoltage;
    }

#if defined(LOCAL_TRACE)
    if (!sOnlyPlotterOutput) {
//        Serial.print(F("ADMUX=0x"));
//        Serial.print(ADMUX, HEX);
//        Serial.print(F(" Ref="));
//        Serial.print(sLoggerADCVoltageReference); // 3 = INTERNAL
        Serial.print(F(" AVGRawVoltage="));
        Serial.println(tRawVoltageValue / LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT);
    }
#endif

    sLogger1SecondRawSampleCount++;
    digitalWrite(LED_BUILTIN, LOW);
}

void getBatteryCurrent() {
    uint16_t tShuntVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_CURRENT, INTERNAL);
    sMeasurementInfo.Milliampere = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) * tShuntVoltageRaw)
            / (1023L * ESR_SHUNT_RESISTOR_MILLIOHM));
}

/*
 * Assumes that load is activated before called
 */
void getBatteryValues() {
// Do it before deactivating the load
    getBatteryCurrent();
    getBatteryVoltageMillivolt();    // get current battery load voltage (no load in case of stopped)

    if (sMeasurementState == STATE_STOPPED) return; // thats all if stopped :-)

// Deactivate load and wait for voltage to settle
// During the no load period switch on the LED
    setLoad(NO_LOAD);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].LoadSwitchSettleTimeMillis);
    getBatteryVoltageMillivolt();    // get current battery no load voltage
// restore original load state
    setLoad(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].LoadType);
    digitalWrite(LED_BUILTIN, LOW);

    sMeasurementInfo.sESRDeltaMillivolt = sMeasurementInfo.Voltages.Battery.NoLoadMillivolt
            - sMeasurementInfo.Voltages.Battery.LoadMillivolt;

    if (sMeasurementInfo.Milliampere > 1) {
        /*
         * Compute sESRAverage
         * Shift history array and insert current value
         */
        uint8_t tESRAverageHistoryCounter = 1; // we always add sESRHistory[0]
        uint32_t tESRAverageAccumulator = 0;
        for (uint_fast8_t i = HISTORY_SIZE_FOR_ESR_AVERAGE - 1; i > 0; --i) {
            if (sESRHistory[i - 1] != 0) {
                // shift i-1 to i and add to average
#if defined(LOCAL_TRACE)
                Serial.print(sESRHistory[i - 1]);
                Serial.print('+');
#endif
                tESRAverageHistoryCounter++; // count only valid entries
                tESRAverageAccumulator += sESRHistory[i - 1];
                sESRHistory[i] = sESRHistory[i - 1];
            }
        }
        // insert current value
        uint32_t tESRMilliohm = (sMeasurementInfo.sESRDeltaMillivolt * 1000L) / sMeasurementInfo.Milliampere;
        if (tESRMilliohm > __UINT16_MAX__) {
            sESRHistory[0] = __UINT16_MAX__; // indicate overflow
        } else {
            sESRHistory[0] = tESRMilliohm;
        }

        tESRAverageAccumulator += sESRHistory[0];
        sMeasurementInfo.ESRMilliohm = (tESRAverageAccumulator + (tESRAverageHistoryCounter / 2)) / tESRAverageHistoryCounter;

#if defined(LOCAL_TRACE)
        Serial.print(sESRHistory[0]);
        Serial.print('/');
        Serial.print(tESRAverageHistoryCounter);
        Serial.print('=');
        Serial.print(sMeasurementInfo.ESRMilliohm);
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
        sCurrentLoadResistorHistory[0] = (sMeasurementInfo.Voltages.Battery.LoadMillivolt * 1000L / sMeasurementInfo.Milliampere);
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
 * Switch to state STATE_STOPPED if stop condition met
 */
void checkAndHandleStopConditionLCD() {
    bool tStopConditionIsMet = false;
    if (sOnlyLoggerFunctionality) {
        if (sMeasurementInfo.Milliampere < (sLastMilliampereLowPassFiltered5 >> (sCutoffLevel + 1))) {
            /*
             * Switch off current condition for logger met
             */
            if (!sOnlyPlotterOutput) {
                Serial.print(F("Switch off current percentage "));
                Serial.print(100 >> (sCutoffLevel + 1));
                Serial.print(F(" % mA of "));
                Serial.print(sLastMilliampereLowPassFiltered5);
                Serial.print(F(" mA reached, I="));
                Serial.print(sMeasurementInfo.Milliampere);
                Serial.print(F(" mA, capacity="));
                Serial.print(sMeasurementInfo.CapacityMilliampereHour);
                Serial.println(F(" mAh"));
            }
            tStopConditionIsMet = true;
        }

    } else {
        uint16_t tSwitchOffVoltageMillivolt;
        if (sCutoffLevel == CUTOFF_LEVEL_ZERO) {
            tSwitchOffVoltageMillivolt = DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT;
        } else if (sCutoffLevel == CUTOFF_LEVEL_LOW) {
            tSwitchOffVoltageMillivolt = BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].CutoffVoltageMillivoltLow;
        } else { // CUTOFF_LEVEL_HIGH
            tSwitchOffVoltageMillivolt = BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].CutoffVoltageMillivoltHigh;
        }
        if (sMeasurementInfo.Voltages.Battery.NoLoadMillivolt < tSwitchOffVoltageMillivolt) {
            /*
             * Switch off voltage condition for battery met
             */
            setLoad(NO_LOAD);
            if (!sOnlyPlotterOutput) {
                Serial.print(F("Switch off voltage "));
                Serial.print(tSwitchOffVoltageMillivolt);
                Serial.print(F(" mV reached, capacity="));
                Serial.print(sMeasurementInfo.CapacityMilliampereHour);
                Serial.println(F(" mAh"));
            }
            tStopConditionIsMet = true;
        }
    }
    if (tStopConditionIsMet && sMeasurementState == STATE_STORE_TO_EEPROM) {
        switchToStateStoppedLCD('-');
#if defined(USE_LCD)
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS); // show "stopped"
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
uint8_t getBatteryTypeIndex(uint16_t aBatteryVoltageMillivolt) {

// scan all threshold voltage of all battery types
    for (uint_fast8_t i = 0; i < sizeof(BatteryTypeInfoArray) / sizeof(BatteryTypeInfoStruct) - 1; i++) {
        if (aBatteryVoltageMillivolt < BatteryTypeInfoArray[i].DetectionThresholdVoltageMillivolt) {
#if defined(LOCAL_DEBUG)
            Serial.print(F(" Battery index="));
            Serial.print(i);
            Serial.print(F(" BatteryVoltageMillivolt="));
            Serial.print(aBatteryVoltageMillivolt);
            Serial.print(F(" SwitchOffVoltageMillivolt="));
            Serial.println(BatteryTypeInfoArray[i].CutoffVoltageMillivoltHigh);
#endif
            return i;
        }
    }
// High voltage is detected
    return sizeof(BatteryTypeInfoArray) / sizeof(BatteryTypeInfoStruct) - 1;
}

/*
 * Disables the load, measures the voltage to detecting battery type and enables the load if battery detected
 * For logger it also measures the current for detection
 * @return true, if battery or logger voltage and current detected
 */
void detectBatteryOrLoggerVoltageAndCurrentLCD() {
    setLoad(NO_LOAD);
    getBatteryVoltageMillivolt();

    if (sOnlyLoggerFunctionality) {
        getLoggerCurrent();
        if (sMeasurementInfo.Voltages.Battery.NoLoadMillivolt > NO_BATTERY_MILLIVOLT
                && sMeasurementInfo.Milliampere >= NO_LOGGER_MILLAMPERE) {
            // external voltage and current found
            sBatteryOrVoltageAndCurrentWasDetected = true;
            if (!sOnlyPlotterOutput) {
                Serial.print(F("Found U="));
                Serial.print(sMeasurementInfo.Voltages.Battery.NoLoadMillivolt);
                Serial.print(F(" mV, I="));
                Serial.print(sMeasurementInfo.Milliampere);
                Serial.println(F(" mA"));
            }
        }
#if defined(USE_LCD)
        else if (sMeasurementInfo.Voltages.Battery.NoLoadMillivolt <= NO_BATTERY_MILLIVOLT
                && sMeasurementInfo.Milliampere < NO_LOGGER_MILLAMPERE) {
            myLCD.setCursor(7, 0);
            myLCD.print(F("No U or I"));
        }
#endif
        // else -> we have voltage or current attached.
        return;
    }

    /*
     * Battery type detection here
     */
    sMeasurementInfo.BatteryTypeIndex = getBatteryTypeIndex(sMeasurementInfo.Voltages.Battery.NoLoadMillivolt);
    sMeasurementInfo.Milliampere = 0; // avoid display old current value

    if (sMeasurementInfo.BatteryTypeIndex == TYPE_INDEX_NO_BATTERY) {
#if defined(USE_LCD)
        myLCD.setCursor(7, 0);
        myLCD.print(F(" No batt."));
#endif
    } else {
        // print values
        printVoltageNoLoadMillivoltWithTrailingSpaceLCD();
        if (!sOnlyPlotterOutput) {
            Serial.print(F(" => "));
            Serial.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].TypeName);
            Serial.println(F(" found"));
        }

#if defined(USE_LCD)
        myLCD.setCursor(0, 1);
        myLCD.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].TypeName);
        myLCD.print(F(" found"));
// The current battery voltage is displayed, so clear "No batt." message selectively
        myLCD.setCursor(7, 0);
        myLCD.print(F("         "));
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
        LCDClearLine(1);
#endif
        sBatteryOrVoltageAndCurrentWasDetected = true;
    }
}

/*
 * Called exclusively from setup() after readAndProcessEEPROMData()
 */
void printStoredDataLCD() {
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    myLCD.print(getVCCVoltage(), 1);
    myLCD.print(F("V Stored data"));
#endif
    /*
     * Print battery values, and use state STATE_SETUP_AND_READ_EEPROM for formatting
     * "0.061o l 1200mAh" using sMeasurementInfo.ESRMilliohm
     */
    printMeasurementValuesLCD();
#if defined(USE_LCD)
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
}

void printVoltageNoLoadMillivoltWithTrailingSpaceLCD() {
    uint16_t tVoltageNoLoadMillivolt = sMeasurementInfo.Voltages.Battery.NoLoadMillivolt; // saves 12 bytes programming space
    if (!sOnlyPlotterOutput) {
        printMillisValueAsFloat(tVoltageNoLoadMillivolt);
        Serial.print(F(" V ")); // for optional following mA
    }
    sLastVoltageNoLoadMillivoltForPrintAndCountdown = tVoltageNoLoadMillivolt;
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    LCDPrintAsFloatWith3Decimals(tVoltageNoLoadMillivolt);
    myLCD.print(F("V "));
    sVoltageNoLoadIsDisplayedOnLCD = true;
// cursor is now at 7, 0
#endif

}

/*
 * Print no newline
 */
void printCapacity5Digits() {
    if (!sOnlyPlotterOutput) {
        Serial.print(F("capacity="));
        Serial.print(sMeasurementInfo.CapacityMilliampereHour);
        Serial.print(F(" mAh"));
    }
#if defined(USE_LCD)
    char tString[6];
    sprintf_P(tString, PSTR("%5u"), sMeasurementInfo.CapacityMilliampereHour);
    myLCD.print(tString);
    myLCD.print(F("mAh"));
#endif
}

/*
 * Print no newline
 */
void printMilliampere4DigitsLCD() {
    if (!sOnlyPlotterOutput) {
        Serial.print(sMeasurementInfo.Milliampere);
        Serial.print(F(" mA "));
        if (!sOnlyLoggerFunctionality) {
            Serial.print(F("at "));
            printMillisValueAsFloat(sCurrentLoadResistorHistory[0]);
            Serial.print(F(" ohm, "));
        }
    }
#if defined(USE_LCD)
    myLCD.setCursor(10, 0);
    char tString[6];
    sprintf_P(tString, PSTR("%4u"), sMeasurementInfo.Milliampere);
    myLCD.print(tString);
    myLCD.print(F("mA"));
#endif
}

/*
 * Evaluates sMeasurementState and prints:
 *   - sMeasurementInfo.Voltages.Battery.NoLoadMillivolt
 *   - sMeasurementInfo.Milliampere
 *   - sMeasurementInfo.ESRMilliohm
 *   - optional sESRDeltaMillivolt or capacity
 * to Serial and LCD
 *
 * STATE_INITIAL_SAMPLES:
 * "4.030 V, 27 s  329 mA at 11.896 ohm, ESR=0.329 ohm, 0.108 V"
 *  0   4   8   C  F
 * "4.030V 18  329mA" printing down counter
 * "0.061o l  0.128V" using current ESR from sESRHistory[0]
 *
 * STATE_STORE_TO_EEPROM:
 * "4.030 V,  329 mA at 11.949 ohm, ESR=0.329 ohm, capacity=1200 mAh
 *  0   4   8   C  F
 * "4.030V 312 329mA" printing EEPROM array index
 * "0.392o l 1200mAh" using sMeasurementInfo.ESRMilliohm
 *
 * Called with states STATE_SETUP_AND_READ_EEPROM, STATE_INITIAL_SAMPLES and STATE_STORE_TO_EEPROM
 * State STATE_DETECTING_BATTERY_OR_VOLTAGE is handled in loop()
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
void printMeasurementValuesLCD() {
    sInLCDPrint = true; // disable printing by button handler
    uint8_t tMeasurementState = sMeasurementState; // Because sMeasurementState is volatile
    if (tMeasurementState != STATE_SETUP_AND_READ_EEPROM) {
        /***********************************************************************************
         * First row only for state STATE_INITIAL_SAMPLES or STATE_STORE_TO_EEPROM
         ***********************************************************************************/
        /*
         * Print no load voltage here
         */
        auto tLastVoltageNoLoadMillivoltForPrintAndCountdown = sLastVoltageNoLoadMillivoltForPrintAndCountdown; // Must be before printVoltageNoLoadMillivoltWithTrailingSpaceLCD()
        printVoltageNoLoadMillivoltWithTrailingSpaceLCD();
        if (!sOnlyPlotterOutput) {
            // 4.094 V,  334 mA at 11.949 ohm, ESR=0.334 ohm, capacity=3501 mAh
            Serial.print(F(", "));
        }

        if (tMeasurementState == STATE_INITIAL_SAMPLES) {
            /*
             * Print down counter for STATE_INITIAL_SAMPLES
             * Count down only if we do not have a rapid voltage decrease or we are in logger mode
             */
            if (sOnlyLoggerFunctionality
                    || sMeasurementInfo.Voltages.Battery.NoLoadMillivolt >= tLastVoltageNoLoadMillivoltForPrintAndCountdown
                    || (tLastVoltageNoLoadMillivoltForPrintAndCountdown - sMeasurementInfo.Voltages.Battery.NoLoadMillivolt) < 4) {
                sNumbersOfInitialSamplesToGo--;
            }

            uint8_t tNumbersOfInitialSamplesToGo = sNumbersOfInitialSamplesToGo;
            if (!sOnlyPlotterOutput) {
                Serial.print(tNumbersOfInitialSamplesToGo);
                Serial.print(F(" s, ")); // seconds until discharging
            }
            if (tNumbersOfInitialSamplesToGo < 10) {
#if defined(USE_LCD)
                myLCD.print(' '); // padding space for count
#endif
                tone(BUZZER_PIN, 2000, 40); // costs 1524 bytes code space
            }
#if defined(USE_LCD)
            myLCD.print(tNumbersOfInitialSamplesToGo);
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

        /*
         * Print Current
         */
        printMilliampere4DigitsLCD();
    }

    /**********************
     * Start of second row
     **********************/
    if (sMeasurementState != STATE_DETECTING_BATTERY_OR_VOLTAGE) {
        /*
         * STATE_SETUP_AND_READ_EEPROM + STATE_STORE_TO_EEPROM: "0.061o l 1200mAh" using sMeasurementInfo.ESRMilliohm
         * STATE_INITIAL_SAMPLES:                               "0.061o l  0.128V" using current ESR from sESRHistory[0]
         */
        uint32_t tMilliohm; // Compiler complains about initialize variable, which is wrong
        if (sOnlyLoggerFunctionality) {
            if (!sOnlyPlotterOutput) {
                Serial.print(F(" Min="));
                printMillisValueAsFloat(sMeasurementInfo.Voltages.Logger.MinimumMillivolt);
                Serial.print(F(" V, Avg="));
                printMillisValueAsFloat(sMeasurementInfo.Voltages.Logger.AverageMillivolt);
                Serial.print(F(" V, Max="));
                printMillisValueAsFloat(sMeasurementInfo.Voltages.Logger.MaximumMillivolt);
                Serial.print(F(" V "));
            }
        } else {
            /*
             * Print ESR
             */
            if (tMeasurementState == STATE_INITIAL_SAMPLES && sMeasurementInfo.Milliampere != 0) {
                tMilliohm = sESRHistory[0];
            } else {
                tMilliohm = sMeasurementInfo.ESRMilliohm;
            }

            if (!sOnlyPlotterOutput) {
                Serial.print(F("ESR="));
                if (tMilliohm == __UINT16_MAX__) {
                    /*
                     * No recent current measurement -> show old ESR
                     */
                    Serial.print(F("overflow, "));
                } else {
                    printMillisValueAsFloat(tMilliohm);
                    Serial.print(F(" ohm, "));
                }
            }
#if defined(USE_LCD)
            myLCD.setCursor(0, 1);
            if (sOnlyLoggerFunctionality) {
                LCDPrintAsFloatWith3Decimals(sMeasurementInfo.Voltages.Logger.MaximumMillivolt);
                myLCD.print(F("V"));
            } else {
                if (tMilliohm == __UINT16_MAX__) {
                    myLCD.print(F("99.99")); // Overflow
                } else if (tMilliohm < 10000) {
                    myLCD.print(((float) (tMilliohm)) / 1000, 3);
                } else {
                    myLCD.print(((float) (tMilliohm)) / 1000, 2);
                }
                myLCD.print(F("\xF4 ")); // Ohm symbol
            }
#endif
        }

        /*
         * Print cut off level
         */
#if defined(USE_LCD)
        myLCD.setCursor(7, 1); // This avoids problems with values >= 10 ohm
        myLCD.print(getCutoffLevelAsCharacter());
#endif

        /*
         * Print voltage difference or capacity
         */
        if (tMeasurementState == STATE_INITIAL_SAMPLES) {
            if (!sOnlyLoggerFunctionality) {
                /*
                 * Print voltage difference between no load and load used for ESR computation
                 */
                uint16_t tESRDeltaMillivolt = sMeasurementInfo.sESRDeltaMillivolt; // saves 4 bytes programming space
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
        } else {

            /*
             * Print capacity
             */
#if defined(USE_LCD)
            myLCD.setCursor(8, 1); // This avoids problems with values >= 10 ohm
#endif
            printCapacity5Digits();
        }
    }

    /*
     * Postprocessing
     */
    printlnIfNotPlotterOutput();
    checkForDelayedButtorProcessing();
}
#pragma GCC diagnostic pop

void printMillisValueAsFloat(uint16_t aValueInMillis) {
    Serial.print(((float) (aValueInMillis)) / 1000, 3);
}

/*
 * Just clear the complete EEPROM
 */
void updateEEPROMTo_FF() {
    if (!sOnlyPlotterOutput) {
        Serial.println(F("Clear EEPROM"));
    }
    for (unsigned int i = 0; i < E2END; ++i) {
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
 * No compression, only clip to 8 bit range
 */
int8_t clipDelta(int16_t aDelta) {
    if (aDelta > __INT8_MAX__) {
        return __INT8_MAX__;
    } else if (aDelta < -128) {
        return -128;
    }
    return aDelta;
}

/*
 * Compute compressed delta
 * upper 4 bit store the first value (between -8 and 7), lower 4 bit store the second value
 * @param aDelta        The delta to process
 * @param *aDeltaTemp   Storage for the upper 4 bit delta, which cannot directly be written to EEPROM
 * @return clipped aDelta | aDelta which is stored
 */
int16_t setDeltas(int16_t aDelta, uint8_t *aDeltaTemp) {
    if (!sOnlyPlotterOutput) {
        Serial.print(' ');
        Serial.print(aDelta);
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
    } else {
// upper 4 bit store the first value (between -8 and 7), lower 4 bit store the second value
        tDeltaToStore = *aDeltaTemp | tDelta;
    }
    *aDeltaTemp = tDeltaToStore;
    if (!sOnlyPlotterOutput) {
        Serial.print(F("->0x"));
        Serial.print(tDeltaToStore, HEX);
    }
    return aDelta;
}

/*
 * Store values to EEPROM as 4 bit deltas between sMeasurementInfo and ValuesForDeltaStorage and write them to EEPROM every second call
 * Upper 4 bit store the first value, lower 4 bit store the second value
 */
void storeBatteryValuesToEEPROM(uint16_t aVoltageNoLoadMillivolt, uint16_t aMilliampere, uint16_t aMilliohm) {
    if (ValuesForDeltaStorage.DeltaArrayIndex < 0) {

        updateEEPROMTo_FF(); // this may last one or two seconds

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
        StartValues.CutoffLevel = sCutoffLevel;
        StartValues.LoadResistorMilliohm = sCurrentLoadResistorAverage;
        StartValues.CapacityMilliampereHour = 0; // Capacity is written at the end or computed while reading
        eeprom_update_block(&StartValues, &EEPROMStartValues, sizeof(EEPROMStartValues));
#if defined(LOCAL_DEBUG)
        dumpEEPROM((uint8_t*) &EEPROMStartValues, 1);
#endif
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Store initial values to EEPROM"));
        }

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
                    Serial.println(F("Store 8 bit deltas:"));
                }

                /*
                 * Append value to delta values array
                 */
                int16_t tVoltageDelta = aVoltageNoLoadMillivolt - ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt;
                tVoltageDelta = clipDelta(tVoltageDelta);
                ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt += tVoltageDelta;
                ValuesForDeltaStorage.tempDeltas.uncompressed.DeltaMillivolt = tVoltageDelta;

                int16_t tMilliampereDelta = aMilliampere - ValuesForDeltaStorage.lastStoredMilliampere;
                tMilliampereDelta = clipDelta(tMilliampereDelta);
                ValuesForDeltaStorage.lastStoredMilliampere += tMilliampereDelta;
                ValuesForDeltaStorage.tempDeltas.uncompressed.DeltaMilliampere = tMilliampereDelta;

                int16_t tMilliohmDelta = aMilliohm - ValuesForDeltaStorage.lastStoredMilliohm;
                tMilliohmDelta = clipDelta(tMilliohmDelta);
                ValuesForDeltaStorage.lastStoredMilliohm += tMilliohmDelta;
                ValuesForDeltaStorage.tempDeltas.uncompressed.DeltaESRMilliohm = tMilliohmDelta;
                eeprom_update_block(&ValuesForDeltaStorage.tempDeltas, &EEPROMDataArray[ValuesForDeltaStorage.DeltaArrayIndex],
                        sizeof(ValuesForDeltaStorage.tempDeltas));

#if defined(LOCAL_DEBUG)
                Serial.print(F("EEPROM values="));
                Serial.print(ValuesForDeltaStorage.tempDeltas.uncompressed.DeltaMillivolt);
                Serial.print(F("|0x"));
                Serial.print(ValuesForDeltaStorage.tempDeltas.compressed.DeltaMillivolt, HEX);
                Serial.print(' ');
                Serial.print(ValuesForDeltaStorage.tempDeltas.uncompressed.DeltaMilliampere);
                Serial.print(F("|0x"));
                Serial.print(ValuesForDeltaStorage.tempDeltas.compressed.DeltaMilliampere, HEX);
                Serial.print(' ');
                Serial.print(ValuesForDeltaStorage.tempDeltas.uncompressed.DeltaESRMilliohm);
                Serial.print(F("|0x"));
                Serial.print(ValuesForDeltaStorage.tempDeltas.compressed.DeltaESRMilliohm, HEX);
//                Serial.print(F(" CapAccu="));
//                Serial.print(tCapacityAccumulator);
                Serial.println();
#endif
#if defined(LOCAL_TRACE)
                // dump 2 lines containing the 3 byte data
                dumpEEPROM((uint8_t*) ((uint16_t) &EEPROMDataArray[ValuesForDeltaStorage.DeltaArrayIndex] & 0xFFF0), 2);
#endif

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
                uint8_t *tEEPROMPointer = reinterpret_cast<uint8_t*>(&EEPROMDataArray[ValuesForDeltaStorage.DeltaArrayIndex]);
                for (unsigned int i = sizeof(EEPROMDataUnion) * ValuesForDeltaStorage.DeltaArrayIndex;
                        i < sizeof(EEPROMDataUnion) * MAX_NUMBER_OF_SAMPLES; ++i) {
                    eeprom_update_byte(tEEPROMPointer++, 0xFF);
                }

                /*
                 * Set compression flag
                 */
                StartValues.compressionFlag = FLAG_COMPRESSION;
                eeprom_update_byte(&EEPROMStartValues.compressionFlag, FLAG_COMPRESSION); // store compression flag in EEPROM
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

                /*
                 * Append value to delta values array
                 */
                int16_t tVoltageDelta = aVoltageNoLoadMillivolt - ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt;
                tVoltageDelta = setDeltas(tVoltageDelta, &ValuesForDeltaStorage.tempDeltas.compressed.DeltaMillivolt);
                ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt += tVoltageDelta;

                int16_t tMilliampereDelta = aMilliampere - ValuesForDeltaStorage.lastStoredMilliampere;
                tMilliampereDelta = setDeltas(tMilliampereDelta, &ValuesForDeltaStorage.tempDeltas.compressed.DeltaMilliampere);
                ValuesForDeltaStorage.lastStoredMilliampere += tMilliampereDelta;

                int16_t tMilliohmDelta = aMilliohm - ValuesForDeltaStorage.lastStoredMilliohm;
                tMilliohmDelta = setDeltas(tMilliohmDelta, &ValuesForDeltaStorage.tempDeltas.compressed.DeltaESRMilliohm);
                ValuesForDeltaStorage.lastStoredMilliohm += tMilliohmDelta;
                printlnIfNotPlotterOutput();

                if (!sOnlyPlotterOutput) {
                    if (ValuesForDeltaStorage.tempDeltaIsEmpty) {
                        Serial.println(F("store to EEPROM compress buffer"));
                        ValuesForDeltaStorage.tempDeltaIsEmpty = false;

                    } else {
                        eeprom_update_block(&ValuesForDeltaStorage.tempDeltas,
                                &EEPROMDataArray[ValuesForDeltaStorage.DeltaArrayIndex], sizeof(ValuesForDeltaStorage.tempDeltas));
                        Serial.print(F("store to EEPROM at index "));
                        Serial.print(ValuesForDeltaStorage.DeltaArrayIndex);

                        /*
                         * control read to verify written data
                         */

//                        EEPROMDataUnion tEEPROMData;
//                        eeprom_read_block(&tEEPROMData, &EEPROMDataArray[ValuesForDeltaStorage.DeltaArrayIndex],
//                                sizeof(tEEPROMData));
//                        if (ValuesForDeltaStorage.tempDeltas.DeltaMillivolt != tEEPROMData.DeltaMillivolt
//                                || ValuesForDeltaStorage.tempDeltas.DeltaMilliampere != tEEPROMData.DeltaMilliampere
//                                || ValuesForDeltaStorage.tempDeltas.DeltaESRMilliohm != tEEPROMData.DeltaESRMilliohm) {
//                            // Yes, I have seen this (starting with index 4 6 times 0xFF for current). Maybe undervoltage while powered by battery.
//                            tone(BUZZER_PIN, NOTE_C7, 20);
//                            delay(40);
//                            tone(BUZZER_PIN, NOTE_C6, 20);
//                            delay(40);
//                            tone(BUZZER_PIN, NOTE_C7, 20);
//                        }
                        // requires 2 bytes less program space compared with 3 fixed comparisons above
//                        uint8_t * tEEPROMPointer = reinterpret_cast<uint8_t*>(&EEPROMDataArray[ValuesForDeltaStorage.DeltaArrayIndex]);
                        uint8_t *tRAMPointer = reinterpret_cast<uint8_t*>(&ValuesForDeltaStorage.tempDeltas);
                        for (uint16_t i = reinterpret_cast<uint16_t>(&EEPROMDataArray[ValuesForDeltaStorage.DeltaArrayIndex]);
                                i
                                        < reinterpret_cast<uint16_t>(&EEPROMDataArray[ValuesForDeltaStorage.DeltaArrayIndex])
                                                + sizeof(EEPROMDataUnion); ++i) {
                            if (eeprom_read_byte((uint8_t*) i) != *tRAMPointer++) {
                                // Yes, I have seen this (starting with index 4 6 times 0xFF for current). Maybe undervoltage while powered by battery.
                                tone(BUZZER_PIN, NOTE_C7, 20);
                                delay(40);
                                tone(BUZZER_PIN, NOTE_C6, 20);
                                delay(40);
                                tone(BUZZER_PIN, NOTE_C7, 20);
                                break;
                            }
                        }

                        // start two new 4 bit compressed values
                        ValuesForDeltaStorage.DeltaArrayIndex++; // increase every second sample
                        ValuesForDeltaStorage.tempDeltaIsEmpty = true;
                    }
                }
            }
        }
    }

    printValuesForPlotter(aVoltageNoLoadMillivolt, aMilliampere, aMilliohm, sDoPrintCaption);
    Serial.println();

}

void dumpEEPROM(uint8_t *aEEPROMAdress, uint8_t aNumberOf16ByteBlocks) {
    for (uint8_t i = 0; i < aNumberOf16ByteBlocks; ++i) {
        Serial.print(F("0x"));
        Serial.print((uint16_t) aEEPROMAdress, HEX);
        Serial.print(F(": "));
        for (uint8_t j = 0; j < 16; ++j) {
            uint8_t tEEPROMValue = eeprom_read_byte(aEEPROMAdress++);
            Serial.print(F(" 0x"));
            Serial.print(tEEPROMValue, HEX);
        }
        Serial.println();
    }
}

void storeCapacityAndCutoffLevelToEEPROM_LCD() {
    eeprom_update_word(&EEPROMStartValues.CapacityMilliampereHour, sMeasurementInfo.CapacityMilliampereHour);
    eeprom_update_byte(&EEPROMStartValues.CutoffLevel, sCutoffLevel);
    if (!sOnlyPlotterOutput) {
// Print should be done after checkForDoublePress() in order to not disturb the double press detection
        Serial.print(F("Cut off level "));
        Serial.print(getCutoffLevelAsCharacter());
        Serial.print(F(" and capacity "));
        Serial.print(sMeasurementInfo.CapacityMilliampereHour);
        Serial.println(F(" mAh stored"));
    }
#if defined(USE_LCD)
    myLCD.setCursor(0, 0);
    myLCD.print(F("Capacity stored "));
    delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
#if defined(LOCAL_DEBUG)
    dumpEEPROM((uint8_t*) &EEPROMStartValues, 1);
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
        if (sOnlyLoggerFunctionality) {
            printMillisValueAsFloat(sMeasurementInfo.Voltages.Logger.MaximumMillivolt);
        } else {
            printMillisValueAsFloat(aVoltageToPrint);
        }
        Serial.print(F("V:"));
        Serial.print(aVoltageToPrint);
        Serial.print(F(" Current="));
//        Serial.print(F("V Current="));
        Serial.print(StartValues.initialDischargingMilliampere);
        Serial.print(F("mA->"));
        Serial.print(aMilliampereToPrint);
        Serial.print(F("mA:"));
        Serial.print(aMilliampereToPrint);
        if (StartValues.initialDischargingMilliohm > 0) {
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
            Serial.print(F("ohm"));
        }
        Serial.print(F(" Capacity="));
        if (sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff != 0
                && sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff != sMeasurementInfo.CapacityMilliampereHour) {
            Serial.print(sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff);
            Serial.print('_');
        }
        Serial.print(sMeasurementInfo.CapacityMilliampereHour);
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
        if (StartValues.initialDischargingMilliohm > 0) {
            Serial.print(' ');
            Serial.print(aMilliohmToPrint);
        }
    }
#endif
}

#define CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE            0
#define CAPACITY_STARTED                1       // Current voltage is below or equal NominalFullVoltageMillivolt and higher or equal CutoffVoltageMillivoltHigh
#define CAPACITY_COMPLETED              2       // Current voltage is below CutoffVoltageMillivoltHigh
/*
 * Reads EEPROM delta values array
 * - print data for plotter and compute ESR on the fly from voltage, current and load resistor
 * - compute capacity from current (if defined SUPPORT_CAPACITY_RESTORE)
 * - restore battery type and capacity accumulator as well as mAh
 * - Capacity is stored in sMeasurementInfo.CapacityMilliampereHour and sMeasurementInfo.CapacityAccumulator
 */
void readAndProcessEEPROMData(bool aDoConvertInsteadOfPrint) {
    EEPROMDataUnion tEEPROMData;
    /*
     * First copy EEPROM start values to RAM
     */
    eeprom_read_block(&StartValues, &EEPROMStartValues, sizeof(EEPROMStartValues));

    bool tIsCompressed = (StartValues.compressionFlag != FLAG_NO_COMPRESSION);

// search last non 0xFF (not cleared) value
    int tLastNonZeroIndex;
    for (tLastNonZeroIndex = (MAX_NUMBER_OF_SAMPLES - 1); tLastNonZeroIndex >= 0; --tLastNonZeroIndex) {
        eeprom_read_block(&tEEPROMData, &EEPROMDataArray[tLastNonZeroIndex], sizeof(tEEPROMData));
        if (tEEPROMData.compressed.DeltaMillivolt != 0xFF || tEEPROMData.compressed.DeltaMilliampere != 0xFF
                || tEEPROMData.compressed.DeltaESRMilliohm != 0xFF) {
            break;
        }
    }
    tLastNonZeroIndex++; // Convert from 0 to MAX_NUMBER_OF_SAMPLES-1 to ValuesForDeltaStorage.DeltaArrayIndex to 0 to MAX_NUMBER_OF_SAMPLES

    sCutoffLevel = StartValues.CutoffLevel;
    uint16_t tVoltage = StartValues.initialDischargingMillivolt;
    sMeasurementInfo.Voltages.Logger.MaximumMillivolt = tVoltage; // for logger
    sMeasurementInfo.BatteryTypeIndex = getBatteryTypeIndex(tVoltage);
    uint8_t tCapacityMilliampereHourStandardValueState;
    /*
     * Check if start voltage > voltage for standard capacity computation
     * Assume, that voltage is not rising, so check at first value is sufficient
     */
    if (tVoltage >= BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].NominalFullVoltageMillivolt) {
        sMeasurementInfo.isStandardCapacity = true;
        tCapacityMilliampereHourStandardValueState = CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE;
    } else {
        sMeasurementInfo.isStandardCapacity = false;
        tCapacityMilliampereHourStandardValueState = CAPACITY_STARTED;
    }

    uint16_t tMilliampere = StartValues.initialDischargingMilliampere;
    uint16_t tMilliohm = StartValues.initialDischargingMilliohm;
    sMeasurementInfo.ESRMilliohm = tMilliohm; // displayed in summary print
    sMeasurementInfo.Milliampere = tMilliampere; // To avoid overflow detection for ESRMilliohm print
    sMeasurementInfo.CapacityMilliampereHour = StartValues.CapacityMilliampereHour; // required for printing and append to EEPROM functionality
    sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff = 0;

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
        Serial.print(F("compressed EEPROM values found"));
        if (!sOnlyLoggerFunctionality) {
            Serial.print(F(" for type="));
            Serial.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].TypeName);
        }
        Serial.print(F(", cut off level="));
        Serial.println(getCutoffLevelAsCharacter());
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
        eeprom_read_block(&tEEPROMData, &EEPROMDataArray[i], sizeof(tEEPROMData));

        if (!tIsCompressed) {
            /***************
             * Uncompressed
             ***************/
            tVoltage += tEEPROMData.uncompressed.DeltaMillivolt;
            tMilliampere += tEEPROMData.uncompressed.DeltaMilliampere;
            tMilliohm += tEEPROMData.uncompressed.DeltaESRMilliohm;
            if (aDoConvertInsteadOfPrint) {
                /*
                 * Convert uncompressed values here
                 */
                storeBatteryValuesToEEPROM(tVoltage, tMilliampere, tMilliohm);
//                Serial.print(F("Stored result "));
//                printValuesForPlotter(ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt, ValuesForDeltaStorage.lastStoredMilliampere, ValuesForDeltaStorage.lastStoredMilliohm, false);
            }
            tCapacityAccumulator += tMilliampere; // putting this into printValuesForPlotter() increases program size
#if defined(LOCAL_DEBUG)
            Serial.print(F("EEPROM values="));
            Serial.print(tEEPROMData.uncompressed.DeltaMillivolt);
            Serial.print(F("|0x"));
            Serial.print(tEEPROMData.compressed.DeltaMillivolt, HEX);
            Serial.print(' ');
            Serial.print(tEEPROMData.uncompressed.DeltaMilliampere);
            Serial.print(F("|0x"));
            Serial.print(tEEPROMData.compressed.DeltaMilliampere, HEX);
            Serial.print(' ');
            Serial.print(tEEPROMData.uncompressed.DeltaESRMilliohm);
            Serial.print(F("|0x"));
            Serial.print(tEEPROMData.compressed.DeltaESRMilliohm, HEX);
//                Serial.print(F(" CapAccu="));
//                Serial.print(tCapacityAccumulator);
            Serial.println();
#endif

        } else {
            /*************
             * Compressed
             *************/
            /*
             * Process first part of compressed data
             */
            uint8_t t4BitVoltageDelta = tEEPROMData.compressed.DeltaMillivolt;
            tVoltage += getDelta(t4BitVoltageDelta >> 4);

            uint8_t t4BitMilliampereDelta = tEEPROMData.compressed.DeltaMilliampere;
            tMilliampere += getDelta(t4BitMilliampereDelta >> 4);

            uint8_t t4BitMilliohmDelta = tEEPROMData.compressed.DeltaESRMilliohm;
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
                if (sMeasurementInfo.Voltages.Logger.MaximumMillivolt < tVoltage)
                    sMeasurementInfo.Voltages.Logger.MaximumMillivolt = tVoltage;
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
#if defined(LOCAL_DEBUG)
            Serial.print(F("EEPROM values=0x"));
            Serial.print(tEEPROMData.compressed.DeltaMillivolt, HEX);
            Serial.print(F(" 0x"));
            Serial.print(tEEPROMData.compressed.DeltaMilliampere, HEX);
            Serial.print(F(" 0x"));
            Serial.print(tEEPROMData.compressed.DeltaESRMilliohm, HEX);
//            Serial.print(F(" CapAccu="));
//            Serial.print(tCapacityAccumulator);
            Serial.println();

#endif
        }

        if (!aDoConvertInsteadOfPrint) {

            if (sMeasurementInfo.Voltages.Logger.MaximumMillivolt < tVoltage) sMeasurementInfo.Voltages.Logger.MaximumMillivolt =
                    tVoltage;
            uint16_t tVoltageForPrint = tVoltage; // To print markers for start and end of standard capacity
            uint8_t tPrintDelayed = 0; // to append text at values print output

            if (!sOnlyLoggerFunctionality) {
                /*
                 * Get "standard" capacity from NominalFullVoltageMillivolt to CutoffVoltageMillivoltHigh
                 */
                if (tCapacityMilliampereHourStandardValueState == CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE
                        && tVoltage <= BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].NominalFullVoltageMillivolt) {
                    // Store initial capacity at nominal full voltage to subtract it later
                    tCapacityMilliampereHourStandardValueState = CAPACITY_STARTED;
                    tCapacityAccumulatorStartForStandardValue = tCapacityAccumulator;
                    tPrintDelayed = 1; // print text after print of values
                    if (sOnlyPlotterOutput) {
                        tVoltageForPrint += 50; // modify voltage before print of values
                    }

                } else if (tCapacityMilliampereHourStandardValueState == CAPACITY_STARTED
                        && tVoltage < BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].CutoffVoltageMillivoltHigh) {
                    tCapacityMilliampereHourStandardValueState = CAPACITY_COMPLETED;
                    sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff = (tCapacityAccumulator
                            - tCapacityAccumulatorStartForStandardValue) / NUMBER_OF_STORAGES_PER_HOUR; // -> tCapacityAccumulator / 60
                    if (i != tLastNonZeroIndex - 1) { // do not modify last value line containing caption
                        tPrintDelayed = 2; // print text after print of values
                        if (sOnlyPlotterOutput) {
                            tVoltageForPrint += 50; // modify voltage before print of values
                        }
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
                    if (sMeasurementInfo.isStandardCapacity) {
                        Serial.print(F("Standard "));
                    }
                    if (sOnlyLoggerFunctionality) {
                        Serial.print(F("capacity="));
                    } else {
                        Serial.print(F("capacity at high cut off="));
                    }
                    Serial.print(sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff);
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
            if (sMeasurementInfo.CapacityMilliampereHour == 0) {
                Serial.print(F("No capacity was stored, so use computed capacity of "));
                Serial.print(tCurrentCapacityMilliampereHourComputed);
            } else {
                /*
                 * The observed delta was around 1% :-)
                 */
                int16_t tCurrentCapacityMilliampereHourDelta = sMeasurementInfo.CapacityMilliampereHour
                        - tCurrentCapacityMilliampereHourComputed;
                Serial.print(F("Stored minus computed capacity="));
                Serial.print(tCurrentCapacityMilliampereHourDelta);
            }
            Serial.println(F(" mAh"));

            /*
             * Print Standard capacity a between NominalFullVoltageMillivolt and  CutoffVoltageMillivoltHigh,
             * if we have both values.
             */
            if (!sOnlyLoggerFunctionality && sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff != 0
                    && sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff != tCurrentCapacityMilliampereHourComputed) {
                if (sMeasurementInfo.isStandardCapacity) {
                    Serial.print(F("Standard "));
                }
                Serial.print(F("computed capacity between "));
                if (sMeasurementInfo.isStandardCapacity) {
                    Serial.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].NominalFullVoltageMillivolt);
                } else {
                    Serial.print(StartValues.initialDischargingMillivolt);
                }
                Serial.print(F(" mV and "));
                Serial.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].CutoffVoltageMillivoltHigh);
                Serial.print(F(" mV="));
                Serial.print(sMeasurementInfo.CapacityMilliampereHourValueAtHighCutoff);
                Serial.println(F(" mAh"));
            }
        } // if (!sOnlyPlotterOutput)

        if (sMeasurementInfo.CapacityMilliampereHour == 0) {
            sMeasurementInfo.CapacityMilliampereHour = tCurrentCapacityMilliampereHourComputed;
        }

// restore capacity accumulator
        sMeasurementInfo.CapacityAccumulator = sMeasurementInfo.CapacityMilliampereHour
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
