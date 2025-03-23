/*
 *  UltimateBatteryTester.cpp
 *
 *  Measure Voltage, current, ESR (internal series resistance) and capacity of batteries.
 *  Stores the values in EEPROM and display the values and the discharge graph on a tablet or mobile
 *  running the BlueDisplay app https://play.google.com/store/apps/details?id=de.joachimsmeyer.android.bluedisplay
 *  or on a connected Arduino Plotter.
 *
 *  The load is periodically detached to compute the ESR of the battery.
 *  ESR is: (noLoadVoltage - loadVoltage) / loadCurrent.
 *  The internal LED is active for the time the load is detached. This results in a 1 second blinking.
 *
 *  To suspend a measurement while in storage mode, press single for stopping and storing current capacity.
 *  If measurement is stopped, it can be started by another press and then the new measurement must be appended by another press.
 *
 *  If pin 9 not connected to ground, verbose output for Arduino Serial Monitor is enabled. This is not suitable for Arduino Plotter.
 *
 *  Stored and displayed ESR is the average of the ESR's of the last storage period (1 min).
 *
 *  Data is stored to EEPROM in a delta format, starting with 16 bit start value
 *  adding an 8 bit signed delta at each storage sample.
 *
 *  Storage time is 5 hours and 36 min (336 + initial sample) for one sample every minute.
 *  When EEPROM space is exhausted, data is compressed by combining 2 deltas,
 *  resulting in an effective 2 minutes storage sample time, thus reducing the resolution.
 *  With 2 minutes per sample, storage time is 11h 12min. For a Li-ion this is equivalent to around 3300 mAh at a default load resistor of 2 + 1 Ohm.
 *  With 4 minutes per sample, storage time is 22h 24min.
 *
 *  One EEPROM block contains the initial start voltage, current value as well as the capacity, battery type and value of the used load resistor.
 *  These values are stored at the beginning of the measurement
 *  The capacity is stored at end of measurement or on button press during the storage.
 *
 *  LOGGER:
 *  Logger resistor is 0.2 Ohm and much smaller than the resistor used for ESR measurement.
 *  This is because this resistance is added to the ESR of the probe and may influence e.g. charging or discharging end detection of the external circuit.
 *  The current is sampled
 *  The logger resistor is connected to pin A4.
 *
 *
 *  Copyright (C) 2021-2025  Armin Joachimsmeyer
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

#define VERSION_EXAMPLE "5.1"
// The change log is at the bottom of the file

//#define TRACE
//#define DEBUG

//#define SUPPORT_BLUEDISPLAY_CHART // Enables output of values and the discharge graph on a tablet or mobile running the BlueDisplay app, disables Arduino plotter functions.

/*
 * If you want, you can calibrate your ADC readout by replacing this value with the voltage you measured a the AREF pin after the program started.
 * For my Nanos I measured e.g. 1060 mV and 1093 mV.
 */
#if !defined(ADC_INTERNAL_REFERENCE_MILLIVOLT)
#define ADC_INTERNAL_REFERENCE_MILLIVOLT    1100L // Change to value measured at the AREF pin. If value > real AREF voltage, measured values are > real values
#endif

/*
 * Activate the type of LCD you use
 * Default is parallel LCD with 2 rows of 16 characters (1602).
 * 2004 Display is and will not be supported. Use BlueDisplay app instead.
 * Serial LCD uses A4/A5 - the hardware I2C pins on Arduino
 */
#if !defined(USE_SERIAL_LCD) && !defined(USE_PARALLEL_LCD) && !defined(USE_NO_LCD)
// Choose your default LCD type
#define USE_PARALLEL_LCD
//#define USE_SERIAL_LCD
#endif

/*
 * Pin and ADC definitions
 *
 * Pin 2 / INT0 is used for Start/Stop button
 * Pin 3 to 8 are used for parallel LCD connection
 */
#define ADC_CHANNEL_FOR_VOLTAGE      0 // Pin A0 for Uno. This is the ADC channel, not the pin for voltage measurement!
#define ADC_CHANNEL_CURRENT          1 // Pin A1 for Uno. This is the ADC channel, not the pin for current measurement!
#define VOLTAGE_RANGE_EXTENSION_PIN A2 // This pin is low to extend the voltage range from 2.2 volt to 4.4 volt
#define LOAD_HIGH_PIN               A3 // This pin is high to switch on the high load (3 ohm)
#if defined(USE_PARALLEL_LCD)
#define ADC_CHANNEL_LOGGER_CURRENT   4 // Pin A4 for Uno. This is the ADC channel, not the pin for current measurement for Logger!
#define BUZZER_PIN                  A5
#else
// USE_SERIAL_LCD here
// A4 + A5, the hardware I2C pins on Arduino, are used for Serial LCD, so we must redefine this 2 pins
#define ADC_CHANNEL_LOGGER_CURRENT   6 // Pin A6 for Nano, not available on Uno. This is the ADC channel, not the pin for current measurement for Logger!
#define BUZZER_PIN                   3
#endif

// Mode pins
#define ONLY_PLOTTER_OUTPUT_PIN      9 // Verbose output to Arduino Serial Monitor is disabled, if connected to ground. This is intended for Arduino Plotter mode.
#define ONLY_LOGGER_MODE_PIN        10 // If connected to ground, current is measured at the shunt at channel 4 / A4 and voltage still at channel 0 / pin A0.
// If powered by USB verbose verbose output to Arduino Serial Monitor is disabled, if NOT connected to ground.
#define CUTOFF_LEVEL_PIN            11 // If connected to ground, "cut off is low" is displayed and discharge ends at a lower voltage. E.g. Li-ion discharge ends at 3000 mV instead of 3500 mV
#define LOAD_LOW_PIN                12 // This pin is high to switch on the low load (10 ohm). A4 is occupied by I2C for serial LCD display.

/*
 * External circuit definitions
 */
#define LOGGER_SHUNT_RESISTOR_MILLIOHM          200L    // 0.2 ohm -> Resolution of 5 mA
#define ESR_SHUNT_RESISTOR_MILLIOHM             2000L   // 2 ohm
#define LOAD_LOW_MILLIOHM                       (1000 + ESR_SHUNT_RESISTOR_MILLIOHM) // Additional 1 ohm
#define LOAD_HIGH_MILLIOHM                      (10 * 1000 + ESR_SHUNT_RESISTOR_MILLIOHM) // Additional 10 ohm
#define ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE    2L      // Divider with 100 kOhm and 100 kOhm -> 2.2 V range
#define ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE   4L      // Divider with 100 kOhm and 33.333 kOhm -> 4.4 V range

/*
 * Imports and definitions for start/stop button at pin 2
 */
#define USE_BUTTON_0                // Enable code for button 0 at INT0 / pin 2.
#define NO_BUTTON_RELEASE_CALLBACK
//#define BUTTON_IS_ACTIVE_HIGH     // If you have an active high button (sensor button) attached
#include "EasyButtonAtInt01.hpp"
void handleStartStopButtonPress(bool aButtonToggleState);       // The button press callback function
EasyButton startStopButton0AtPin2(&handleStartStopButtonPress); // Button is connected to INT0 (pin2)
void checkForDelayedButtorProcessing(); // If LCD is in use do not process button
volatile bool sInLCDPrint;              // To synchronize LCD access for button handler
bool sOnlyPlotterOutput; // Suppress all serial output except logger data. Contains the (inverted) value of the pin ONLY_PLOTTER_OUTPUT_PIN

// Flags for logger mode
#define NO_LOGGER_MODE_REQUESTED            0
#define LOGGER_MODE_REQUESTED               1
#define LOGGER_EXTERNAL_CURRENT_DETECTED    2
#define LOGGER_EXTERNAL_VOLTAGE_DETECTED    4
uint8_t sInLoggerModeAndFlags = NO_LOGGER_MODE_REQUESTED; // Initially contains the (inverted) value of the pin ONLY_LOGGER_MODE_PIN

#define CUTOFF_LEVEL_HIGH       0 // Switch off current percentage is 50% for logger. Is default case
#define CUTOFF_LEVEL_LOW        1 // 25% for logger.
#define CUTOFF_LEVEL_ZERO       2 // 12% for logger. End discharging at 0.05 volt (DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT)
#define DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT   50  // 50 mV
#define NO_BATTERY_MILLIVOLT                    DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT   // 50 mV
#define NO_LOGGER_MILLAMPERE                    12
/*
 * Cutoff detection is done by comparing current mA value with low pass mA value
 * in sLastMilliampereLowPassFiltered5 shifted by 1 or 2 or 3 (divide by 2 or 4 or 8) corresponding to 50% or 25% or 12.5%
 */
uint16_t sLastMilliampereLowPassFiltered5;      // For sCutoffMilliamperePercent
bool sLastValueOfCutoffLevelPin;                // To support changing between normal and low by using pin CUTOFF_LEVEL_PIN

//#define NO_TONE_WARNING_FOR_VOLTAGE_TOO_LOW_FOR_STANDARD_CAPACITY_COMPUTATION

/*
 * Support for BlueDisplay
 */
#if defined(SUPPORT_BLUEDISPLAY_CHART)
//#define DO_NOT_NEED_BASIC_TOUCH_EVENTS
#define DO_NOT_NEED_LONG_TOUCH_DOWN_AND_SWIPE_EVENTS  // Disables LongTouchDown and SwipeEnd events.
#define ONLY_CONNECT_EVENT_REQUIRED         // Disables reorientation, redraw and SensorChange events
#define SUPPRESS_SERIAL_PRINT               // To reduce code size

#include "BlueDisplay.hpp" // part of https://github.com/ArminJo/Arduino-BlueDisplay
//#define BLUETOOTH_BAUD_RATE BAUD_115200   // Activate this, if you have reprogrammed the HC05 module for 115200
#  if !defined(BLUETOOTH_BAUD_RATE)
#define BLUETOOTH_BAUD_RATE     9600        // Default baud rate of my HC-05 modules, which is not very reactive
#  endif

/*
 * Scale the screen such, that this fit horizontally.
 * Border - YLabels - Chart with CO2_ARRAY_SIZE / 2 - Border - Buttons for 6 big characters - Border
 * Take border as CO2_ARRAY_SIZE / 20, button width as  4 * CO2_ARRAY_SIZE / 20 and base font size as CO2_ARRAY_SIZE / 40
 */
#define DISPLAY_WIDTH   ((MAX_NUMBER_OF_SAMPLES * 33L) / 20L) // 556
#define BASE_TEXT_SIZE  (MAX_NUMBER_OF_SAMPLES / 20L) // 16
#define BASE_TEXT_WIDTH ((((MAX_NUMBER_OF_SAMPLES / 20L) * 6 ) + 4) / 10) // 10
#define BUTTON_WIDTH    (BASE_TEXT_SIZE * 5)
#define CHART_START_X   (BASE_TEXT_SIZE * 3)
#define CHART_WIDTH     (MAX_NUMBER_OF_SAMPLES + 1) // +1 for the first sample at minute 0 -> 337, 5 hours and 36 min
#define CHART_AXES_SIZE (BASE_TEXT_SIZE / 8)
#define BUTTONS_START_X ((BASE_TEXT_SIZE * 4) + CHART_WIDTH)

#define MAIN_VALUES_COLOR           COLOR16_BLUE
#define VALUES_COLOR                COLOR16_RED
#define VALUES_TEXT_SIZE            (BASE_TEXT_SIZE * 2)
#define VALUES_POSITION_Y           BASE_TEXT_SIZE
#define VALUES_POSITION_X           (BASE_TEXT_SIZE * 2)
#define MESSAGE_START_POSITION_Y    (BASE_TEXT_SIZE * 3)

#define ESR_POSITION_X              (VALUES_POSITION_X + (BASE_TEXT_SIZE * 20))
#define CURRENT_POSITION_X          (VALUES_POSITION_X + (BASE_TEXT_SIZE * 10))
#define CHART_VALUES_POSITION_X     (CHART_START_X + CHART_WIDTH)

#define CHART_VOLTAGE_COLOR     COLOR16_RED
#define CHART_ESR_COLOR         COLOR16_GREEN
#define CHART_CURRENT_COLOR     COLOR16_BLUE

#define CHART_AXES_COLOR        COLOR16_BLUE
#define CHART_GRID_COLOR        COLOR16_YELLOW
#define CHART_DATA_COLOR        COLOR16_RED
#define CHART_TEXT_COLOR        COLOR16_BLACK

#define CHART_MAXIMUM_X_SCALE_FACTOR            8L
#define CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED 30L
#define SECONDS_PER_MINUTES                    60L

#define BRIGHTNESS_LOW      2
#define BRIGHTNESS_MIDDLE   1
#define BRIGHTNESS_HIGH     0
#define START_BRIGHTNESS    BRIGHTNESS_HIGH
uint8_t sCurrentBrightness = START_BRIGHTNESS;
color16_t sBackgroundColor = COLOR16_WHITE;
color16_t sTextColor = COLOR16_BLACK;
uint8_t sChartDataTextSize;

uint16_t sCompressionOffset200Millivolt; // Value to subtract from millivolt before compressing
uint8_t sCompressionFactor; // Factor for millivolt -> chart array data

Chart VoltageChart;
Chart ResistanceAndCurrentChart;

const char RunningStateButtonStringBooting[] PROGMEM = "Booting";
const char RunningStateButtonStringWaiting[] PROGMEM = "Waiting";
const char RunningStateButtonStringTesting[] PROGMEM = "Testing";
const char RunningStateButtonStringRunning[] PROGMEM = "Running";
const char RunningStateButtonStringStopped[] PROGMEM = "Stopped"; // stopped manually
const char RunningStateButtonStringFinished[] PROGMEM = "Finished"; // stopped by detecting end condition
const char *const sRunningStateButtonTextStringArray[] PROGMEM = { RunningStateButtonStringBooting, RunningStateButtonStringWaiting,
        RunningStateButtonStringTesting, RunningStateButtonStringRunning, RunningStateButtonStringStopped,
        RunningStateButtonStringFinished };
BDButton TouchButtonRunningState;
void setRunningStateButtonText();

BDButton TouchButtonAppend;

const char CutoffButtonStringHigh[] PROGMEM = "Cutoff High";
const char CutoffButtonStringLow[] PROGMEM = "Cutoff Low";
const char CutoffButtonStringZero[] PROGMEM = "Cutoff Zero";
const char *const sCutoffButtonTextStringArray[] PROGMEM = { CutoffButtonStringHigh, CutoffButtonStringLow, CutoffButtonStringZero };
BDButton TouchButtonCutoffHighLowZero;
void setCutoffHighLowZeroButtonText(bool doDrawButton);

BDButton TouchButtonBatteryLogger;
//BDButton TouchButtonRedraw;
BDButton TouchButtonBrightness; // Brightness handling costs 400 byte
BDButton TouchButtonOnlyTextVolt;
BDButton TouchButtonOnlyTextESR;
BDButton TouchButtonOnlyTextAmpere;

#define TYPE_VOLTAGE    0
#define TYPE_ESR        1
#define TYPE_CURRENT    2
uint8_t sChartReadValueArrayType; // 0 = voltage, 1 = ESR, 2 = current
uint8_t sChartDisplayValueArrayType; // 0 = voltage, 1 = ESR, 2 = current

void connectHandler(void);

void initBatteryChart();
void initDisplay(void);
void redrawDisplay(void);
void clearValueArea();
void drawButtons();
void drawTextButtons();
void clearAndDrawChart();

void changeBrightness();
void doBrightness(BDButton *aTheTouchedButton, int16_t aValue);

//void printMeasurementValues();
void printChartValues();
void printCapacityValue();
void readAndDrawEEPROMValues();
#endif // SUPPORT_BLUEDISPLAY_CHART

/*
 * Activate the type of LCD you use
 * Default is parallel LCD with 2 rows of 16 characters (1602).
 * Serial LCD uses A4/A5 - the hardware I2C pins on Arduino
 */
#if !defined(USE_SERIAL_LCD) && !defined(USE_PARALLEL_LCD) && !defined(USE_NO_LCD)
#define USE_PARALLEL_LCD
//#define USE_SERIAL_LCD
#endif

// definitions for a 1602 LCD
#define LCD_COLUMNS 16
#define LCD_ROWS 2

#define LCD_MESSAGE_PERSIST_TIME_MILLIS     2000 // 2 second to view a message on LCD
#if defined(USE_SERIAL_LCD)
#define USE_SOFT_I2C_MASTER // Requires SoftI2CMaster.h + SoftI2CMasterConfig.h. Saves 2110 bytes program memory and 200 bytes RAM compared with Arduino Wire
#include "LiquidCrystal_I2C.hpp"  // This defines USE_SOFT_I2C_MASTER, if SoftI2CMasterConfig.h is available. Use only the modified version delivered with this program!
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

//#define ENABLE_STACK_ANALYSIS
#if defined(ENABLE_STACK_ANALYSIS)
#include "AVRUtils.h" // include for initStackFreeMeasurement() and printRAMInfo()
#endif

/*
 * Measurement timing
 */
#define MILLIS_IN_ONE_SECOND 1000L
#define SECONDS_IN_ONE_MINUTE 60L
//#define TEST // to speed up testing the code
#if defined(TEST)
#define NUMBER_OF_INITIAL_SAMPLES               4 // 4 seconds (-2 for initial display of append message) before starting discharge and storing, to have time to just test for ESR of battery.
#define SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS   500L // The time of the activated load for one sample.
#define INITIAL_NUMBER_OF_SECONDS_PER_STORAGE   5 // 1 minute, if we have 1 sample per second
#else
#define NUMBER_OF_INITIAL_SAMPLES               30 // Before starting discharge and storing, to have time to just test for ESR of battery. 30 seconds with SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS as 1000.
#define SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS   MILLIS_IN_ONE_SECOND // 1 s. The time of the activated load for one sample.
#  if !defined(INITIAL_NUMBER_OF_SECONDS_PER_STORAGE)
#define INITIAL_NUMBER_OF_SECONDS_PER_STORAGE   SECONDS_IN_ONE_MINUTE // 60, if we have 1 sample per second (SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS)
#  endif
#endif

/*
 * For Logger current, we take the average of 20 ms (50 HZ) | 769 samples to cover variations due to mains frequency
 * and 1 voltage value.
 * This is done 20 times per second and then averaged again.
 */
#define LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT 769L // for ADC_PRESCALE32 and 20 ms (50 Hz)
#define LOGGER_SAMPLE_PERIOD_MILLIS             50 // 20 Hz.
#define LOGGER_SAMPLE_FREQUENCY_HZ              (MILLIS_IN_ONE_SECOND / LOGGER_SAMPLE_PERIOD_MILLIS)
#define LOGGER_SAMPLES_PER_MINUTE               (SECONDS_IN_ONE_MINUTE * LOGGER_SAMPLE_FREQUENCY_HZ * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT)  // = 230.400 every minute for 10 HZ

#define MAX_VALUES_DISPLAYED_IN_PLOTTER         500 // The Arduino 1.8 Plotter displays 500 values before scrolling
#define BATTERY_OR_VOLTAGE_DETECTION_PERIOD_MILLIS         (MILLIS_IN_ONE_SECOND / 2) // 500 ms
#define BATTERY_DETECTION_MINIMAL_MILLIVOLT     50

/*
 * Values for different battery types
 */
struct BatteryTypeInfoStruct {
    const char TypeName[11];
    uint16_t DetectionThresholdVoltageMillivolt; // Type is detected if voltage is below this threshold
    uint16_t NominalFullVoltageMillivolt;        // The voltage to start the "standard" capacity computation
    uint16_t CutoffVoltageMillivoltHigh; // The voltage to stop the "standard" capacity computation. Cut off happens below this voltage
    uint16_t CutoffVoltageMillivoltLow;
    uint8_t LoadType;                    // High (3 Ohm) or low (12 Ohm)
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
#define TYPE_INDEX_LOGGER       42

struct BatteryTypeInfoStruct BatteryTypeInfoArray[] = { { "No battery", 100, 0, 0, 0, NO_LOAD, 0 }, /* Below 100 mV and not below 50, to avoid toggling between no and low batt */
{ "Low batt. ", 1000, 0, NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW, DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT, HIGH_LOAD, 100 }, /* For researching of worn out batteries. */
{ "NiCd NiMH ", 1460, 1400, NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT, NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW, HIGH_LOAD, 100 }, /*400 mA*/
{ "Alkali    ", 1550, 1500, 1300, 1000, HIGH_LOAD, 100 }, /*500 mA*/
{ "NiZn batt.", 1850, 1650, 1400, 1300, HIGH_LOAD, 100 }, /*550 mA*/
{ "LiFePO4   ", 3400, 3400, 3050, 2700, LOW_LOAD, 10 }, /*270 mA https://www.jackery.com/blogs/knowledge/ultimate-guide-to-lifepo4-voltage-chart*/
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
            uint16_t AverageMillivolt; // Same memory location as NoLoadMillivolt and therefore is stored in EEPROM
            uint16_t MinimumMillivolt;
            uint16_t MaximumMillivolt;
        } Logger;
    } Voltages;
    uint16_t Milliampere;
    uint16_t ESRMilliohm; // Average of last 60 values. ESR - Equivalent Series Resistor | internal battery resistance.
    uint16_t sESRDeltaMillivolt = 0; // only displayed at initial ESR testing
    uint32_t CapacityAccumulator;
    uint16_t CapacityMilliampereHour;

    uint8_t LoadState; // NO_LOAD | LOW_LOAD 12 ohm | HIGH_LOAD 3 ohm
    uint8_t BatteryTypeIndex = TYPE_INDEX_NO_BATTERY;
    uint8_t CutoffLevel; // One of CUTOFF_LEVEL_HIGH, CUTOFF_LEVEL_LOW and CUTOFF_LEVEL_ZERO. Starts with the (inverted) value of the pin CUTOFF_LEVEL_PIN
    uint8_t LoadSwitchSettleTimeMillis; // One of CUTOFF_LEVEL_HIGH, CUTOFF_LEVEL_LOW and CUTOFF_LEVEL_ZERO. Starts with the (inverted) value of the pin CUTOFF_LEVEL_PIN
} sMeasurementInfo;

/*
 * Standard capacity is from NominalFullVoltageMillivolt to CutoffVoltageMillivoltHigh.
 * It is used to compare capacities independent of initial charge voltage and cutoff voltages.
 * It is computed while reading data from EEPROM.
 * Not used for logger.
 */
uint16_t sStandardCapacityMilliampereHour;
bool isStandardCapacityAvailable;

#define TYPE_CHAR_FOR_INCOMPLETE_CAPACITY   ' '
#define TYPE_CHAR_FOR_HIGH_CUTOFF  'h'
#define TYPE_CHAR_FOR_LOW_CUTOFF   'l'
#define TYPE_CHAR_FOR_ZERO_CUTOFF   'l'
#define TYPE_CHAR_FOR_STANDARD_CAPACITY     's'

struct lastDiplayedValuesStruct {
    uint16_t VoltageMillivolt;
    uint16_t Milliampere;
    uint16_t ESRMilliohm;
    uint16_t CapacityMilliampereHour;
} sLastDiplayedValues;

uint8_t sLastBatteryTypeIndex = TYPE_INDEX_MAX + 2; // For triggering printing only if BatteryTypeIndex value changed
uint16_t sLastVoltageNoLoadMillivoltForPrintAndCountdown;
bool sBatteryOrCurrentOrVoltageWasDetected = false; // set by detectBatteryOrLoggerVoltageOrCurrentLCD() if detection was successful
bool sButtonUsageMessageWasPrinted = false;
bool sVoltageNoLoadIsDisplayedOnLCD = false;

bool sVoltageRangeIsLow;                        // true for 2.2 V range

uint8_t sLoggerADCVoltageReference = INTERNAL;  // INTERNAL or DEFAULT
struct Logger1SecondAccumulatorStruct {
    uint32_t RawVoltageAccumulator;    // normalized for 4.4 V range
    uint32_t RawCurrentAccumulator;    // 16 bit is only OK up to 50 Hz
    uint16_t RawSampleCount;           // is 20 for 20 HZ
    uint16_t MinimumRawVoltage;
    uint16_t MaximumRawVoltage;
} sLogger1SecondAccumulator;

struct Logger1MinuteAccumulatorStruct {
    uint16_t RawSampleCount;          // is 3000 every minute for 50 HZ
    uint32_t RawVoltageAccumulator8ShiftRight; // Unshifted maximum is 5000 * 769 * 20 * 60 = 6.921 billion => overflow at 12.2 Volt
    uint32_t RawCurrentAccumulator;   // Unshifted maximum is 1023 * 769 * 20 * 60 = 1.416 billion
} sLogger1MinuteAccumulator;

/*
 * Tester state machine
 */
#define STATE_SETUP_AND_READ_EEPROM          0
#define STATE_WAITING_FOR_BATTERY_OR_EXTERNAL 1 // Check if battery is inserted and determine type or external current or voltage is connected to start with logging
#define STATE_INITIAL_SAMPLES                2 // Only voltage (and ESR) measurement every n seconds for NUMBER_OF_INITIAL_SAMPLES samples
#define STATE_SAMPLE_AND_STORE_TO_EEPROM     3 // Main measurement state, get values and store to EEPROM
#define STATE_STOPPED                        4 // Switch off voltage reached, until removal of battery
volatile uint8_t sMeasurementState = STATE_SETUP_AND_READ_EEPROM;
bool sMeasurementWasFinished = false; // true if stopped by detecting end condition and not by user button.

// Override defaults defined in ADCUtils.h
#define LI_ION_VCC_UNDERVOLTAGE_THRESHOLD_MILLIVOLT 3500 // 3.5 volt
#define VCC_CHECK_PERIOD_MILLIS                     (60000L) // check every minute
#define VCC_UNDERVOLTAGE_CHECKS_BEFORE_STOP         5 // Shutdown after 5 times below VCC_UNDERVOLTAGE_THRESHOLD_MILLIVOLT or below VCC_EMERGENCY_UNDERVOLTAGE_THRESHOLD_MILLIVOLT
#include "ADCUtils.hpp"

/*
 * Attention timing
 */
#define STATE_BATTERY_DETECTION_ATTENTION_PERIOD_MILLIS     (MILLIS_IN_ONE_SECOND * 60)
#define STATE_STOP_ATTENTION_PERIOD_MILLIS                  (MILLIS_IN_ONE_SECOND * 600)
unsigned long sLastMillisOfStateWaitingForBatteryOrVoltageBeep;
unsigned long sLastMillisOfStateStoppedBeep;

uint16_t sSampleCountForStoring;
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
// The start values for the delta array
struct EEPROMStartValuesStruct {
    uint16_t initialDischargingMillivolt;
    uint16_t initialDischargingMilliampere;
    uint16_t initialDischargingMilliohm;
    uint16_t LoadResistorMilliohm;
    uint16_t CapacityMilliampereHour; // Is set at end of measurement or by store button
    uint8_t CutoffLevel; // One of CUTOFF_LEVEL_HIGH, CUTOFF_LEVEL_LOW and CUTOFF_LEVEL_ZERO.
    uint8_t BatteryTypeIndex;
    uint8_t inLoggerModeAndFlags;
    uint8_t NumberOfSecondsPerStorage;
} StartValues;
EEMEM EEPROMStartValuesStruct EEPROMStartValues;
#define EEPROM_EMPTY_VALUE      0xFF // the value of an unwritten / empty EEPROM byte

#if defined(TEST)
#define MAX_NUMBER_OF_SAMPLES   9
#else
// EEPROM size for values is (1024 - sizeof(EEPROMStartValues)) / 3 = 108 / 3 = 336
#define MAX_NUMBER_OF_SAMPLES      (((E2END - sizeof(EEPROMStartValuesStruct)) / 3) & ~0x01) // 336 (+ the initial value) For compressing it is forced to be even
#endif
struct EEPROMData {
    int8_t DeltaMillivolt; // one 8 bit delta
    int8_t DeltaMilliampere;
    int8_t DeltaESRMilliohm;
};
EEMEM EEPROMData EEPROMDataArray[MAX_NUMBER_OF_SAMPLES];

#if defined(SUPPORT_BLUEDISPLAY_CHART)
// Must be below definition of MAX_NUMBER_OF_SAMPLES :-(
uint8_t sChartValueArray[MAX_NUMBER_OF_SAMPLES] __attribute__((section(".noinit"))); // must be in noinit, to be first overwritten on stack overflow
uint16_t sChartValueArrayIndex;
struct ChartValuesStruct {
    uint16_t Millivolt;
    uint16_t Milliampere;
    uint16_t ESRMilliohm;
};
ChartValuesStruct sLastChartData;
#endif

struct ValuesForDeltaStorageStruct {
    uint16_t lastStoredVoltageNoLoadMillivolt;
    uint16_t lastStoredMilliampere;
    uint16_t lastStoredMilliohm;
    int DeltaArrayIndex; // The index of the next values to be written. -1 to signal, that start values must be written.
} ValuesForDeltaStorage;

bool sDoPrintCaption = true; // Value used for (recursive) call to printValuesForPlotterAndChart().

void getBatteryOrLoggerVoltageMillivolt();
void addToCapacity();
uint16_t getBatteryOrLoggerRawVoltage();
void detectBatteryOrLoggerVoltageOrCurrentLCD();
void clearLogger1SecondAccumulator();
void clearLogger1MinuteAccumulator();
void getLogger1SecondValues();
void getLogger1MinuteValues();
void handlePeriodicAccumulatingLoggerValues();

void getCurrent(uint8_t aADCChannel, uint16_t aShuntResistorMilliohm);
void getBatteryValues();
void checkAndHandleStopConditionLCD();
bool isVoltageOrCurrentRemoved();
void playEndTone();
void playAttentionTone();
void setLoad(uint8_t aNewLoadState);
void printStoredDataLCD();
void printMilliampere4DigitsLCD();
void printVoltageNoLoadMillivoltWithTrailingSpaceLCD();
void clearLastDiplayedValues();
void printMeasurementValuesLCD();
void printValuesForPlotterAndChart(uint16_t aMillivoltToPrint, uint16_t aMilliampereToPrint, uint16_t aMilliohmToPrint,
        bool aIsLastElement);
void printMillisValueAsFloat(uint16_t aValueInMillis);
void printCounter(uint16_t aNumberToPrint);

void dumpEEPROM(uint8_t *aEEPROMAdress, uint8_t aNumberOf16ByteBlocks);
void storeBatteryValuesToEEPROM(uint16_t aVoltageNoLoadMillivolt, uint16_t aMilliampere, uint16_t aMilliohm);
void storeCapacityAndCutoffLevelToEEPROM_LCD();
void readAndProcessEEPROMData(bool aStoreValuesForDisplayAndAppend);
void handlePeriodicStoringToEEPROM();
void handleStateStopped();
void handleEndOfStateInitialSamples();
void handlePeriodicDetectionOfProbe();
void checkAndHandleVCCUndervoltage();
void checkAndHandleCutOffPinLevelChange();

void delayAndCheckForButtonPress();
void printButtonUsageMessageLCD();
char getCutoffLevelAsCharacter();
void printCutoff();
void printlnIfNotPlotterOutput();
void printStateString(uint8_t aState);

void switchToStateWaitingForBatteryOrVoltage();
void switchToStateInitialSamples();
void switchToStateSampleAndStoreToEEPROM(uint16_t aInitialSampleCountForStoring);
void switchToStateStoppedLCD(char aReasonCharacter);

#if defined(USE_LCD)
void LCDPrintAsFloatWith2Decimals(uint16_t aValueInMillis);
void LCDPrintAsFloatWith3Decimals(uint16_t aValueInMillis);
void LCDPrintVCC(uint8_t aLCDLine);
void LCDResetCursor();
void LCDClearLine(uint8_t aLineNumber);
#endif

/*
 * Program starts here
 */
#if defined(TRACE)
#define LOCAL_TRACE
#else
//#define LOCAL_TRACE // This enables TRACE output only for this file
#endif
#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file
#endif

void _delay(unsigned long aDelayMillis) {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    delayMillisWithCheckAndHandleEvents(aDelayMillis);
#else
    delay(aDelayMillis);
#endif
}

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

#if defined(SUPPORT_BLUEDISPLAY_CHART)
    initSerial(BLUETOOTH_BAUD_RATE); // converted to Serial.begin(BLUETOOTH_BAUD_RATE);
#else
    Serial.begin(115200);
#endif

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif

    if (!digitalRead(ONLY_LOGGER_MODE_PIN)) {
        sInLoggerModeAndFlags = LOGGER_MODE_REQUESTED;
    }
    sOnlyPlotterOutput = !digitalRead(ONLY_PLOTTER_OUTPUT_PIN); // default behavior
    if (!sOnlyPlotterOutput) {
        // Just to know which program is running on my Arduino
        Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));
#if !defined(SUPPORT_BLUEDISPLAY_CHART)
        /*
         * Button pin info
         */
        Serial.print(F("Button pin="));
        Serial.println(INT0_PIN);
        Serial.println(
                F(
                        "To suppress such prints not suited for Arduino plotter, connect pin " STR(ONLY_PLOTTER_OUTPUT_PIN) " to ground"));

#  if (INITIAL_NUMBER_OF_SECONDS_PER_STORAGE * SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS != 60000)
        Serial.print(F("Sample period="));
        Serial.print(SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS);
        Serial.print(F("ms, storage period="));
        Serial.print(INITIAL_NUMBER_OF_SECONDS_PER_STORAGE);
        Serial.println('s');
#  endif
        Serial.print(F("Maximum number of uncompressed samples="));
        Serial.print(MAX_NUMBER_OF_SAMPLES + 1); // + 1 since we always have the initial value.
        Serial.print(F(" | "));
        Serial.print(((MAX_NUMBER_OF_SAMPLES) * (INITIAL_NUMBER_OF_SECONDS_PER_STORAGE / SECONDS_IN_ONE_MINUTE)) / 60);
        Serial.print(F("h "));
        Serial.print(((MAX_NUMBER_OF_SAMPLES) * (INITIAL_NUMBER_OF_SECONDS_PER_STORAGE / SECONDS_IN_ONE_MINUTE)) % 60);
        Serial.println(F("min"));
#endif
    }

#if defined(ENABLE_STACK_ANALYSIS)
    initStackFreeMeasurement(); // used 229, unused 339
#endif

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
    LCDResetCursor();
    myLCD.print(F("Battery Tester "));
    myLCD.setCursor(0, 1);
    myLCD.print(F(VERSION_EXAMPLE "  " __DATE__));
#endif

    /******************************
     * BlueDisplay initialization
     ******************************/
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (!sOnlyPlotterOutput) {
        BlueDisplay1.initCommunication(&Serial, &connectHandler); // introduces up to 1.5 seconds delay
    }
#endif

    _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

#if defined(ENABLE_STACK_ANALYSIS)
        printRAMInfo(&Serial);
#  if !defined(BD_USE_SIMPLE_SERIAL)
        Serial.flush();
#  endif
#endif

#if defined(USE_LCD)
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (!BlueDisplay1.isConnectionEstablished()) {
#endif
        myLCD.setCursor(0, 1);
        if (sOnlyPlotterOutput) {
            myLCD.print(F("Only plotter out"));
        } else {
            myLCD.print(F("No plotter out  "));
        }
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS / 2);

        myLCD.setCursor(0, 1);
        if (sInLoggerModeAndFlags) {
#    if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Only logger mode"));
        }
#    endif
            myLCD.print(F("Only logger mode"));
        }
        _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
        LCDClearLine(1); // Clear line "Only logger mode"
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    }
#endif

#  if (INITIAL_NUMBER_OF_SECONDS_PER_STORAGE * SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS != 60000)
    myLCD.setCursor(0, 1);
    myLCD.print(INITIAL_NUMBER_OF_SECONDS_PER_STORAGE);
    myLCD.print(F(" s / storage "));
    _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#  endif
#endif // defined(USE_LCD)

    tone(BUZZER_PIN, 2200, 100); // usage of tone() costs 1524 bytes code space

    /*
     * Get and print EEPROM data
     * get sMeasurementInfo.CutoffLevel for later appending
     */
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (!BlueDisplay1.isConnectionEstablished()) {
        readAndProcessEEPROMData(true); // just get the values for LCD display
    }
#else
    readAndProcessEEPROMData(true);
#endif

    printStoredDataLCD();

    printlnIfNotPlotterOutput(); // end of stored data

    if (sInLoggerModeAndFlags) {
        sCurrentLoadResistorAverage = LOGGER_SHUNT_RESISTOR_MILLIOHM;
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
    getBatteryOrLoggerVoltageMillivolt();
    if (sMeasurementInfo.Voltages.Battery.NoLoadMillivolt < NO_BATTERY_MILLIVOLT) {
        // Battery / Logger is removed here, so start with mode determined by pin
        sMeasurementInfo.CutoffLevel = !sLastValueOfCutoffLevelPin;
        if (sInLoggerModeAndFlags && sMeasurementInfo.CutoffLevel == CUTOFF_LEVEL_HIGH) {
            // Pin not connected leads to logger default cut off level zero instead of high
            sMeasurementInfo.CutoffLevel = CUTOFF_LEVEL_ZERO;
        }
        StartValues.CutoffLevel = sMeasurementInfo.CutoffLevel; // Required, in order to keep level during conversion to compressed.
    }
    printCutoff(); // print actual cut off level

    if (isStandardCapacityAvailable) {
        /*
         * Display standard capacity info a few seconds later, but only at startup
         */
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            if (isStandardCapacityAvailable) {
                Serial.print(F("Standard "));
            }
            Serial.print(F("capacity="));
            Serial.print(sStandardCapacityMilliampereHour);
            Serial.println(F(" mAh"));
        }
#endif
#if defined(USE_LCD)
        myLCD.setCursor(7, 1);
        myLCD.print(F("s "));
        char tString[6];
        snprintf_P(tString, sizeof(tString), PSTR("%5u"), sStandardCapacityMilliampereHour);
        myLCD.print(tString);
        myLCD.print(F("mAh"));
        _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
    }

    switchToStateWaitingForBatteryOrVoltage();
}

/*
 * The main loop with a delay of 100 ms
 */
void loop() {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    checkAndHandleEvents();
#endif
    checkAndHandleVCCUndervoltage();

    if (sMeasurementState == STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
        handlePeriodicDetectionOfProbe();
    } else {
        /*
         * STATE_INITIAL_SAMPLES or STATE_SAMPLE_AND_STORE_TO_EEPROM or STATE_STOPPED here
         */
        handlePeriodicAccumulatingLoggerValues();
        auto tMillis = millis();
// For discharging, add LoadSwitchSettleTimeMillis to the second condition
        if ((sInLoggerModeAndFlags && sLogger1SecondAccumulator.RawSampleCount == LOGGER_SAMPLE_FREQUENCY_HZ)
                || (!sInLoggerModeAndFlags
                        && (unsigned) (tMillis - sLastMillisOfSample)
                                >= (SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS + sMeasurementInfo.LoadSwitchSettleTimeMillis))) {
            /*
             * Here battery is inserted or voltage detected and sample period (one second) expired
             * sMeasurementState is STATE_INITIAL_SAMPLES or STATE_SAMPLE_AND_STORE_TO_EEPROM or STATE_STOPPED
             * Do all this every second (of battery load)
             */
            sLastMillisOfSample = tMillis;

            /*
             * Get values
             */
            if (sInLoggerModeAndFlags) {
                getLogger1SecondValues();
            } else {
                getBatteryValues();
            }

            if (sMeasurementState == STATE_STOPPED) {
                // Print only tVoltageNoLoadMillivolt and check for periodic attention
                handleStateStopped();
                if (sInLoggerModeAndFlags) {
                    /*
                     * In logger mode continue to print voltage and current values on LCD and chart
                     */
                    if (sMeasurementInfo.Milliampere >= NO_LOGGER_MILLAMPERE) {
#if defined(USE_LCD)
                        myLCD.setCursor(7, 0);
                        myLCD.print("---"); // Overwrite "Finished" and signal with "---", that we are stopped now
#endif
                        printMilliampere4DigitsLCD();
                    }
                    if (sMeasurementInfo.Voltages.Battery.NoLoadMillivolt > NO_BATTERY_MILLIVOLT) {
                        printVoltageNoLoadMillivoltWithTrailingSpaceLCD();
                    }
                }

            } else {
                /*
                 * STATE_INITIAL_SAMPLES or STATE_SAMPLE_AND_STORE_TO_EEPROM here
                 * Check stop or remove conditions and display values
                 */
                if (isVoltageOrCurrentRemoved()) {
                    switchToStateWaitingForBatteryOrVoltage(); // switch back to start and do not overwrite already displayed values
                } else {
                    // Here not stopped or just removed -> Print measurement values
                    addToCapacity();
                    printMeasurementValuesLCD();

                    // For battery storing to EEPROM, we want to store the value of the stop condition in EEPROM, so do not check here.
                    if (sInLoggerModeAndFlags) {
                        checkAndHandleStopConditionLCD(); // Check every second if current drops.
                    }
                }
                /*
                 * Check for end of STATE_INITIAL_SAMPLES
                 */
                if (sMeasurementState == STATE_INITIAL_SAMPLES && sNumbersOfInitialSamplesToGo == 0) {
                    handleEndOfStateInitialSamples();
                }

                if (sMeasurementState == STATE_SAMPLE_AND_STORE_TO_EEPROM) {
                    /*
                     * STORE to EEPROM
                     */
                    handlePeriodicStoringToEEPROM(); // also calls checkAndHandleStopConditionLCD() after storing
                }
            }
        }
    } // end of handling each second

    if (!sInLoggerModeAndFlags) {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
        delayMillisAndCheckForEvent(100);
#else
        delay(100);
#endif

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

    checkAndHandleCutOffPinLevelChange();

} // end of loop()

void checkAndHandleCutOffPinLevelChange() {
    bool tValueOfCutoffLevelPin = digitalRead(CUTOFF_LEVEL_PIN);
    if (sLastValueOfCutoffLevelPin != tValueOfCutoffLevelPin) {
        sMeasurementInfo.CutoffLevel = sLastValueOfCutoffLevelPin;
        sLastValueOfCutoffLevelPin = tValueOfCutoffLevelPin;

        if (StartValues.CutoffLevel != sMeasurementInfo.CutoffLevel) {
            StartValues.CutoffLevel = sMeasurementInfo.CutoffLevel; // Update EEPROM value for next appending
            eeprom_update_byte(&EEPROMStartValues.CutoffLevel, sMeasurementInfo.CutoffLevel);
        }
        printCutoff();
    }
}
/*
 * Check for VCC undervoltage during measurements
 */
void checkAndHandleVCCUndervoltage() {
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
}
/*
 * Check if battery was inserted or voltage connected.
 * If yes, show values and type detected, update cutoff values and switch to state STATE_INITIAL_SAMPLES.
 *      Once after boot print "dbl press = stop" and "Press button to append to EEPROM".
 *
 * If no, print VCC voltage, but not if voltage was detected or battery was inserted before,
 *      to not overwrite battery/logger voltage printed in state STATE_INITIAL_SAMPLES.
 * Check for attention every minute.
 */
void handlePeriodicDetectionOfProbe() {
    if (millis() - sLastMillisOfBatteryOrVoltageDetection >= BATTERY_OR_VOLTAGE_DETECTION_PERIOD_MILLIS) {

        sLastMillisOfBatteryOrVoltageDetection = millis();
        /*
         * Check if battery was inserted or voltage connected
         */
        detectBatteryOrLoggerVoltageOrCurrentLCD();
        // we waited up to 2 seconds in detectBatteryOrLoggerVoltageOrCurrentLCD(), so must check if mode has not changed by button press
        if (sMeasurementState == STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
            if (sBatteryOrCurrentOrVoltageWasDetected) {
                /*
                 * Successfully detected here
                 * switch state and show values detected
                 */
                switchToStateInitialSamples();

                printMeasurementValuesLCD();
#if defined(USE_LCD)
                _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
#if !defined(SUPPORT_BLUEDISPLAY_CHART)
                // If found, print button usage once at start of InitialESRMeasurement, but not, if display is attached, which has attach button
                if (!sButtonUsageMessageWasPrinted) {
                    printButtonUsageMessageLCD();
                    sButtonUsageMessageWasPrinted = true;
                    sVoltageNoLoadIsDisplayedOnLCD = false;
                }
#  if defined(USE_LCD)
                LCDClearLine(1); // Clear line "append to EEPROM"
#  endif
#endif
                sLastDiplayedValues.Milliampere = 0; // to force overwrite of value
                if (sInLoggerModeAndFlags) {
                    /*
                     * Initialize logger accumulators
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
                 * Logger:  "3.98V No U and I"
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
                    LCDPrintVCC(0);
#endif
                }

                if (tNoLoadVoltageOrCurrentPrinted || sMeasurementInfo.Milliampere > 0) {
                    /*
                     * Here U or I is valid, but not both
                     */
                    printMilliampere4DigitsLCD();
                    tNoLoadVoltageOrCurrentPrinted = true;
                }

                if (tNoLoadVoltageOrCurrentPrinted) {
                    printlnIfNotPlotterOutput();
                }
                /*
                 * if not connected to USB, check for attention every minute
                 */
                if (!isVCCUSBPowered() && millis() - sLastMillisOfStateWaitingForBatteryOrVoltageBeep >= STATE_BATTERY_DETECTION_ATTENTION_PERIOD_MILLIS) {
                    sLastMillisOfStateWaitingForBatteryOrVoltageBeep = millis();
                    playAttentionTone();
                }
            }
        }
    }
}

/*
 * If current voltage is too low for computation of our standard capacity
 * show it on LCD and beep 3 times
 * If the stop button was not pressed (during this action)
 * initialize a new measurement and storage to EEPROM.
 * At least switch to state STATE_SAMPLE_AND_STORE_TO_EEPROM
 */
void handleEndOfStateInitialSamples() {
    if (!sInLoggerModeAndFlags
            && sMeasurementInfo.Voltages.Battery.NoLoadMillivolt
                    < BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].NominalFullVoltageMillivolt) {
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            Serial.print(F("Start voltage "));
            Serial.print(sMeasurementInfo.Voltages.Battery.NoLoadMillivolt);
            Serial.print(F(" V is below NominalFullVoltageMillivolt of "));
            Serial.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].NominalFullVoltageMillivolt);
            Serial.println(F(" V => standard capacity can not be computed!"));
        }
#endif
#if defined(USE_LCD)
        LCDResetCursor();
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
    _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#  endif
#endif
#if defined(USE_LCD)
        myLCD.clear();
#endif
    }

// If button was not pressed before, start a new data set
    if (sMeasurementState == STATE_INITIAL_SAMPLES) {
        /*
         * Force new data set
         */
        ValuesForDeltaStorage.DeltaArrayIndex = -1;
        sMeasurementInfo.CapacityAccumulator = 0;
        // Must reset this values here, because values are displayed before computed again from CapacityAccumulator
        sMeasurementInfo.CapacityMilliampereHour = 0;
        memset(sCurrentLoadResistorHistory, 0, sizeof(sCurrentLoadResistorHistory)); // Clear history array
        StartValues.NumberOfSecondsPerStorage = INITIAL_NUMBER_OF_SECONDS_PER_STORAGE;
        // Store first EEPROM value immediately, append is done by button and waits a full period
        switchToStateSampleAndStoreToEEPROM(INITIAL_NUMBER_OF_SECONDS_PER_STORAGE);
    }
}

/*
 * Print only tVoltageNoLoadMillivolt if it changed more than 5 mV and check for periodic attention
 */
void handleStateStopped() {
    if (abs(sLastVoltageNoLoadMillivoltForPrintAndCountdown - sMeasurementInfo.Voltages.Battery.NoLoadMillivolt) > 5) {
        printVoltageNoLoadMillivoltWithTrailingSpaceLCD();
        printlnIfNotPlotterOutput();
    }
    /*
     * Check for attention every 10 minute, after the current measurement was finished
     */
    if (millis() - sLastMillisOfStateStoppedBeep >= STATE_STOP_ATTENTION_PERIOD_MILLIS) {
        sLastMillisOfStateStoppedBeep = millis();
        playAttentionTone();
    }
}

void handlePeriodicStoringToEEPROM() {
    sSampleCountForStoring++;
//    Serial.print(F(" |"));
//    Serial.print(sSampleCountForStoring);
    /*
     * Check for periodic storage to EEPROM
     */
    if ((sSampleCountForStoring * MILLIS_IN_ONE_SECOND) / SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS
            >= StartValues.NumberOfSecondsPerStorage) { // Will be optimized by compiler :-)
        sSampleCountForStoring = 0;

        if (sInLoggerModeAndFlags) {
            getLogger1MinuteValues(); // get the smoother values here for storing to EEPROM
        }
        if (sMeasurementState != STATE_STOPPED) {
            /*
             * Store to EEPROM, and then read from EEPROM for drawing of chart
             */
            storeBatteryValuesToEEPROM(sMeasurementInfo.Voltages.Battery.NoLoadMillivolt, sMeasurementInfo.Milliampere,
                    sMeasurementInfo.ESRMilliohm);
#if defined(SUPPORT_BLUEDISPLAY_CHART)
            if (BlueDisplay1.isConnectionEstablished()) {
                readAndDrawEEPROMValues();
            }
#endif
            checkAndHandleStopConditionLCD();
        }
    }
}

/*
 * Not used yet
 */
void printStateString(uint8_t aState) {
#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        if (aState == STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
            Serial.print(F("DETECTING BATTERY_OR_VOLTAGE"));
        } else if (aState == STATE_INITIAL_SAMPLES) {
            Serial.print(F("INITIAL SAMPLES"));
        } else if (aState == STATE_SAMPLE_AND_STORE_TO_EEPROM) {
            Serial.print(F("STORE TO EEPROM"));
        } else if (aState == STATE_STOPPED) {
            Serial.print(F("STOPPED"));
        }
    }
#else
    (void) aState;
#endif
}

void printlnIfNotPlotterOutput() {
#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        Serial.println();
    }
#endif
}

void printSwitchStateString() {
#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        Serial.print(F("Switch to state "));
        if (sMeasurementState == STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
            Serial.print(F("WAITING FOR BATTERY OR VOLTAGE"));
        } else if (sMeasurementState == STATE_INITIAL_SAMPLES) {
            Serial.print(F("INITIAL SAMPLES"));
        } else if (sMeasurementState == STATE_SAMPLE_AND_STORE_TO_EEPROM) {
            Serial.print(F("STORE TO EEPROM"));
        } else if (sMeasurementState == STATE_STOPPED) {
            Serial.print(F("STOPPED"));
        }
    }
#endif
}

/*
 * sMeasurementState is state before state change i.e. state, which is left
 */
void checkLeavingState() {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (sMeasurementState == STATE_INITIAL_SAMPLES) {
        TouchButtonAppend.removeButton(sBackgroundColor);
    }
    if (sMeasurementState == STATE_STOPPED) {
        sMeasurementWasFinished = false;
    }
#endif
}

void switchToStateWaitingForBatteryOrVoltage() {
    checkLeavingState();
    sMeasurementState = STATE_WAITING_FOR_BATTERY_OR_EXTERNAL;
    printSwitchStateString();
    printlnIfNotPlotterOutput();
    sLastMillisOfStateWaitingForBatteryOrVoltageBeep = millis();
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    setRunningStateButtonText();
#endif
}

void switchToStateInitialSamples() {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (BlueDisplay1.isConnectionEstablished()) {
        TouchButtonAppend.drawButton();
        setCutoffHighLowZeroButtonText(true);
    }
#endif
    sMeasurementState = STATE_INITIAL_SAMPLES;
    printSwitchStateString();
    printlnIfNotPlotterOutput();
    sNumbersOfInitialSamplesToGo = NUMBER_OF_INITIAL_SAMPLES;
    memset(sESRHistory, 0, sizeof(sESRHistory));
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    setRunningStateButtonText();
#endif
}

void switchToStateSampleAndStoreToEEPROM(uint16_t aInitialSampleCountForStoring) {
    sSampleCountForStoring = aInitialSampleCountForStoring;
    checkLeavingState();
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    clearValueArea(); // to remove count
#endif
    clearLastDiplayedValues(); // To force display of all values after LCD messages
    sMeasurementState = STATE_SAMPLE_AND_STORE_TO_EEPROM;
    printSwitchStateString();
    printlnIfNotPlotterOutput();
    // cutoff level may be changed for append
    eeprom_update_byte(&EEPROMStartValues.CutoffLevel, sMeasurementInfo.CutoffLevel);
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    setRunningStateButtonText();
#endif
}

/*
 * aWriteToLCD default is true.
 * @param aReasonCharacter, '-' for terminating condition met (regular end of measurement), U for VCC undervoltage, F for EEPROM full,
 *                           D for button double press, B for button press.
 */
void switchToStateStoppedLCD(char aReasonCharacter) {
    checkLeavingState(); // Append button is removed below anyway
    if (sMeasurementState != STATE_STOPPED) {
        setLoad(NO_LOAD);
        auto tOldMeasurementState = sMeasurementState;
        sMeasurementState = STATE_STOPPED;

#if defined(SUPPORT_BLUEDISPLAY_CHART)
        TouchButtonAppend.removeButton(sBackgroundColor);
        setRunningStateButtonText();
#endif

#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            printSwitchStateString();
            Serial.print(F(", reason="));
            Serial.println(aReasonCharacter);
        }
#endif

#if defined(USE_LCD)
        LCDResetCursor();
        myLCD.print(F("Stop measurement"));
        _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
        LCDClearLine(0);
#endif
        if (tOldMeasurementState == STATE_SAMPLE_AND_STORE_TO_EEPROM) {
            storeCapacityAndCutoffLevelToEEPROM_LCD(); // Store capacity and cut off level
        }

#if defined(USE_LCD)
        LCDResetCursor();
        myLCD.print(F("       Stopped "));
        myLCD.print(aReasonCharacter);
#endif
        sLastVoltageNoLoadMillivoltForPrintAndCountdown = 0; // to force display of NoLoad voltage
    }
}

char getCutoffLevelAsCharacter() {
    if (sMeasurementInfo.CutoffLevel == CUTOFF_LEVEL_ZERO) {
        return 'z';
    } else if (sMeasurementInfo.CutoffLevel == CUTOFF_LEVEL_LOW) {
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

/*
 * Saves 66 bytes :-)
 */
void LCDResetCursor() {
    myLCD.setCursor(0, 0);
}

void LCDPrintVCC(uint8_t aLCDLine) {
    myLCD.setCursor(0, aLCDLine);
    sVCCVoltageMillivolt = getVCCVoltageMillivolt();
    myLCD.print(sVCCVoltageMillivolt / 1000.0, 2);
    myLCD.print(F("V"));
}

void LCDPrintCutoff() {
    LCDResetCursor();
    if (sInLoggerModeAndFlags) {
        myLCD.print(F("Cut off is "));
        myLCD.print(100 >> (sMeasurementInfo.CutoffLevel + 1));
        myLCD.print(F("% I"));
    } else {
        auto tCutoffLevel = sMeasurementInfo.CutoffLevel;
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
    _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
    LCDResetCursor();
    for (uint_fast8_t i = 0; i < LCD_COLUMNS; ++i) {
        myLCD.print(' ');
    }
}
#endif

/*
 * Prints state of cut off level
 * One of CUTOFF_LEVEL_HIGH, CUTOFF_LEVEL_LOW and CUTOFF_LEVEL_ZERO
 * For Logger: CUTOFF_LEVEL_HIGH = 50%, LOW = 25% and ZERO = 12.5%
 */
void printCutoff() {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    setCutoffHighLowZeroButtonText(true);
#endif
#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        if (sInLoggerModeAndFlags) {
            Serial.print(F("End logging below "));
            Serial.print(100 >> (sMeasurementInfo.CutoffLevel + 1));
            Serial.println(F(" % of last current"));

        } else {
            auto tCutoffLevel = sMeasurementInfo.CutoffLevel;
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
#endif
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

#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        Serial.println(F("Double press \"Start/stop\" button to stop measurement"));
    }
#endif
#if defined(USE_LCD)
    LCDResetCursor();
    myLCD.print(F("dbl press = stop"));
    /*
     * and wait for 2 seconds for button press
     */
    delayAndCheckForButtonPress();

    if (sMeasurementState == tOldMeasurementState) {
#endif
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Press \"Start/stop\" button to append values to already stored EEPROM data"));
        }
#endif
#if defined(USE_LCD)
        LCDResetCursor();
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
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Call delayed button processing"));
        }
#endif
    } else {
        sInLCDPrint = false; // Enable printing by button handler
    }
}

/*
 * Check for no current
 * @return true if current is zero
 */
bool isVoltageOrCurrentRemoved() {

// check only if battery was inserted before
    if (sBatteryOrCurrentOrVoltageWasDetected
            && (((!sInLoggerModeAndFlags || (sInLoggerModeAndFlags & LOGGER_EXTERNAL_CURRENT_DETECTED))
                    && sMeasurementInfo.Milliampere == 0)
                    || ((!sInLoggerModeAndFlags || (sInLoggerModeAndFlags & LOGGER_EXTERNAL_VOLTAGE_DETECTED))
                            && sMeasurementInfo.Voltages.Battery.NoLoadMillivolt < NO_BATTERY_MILLIVOLT))) {
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            Serial.print(F("Battery or voltage removing detected. U="));
            Serial.print(sMeasurementInfo.Voltages.Battery.NoLoadMillivolt);
            Serial.print(F(" mV I="));
            Serial.print(sMeasurementInfo.Milliampere);
            Serial.println(F(" mA"));
        }
#endif
        sBatteryOrCurrentOrVoltageWasDetected = false;
        sInLoggerModeAndFlags &= ~(LOGGER_EXTERNAL_CURRENT_DETECTED | LOGGER_EXTERNAL_VOLTAGE_DETECTED);
        return true;
    }
    return false;
}

/*
 * !!!Called in ISR Context!!!
 * !!! We can be called recursively, i.e. while waiting for 2 seconds we can be called for double press !!!
 * Because interrupts are still enabled, millis() is working here :-)
 *
 * Ignore all presses in mode STATE_SETUP_AND_READ_EEPROM and STATE_WAITING_FOR_BATTERY_OR_EXTERNAL
 * Double click in 2 seconds stop measurement. -> Goes to state STATE_STOPPED
 *
 * Single press:
 * In mode STATE_WAITING_FOR_BATTERY_OR_EXTERNAL:
 *      Cycle cut off level
 * In mode STATE_STOPPED:
 *      Switch to state STATE_WAITING_FOR_BATTERY_OR_EXTERNAL
 * In mode STATE_INITIAL_SAMPLES:
 *      Appends data to already stored ones in EEPROM -> switch to state STATE_SAMPLE_AND_STORE_TO_EEPROM
 * In mode STATE_SAMPLE_AND_STORE_TO_EEPROM:
 *      Stores current capacity and stops measurement -> switch to state STATE_STOPPED
 */
void handleStartStopButtonPress(bool aButtonToggleState) {
    (void) aButtonToggleState;

    if (sMeasurementState == STATE_SETUP_AND_READ_EEPROM) {
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Early press ignored"));
        }
#endif
        return;
    }

// Do not process button  as long as LCD is in use
    if (sInLCDPrint) {
        sInLCDPrint = false; // this forces a new call to handleStartStopButtonPress() at the end of PrintBatteryValues()
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Press detected while LCD in use"));
        }
#endif
        return;
    }

    bool tIsDoublePress = startStopButton0AtPin2.checkForDoublePress(2 * EASY_BUTTON_DOUBLE_PRESS_DEFAULT_MILLIS);
#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
// Print should be done after checkForDoublePress() in order to not disturb the double press detection
        Serial.print(F("Button pressed, state="));
        printStateString(sMeasurementState);
        Serial.println();
    }
#endif

    if (tIsDoublePress && sMeasurementState != STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
        /*
         * Double press detected!
         * Go to STATE_STOPPED
         */
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Double press detected"));
        }
#endif
        switchToStateStoppedLCD('D');

    } else {
        /*
         * Single press here
         * Attention, this press can be the first press of a double press,
         * so we must wait 2 seconds and check for double press before processing single press
         */

        if (sMeasurementState == STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
            // Toggle cut off to normal, to low and to zero
            sMeasurementInfo.CutoffLevel++;
            if (sMeasurementInfo.CutoffLevel > CUTOFF_LEVEL_ZERO) {
                sMeasurementInfo.CutoffLevel = CUTOFF_LEVEL_HIGH;
            }
            printCutoff();

        } else if (sMeasurementState == STATE_SAMPLE_AND_STORE_TO_EEPROM) {
            switchToStateStoppedLCD('B'); // no check for double press required here :-)

        } else {
            /*
             * Only state STATE_INITIAL_SAMPLES and STATE_STOPPED left here!
             * Print "Start again" or "Append to EEPROM", and wait for 2 seconds for double press detection, which cancels the action
             */
            uint8_t tOldMeasurementState = sMeasurementState;
#if defined(USE_LCD)
            LCDResetCursor();
#endif
            if (tOldMeasurementState == STATE_STOPPED) {
#if !defined(SUPPRESS_SERIAL_PRINT)
                if (!sOnlyPlotterOutput) {
                    Serial.println(F("Start again"));
                }
#endif
#if defined(USE_LCD)
                myLCD.print(F("Start again     "));
                sVoltageNoLoadIsDisplayedOnLCD = false;
#endif

            } else {
                // STATE_INITIAL_SAMPLES here
#if !defined(SUPPRESS_SERIAL_PRINT)
                if (!sOnlyPlotterOutput) {
                    Serial.println(F("Append data to EEPROM"));
                }
#endif
#if defined(USE_LCD)
                myLCD.print(F("Append to EEPROM"));
#endif
            }

            /*
             * Wait for double press detection for canceling the action,
             * which means sMeasurementState can now also have value STATE_STOPPED here
             */
            delayAndCheckForButtonPress();

// Must check old value (before possible double press) in order to avoid switching from STATE_STOPPED to DetectingBattery at each double press.
            if (tOldMeasurementState == STATE_STOPPED) {
                // start a new measurement cycle
                switchToStateWaitingForBatteryOrVoltage();

            } else if (sMeasurementState == STATE_INITIAL_SAMPLES) {
                /*
                 * No stop requested during 2 seconds wait -> append to EEPROM not canceled
                 * Store next sample in 60 seconds, because we assume double press directly after entering state STATE_INITIAL_SAMPLES
                 * Otherwise we would start the appended data with a short sampling period.
                 */
                switchToStateSampleAndStoreToEEPROM(0);
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

void getBatteryOrLoggerVoltageMillivolt() {

    uint16_t tInputVoltageRaw = getBatteryOrLoggerRawVoltage();
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

#if defined(LOCAL_TRACE)
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
#if defined(LOCAL_TRACE)
    if (!sOnlyPlotterOutput) {
        Serial.println(F(" -> switch to 2.2 V range"));
    }
#endif
}

void setToHighVoltageRange() {
    sVoltageRangeIsLow = false;
    pinMode(VOLTAGE_RANGE_EXTENSION_PIN, OUTPUT);
    digitalWrite(VOLTAGE_RANGE_EXTENSION_PIN, LOW);    // required???
#if defined(LOCAL_TRACE)
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
uint16_t getBatteryOrLoggerRawVoltage() {
    uint16_t tInputVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_FOR_VOLTAGE, INTERNAL);
    /*
     * Automatic range
     */
    if (sVoltageRangeIsLow && tInputVoltageRaw >= 0x3F0) { // 1008
// switch to higher voltage range by activating the range extension resistor at pin A2
#if defined(LOCAL_TRACE)
        if (!sOnlyPlotterOutput) {
            Serial.print(F("VoltageRaw="));
            Serial.print(tInputVoltageRaw);
        }
#endif
        setToHighVoltageRange(); // 4.4 V range
        // no wait, since last reading was same channel, same reference
        tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_FOR_VOLTAGE, INTERNAL);
    }
    if (!sVoltageRangeIsLow) {
        if (tInputVoltageRaw < (((0x3F0L * ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE) / ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE) - 0x10)) {
            // switch to lower voltage range at values below 488 by deactivating the range extension resistor at pin A2
#if defined(LOCAL_TRACE)
            if (!sOnlyPlotterOutput) {
                Serial.print(F("VoltageRaw="));
                Serial.print(tInputVoltageRaw);
            }
#endif
            setToLowVoltageRange(); // 2.2 V range
            tInputVoltageRaw = readADCChannelWithReference(ADC_CHANNEL_FOR_VOLTAGE, INTERNAL);
        } else if (tInputVoltageRaw >= 0x3F0) {
            /*
             * Here we have 17 mV resolution
             * which leads to e.g. 0.3 ohm resolution at 9V and 60 mA
             */
#if defined(LOCAL_TRACE)
            if (!sOnlyPlotterOutput) {
                Serial.print(F("Switch to "));
                Serial.print((sVCCVoltageMillivolt * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE) / 1000.0, 3);
                Serial.println(F(" V range"));
            }
#endif
// switch to highest voltage range by using VCC as reference
            uint16_t tReadoutFor1_1Reference = waitAndReadADCChannelWithReference(ADC_1_1_VOLT_CHANNEL_MUX, DEFAULT); // 225 at 5 volt VCC
            tInputVoltageRaw = waitAndReadADCChannelWithReference(ADC_CHANNEL_FOR_VOLTAGE, DEFAULT);
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
    sLogger1SecondAccumulator.RawVoltageAccumulator = 0;
    sLogger1SecondAccumulator.RawCurrentAccumulator = 0;
    sLogger1SecondAccumulator.RawSampleCount = 0;
    sLogger1SecondAccumulator.MinimumRawVoltage = UINT16_MAX;
    sLogger1SecondAccumulator.MaximumRawVoltage = 0;
}

void clearLogger1MinuteAccumulator() {
    sLogger1MinuteAccumulator.RawVoltageAccumulator8ShiftRight = 0;
    sLogger1MinuteAccumulator.RawCurrentAccumulator = 0;
    sLogger1MinuteAccumulator.RawSampleCount = 0;
// start every minute with new range selection
    setToLowVoltageRange();
    sLoggerADCVoltageReference = INTERNAL;
    sMeasurementInfo.Voltages.Logger.MinimumMillivolt = UINT16_MAX;
    sMeasurementInfo.Voltages.Logger.MaximumMillivolt = 0;
}

/*
 * Compute milliampere and voltage from accumulator values and sample count
 * !!! We must be called only if sLogger1MinuteRawSampleCount == LOGGER_SAMPLE_FREQUENCY_HZ !!!
 */
void getLogger1SecondValues() {
    /*
     * Accumulate for minute
     */
    sLogger1MinuteAccumulator.RawVoltageAccumulator8ShiftRight += sLogger1SecondAccumulator.RawVoltageAccumulator >> 8;
    sLogger1MinuteAccumulator.RawSampleCount += LOGGER_SAMPLE_FREQUENCY_HZ;

    /*
     * Compute Milliampere and avoid overflow
     */
    sMeasurementInfo.Milliampere = ((((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) / 1023L)
            * sLogger1SecondAccumulator.RawCurrentAccumulator)
            / (LOGGER_SHUNT_RESISTOR_MILLIOHM * LOGGER_SAMPLE_FREQUENCY_HZ * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT));

    sLastMilliampereLowPassFiltered5 += ((sLastMilliampereLowPassFiltered5 - sLastMilliampereLowPassFiltered5) + (1 << 4)) >> 5; // 2.5 us, alpha = 1/32 0.03125, cutoff frequency 5.13 Hz @1kHz

    /*
     * Compute voltage and avoid overflow
     * >> 8 and * 4 in divisor are a fast and short way to divide by 1024
     */
    sMeasurementInfo.Voltages.Logger.AverageMillivolt =
            ((ADC_INTERNAL_REFERENCE_MILLIVOLT * (sLogger1SecondAccumulator.RawVoltageAccumulator >> 8))
                    / ((LOGGER_SAMPLE_FREQUENCY_HZ * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT * 4)
                            / ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE));
//    This gives overflow :-(
//    sMeasurementInfo.Voltages.Logger.AverageMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE)
//            * (uint32_t) sLogger1SecondAccumulator.RawVoltageAccumulator >> 8)
//            / (LOGGER_SAMPLE_FREQUENCY_HZ * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT * 4L));

    sMeasurementInfo.Voltages.Logger.MaximumMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE)
            * (uint32_t) sLogger1SecondAccumulator.MaximumRawVoltage) / 1024L);
    sMeasurementInfo.Voltages.Logger.MinimumMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE)
            * (uint32_t) sLogger1SecondAccumulator.MinimumRawVoltage) / 1024L);

#if defined(LOCAL_TRACE)
    Serial.print(F("cnt="));
    Serial.print(sLogger1SecondAccumulator.RawSampleCount);
    Serial.print(F(" Iacc="));
    Serial.print(sLogger1SecondAccumulator.RawCurrentAccumulator);
    Serial.print(' ');
    Serial.print(sMeasurementInfo.Milliampere);
    Serial.print(F(" mA, Uacc="));
    Serial.print(sLogger1SecondAccumulator.RawVoltageAccumulator);
    Serial.print(F(", Umin="));
    Serial.print(sMeasurementInfo.Voltages.Logger.MinimumMillivolt);
    Serial.print(F(" | "));
    Serial.print(sLogger1SecondAccumulator.MinimumRawVoltage);
    Serial.print(F(", Umax="));
    Serial.print(sMeasurementInfo.Voltages.Logger.MaximumMillivolt);
    Serial.print(F(" | "));
    Serial.print(sLogger1SecondAccumulator.MaximumRawVoltage);
    Serial.print(F(", Uav="));
    Serial.print(sMeasurementInfo.Voltages.Logger.AverageMillivolt);
    Serial.print(F(" mV Range="));
    Serial.println(sVoltageRangeIsLow);
#endif

    clearLogger1SecondAccumulator();
}

void getLogger1MinuteValues() {
// avoid overflow
    sMeasurementInfo.Milliampere = ((((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) / 1023L)
            * (sLogger1MinuteAccumulator.RawCurrentAccumulator / LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT))
            / (sLogger1MinuteAccumulator.RawSampleCount * LOGGER_SHUNT_RESISTOR_MILLIOHM));

    /*
     * Compute voltage and avoid overflow
     * Instead of (sLogger1MinuteRawVoltageAccumulator8ShiftRight << 8) / 1024 which would give overflow
     * we do (sLogger1MinuteRawVoltageAccumulator8ShiftRight >> 8) and divide the divisor by 2^6 (64)
     */
    sMeasurementInfo.Voltages.Logger.AverageMillivolt = (ADC_INTERNAL_REFERENCE_MILLIVOLT
            * (sLogger1MinuteAccumulator.RawVoltageAccumulator8ShiftRight >> 8))
            / (((uint32_t) sLogger1MinuteAccumulator.RawSampleCount * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT)
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
 * Sampling of 20ms takes 40.5 ms. For voltage > 4.4V it takes 48.5 ms
 */
void handlePeriodicAccumulatingLoggerValues() {
    if (sInLoggerModeAndFlags && ((unsigned) (millis() - sLastMillisOfLoggerSample) >= LOGGER_SAMPLE_PERIOD_MILLIS)) {
        sLastMillisOfLoggerSample = millis();
        digitalWrite(LED_BUILTIN, HIGH);

        if (sMeasurementState != STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
            /*
             * Read 769 current values in 20ms if not stopped
             */
// switch channel and reference
#if defined(LOCAL_TRACE)
//    uint8_t tOldADMUX =
#endif
            checkAndWaitForReferenceAndChannelToSwitch(ADC_CHANNEL_LOGGER_CURRENT, INTERNAL);
            /*
             * maximum value of readADCChannelWithReferenceAndPrescalerMultiSamples(...769) is 800 000
             * So we can have 5 k of it in a 32 bit integer
             * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT = 769 for ADC_PRESCALE32 / 26us and 20 ms (50 Hz)
             */
            uint32_t tRawCurrentValue = readADCChannelMultiSamples(ADC_PRESCALE32, LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT);
            digitalWrite(LED_BUILTIN, LOW);
            sLogger1SecondAccumulator.RawCurrentAccumulator += tRawCurrentValue;
            sLogger1MinuteAccumulator.RawCurrentAccumulator += tRawCurrentValue;
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
        waitAndReadADCChannelWithReference(ADC_CHANNEL_FOR_VOLTAGE, sLoggerADCVoltageReference);

        uint32_t tRawVoltageValue;
        uint16_t tInputMinimumRawVoltage;
        uint16_t tInputMaximumRawVoltage;

        ADCSRB = 0; // Free running mode. Only active if ADATE is set to 1.
// ADSC-StartConversion ADATE-AutoTriggerEnable ADIF-Reset Interrupt Flag
        ADCSRA = (_BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIF) | ADC_PRESCALE32);

        for (uint16_t i = 0; i < LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT;) {
            if (i == 0) {
                // start a new measurement / new range
                tRawVoltageValue = 0;
                tInputMinimumRawVoltage = UINT16_MAX;
                tInputMaximumRawVoltage = 0;
            }
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
                    checkAndWaitForReferenceAndChannelToSwitch(ADC_CHANNEL_FOR_VOLTAGE, DEFAULT);
#if defined(LOCAL_TRACE)
                if (!sOnlyPlotterOutput) {
                    Serial.print(F("Switch to "));
                    Serial.print((sVCCVoltageMillivolt * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE) / 1000.0, 3);
                    Serial.println(F(" V range"));
                }
#endif
                } else {
                    // Voltage once too high for 4.4 V divider and VCC as reference
#if defined(LOCAL_TRACE)
                if (!sOnlyPlotterOutput) {
                    Serial.println(F("overvoltage"));
                }
#endif
                    tRawVoltageValue = 0x3FF * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT; // put maximum value in tRawVoltageValue
                    break;
                }
                // Start a full new loop after changing input range
                i = 0;
                delayMicroseconds(52); // wait for 2 conversions to be gone after changing range
                continue;
            } // End of range check (tInputRawVoltage >= 0x3F0)

            tRawVoltageValue += tInputRawVoltage;
            if (tInputMinimumRawVoltage > tInputRawVoltage) {
                tInputMinimumRawVoltage = tInputRawVoltage;
            }
            if (tInputMaximumRawVoltage < tInputRawVoltage) {
                tInputMaximumRawVoltage = tInputRawVoltage;
            }
            i++; // To enable re-initialization of loop by i=0 above
        }
        ADCSRA &= ~_BV(ADATE); // Disable auto-triggering (free running mode)

// normalize to 1023 at 4.4 V
        if (sVoltageRangeIsLow) {
            tRawVoltageValue /= (ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE / ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE); // divide by 2
            tInputMaximumRawVoltage /= (ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE / ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE); // divide by 2
            tInputMinimumRawVoltage /= (ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE / ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE); // divide by 2
        } else if (sLoggerADCVoltageReference == DEFAULT) {
            // Adjust tInputVoltageRaw to a virtual 12.5 bit range -> maximum value is 5000 for 20 V
            uint16_t tReadoutFor1_1Reference = waitAndReadADCChannelWithReference(ADC_1_1_VOLT_CHANNEL_MUX, DEFAULT); // 225 at 5 volt VCC
            tRawVoltageValue = (tRawVoltageValue * 1023L) / tReadoutFor1_1Reference;
            tInputMaximumRawVoltage = (tInputMaximumRawVoltage * 1023L) / tReadoutFor1_1Reference;
            tInputMinimumRawVoltage = (tInputMinimumRawVoltage * 1023L) / tReadoutFor1_1Reference;
        }

        sLogger1SecondAccumulator.RawVoltageAccumulator += tRawVoltageValue;
        if (sLogger1SecondAccumulator.MaximumRawVoltage < tInputMaximumRawVoltage) {
            sLogger1SecondAccumulator.MaximumRawVoltage = tInputMaximumRawVoltage;
        }
        if (sLogger1SecondAccumulator.MinimumRawVoltage > tInputMinimumRawVoltage) {
            sLogger1SecondAccumulator.MinimumRawVoltage = tInputMinimumRawVoltage;
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

        sLogger1SecondAccumulator.RawSampleCount++;
        digitalWrite(LED_BUILTIN, LOW);
    }
}

/*
 * Maximal current for a 0.2 ohm shunt resistor is 5.5 A, and resolution is 5.4 mA.
 */
void getCurrent(uint8_t aADCChannel, uint16_t aShuntResistorMilliohm) {
    uint16_t tShuntVoltageRaw = waitAndReadADCChannelWithReference(aADCChannel, INTERNAL);
    sMeasurementInfo.Milliampere = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) * tShuntVoltageRaw)
            / (1023L * aShuntResistorMilliohm));
#if defined(LOCAL_TRACE)
    Serial.print(F("Ch "));
    Serial.print(aADCChannel);
    Serial.print(F(", Raw="));
    Serial.print(tShuntVoltageRaw);
    Serial.print(F(", "));
    Serial.print(sMeasurementInfo.Milliampere);
    Serial.println(F(" mA"));
#endif
}

/*
 * Assumes that load is activated before called
 * only called once in loop.
 * It gets the following values:
 * Milliampere
 * LoadMillivolt
 * + if not stopped:
 * NoLoadMillivolt
 * sESRDeltaMillivolt
 * ESRMilliohm
 * sCurrentLoadResistorAverage
 *
 */
void getBatteryValues() {
// Do it before deactivating the load
    getCurrent(ADC_CHANNEL_CURRENT, ESR_SHUNT_RESISTOR_MILLIOHM);
    getBatteryOrLoggerVoltageMillivolt();    // get current battery load voltage (no load in case of stopped)

    if (sMeasurementState == STATE_STOPPED) return; // thats all if stopped :-)

// Deactivate load and wait for voltage to settle
// During the no load period switch on the LED
    setLoad(NO_LOAD);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(sMeasurementInfo.LoadSwitchSettleTimeMillis);
    getBatteryOrLoggerVoltageMillivolt();    // get current battery NoLoadMillivolt
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
    if (sInLoggerModeAndFlags) {
        if (sMeasurementInfo.Milliampere < (sLastMilliampereLowPassFiltered5 >> (sMeasurementInfo.CutoffLevel + 1))) {
            /*
             * Switch off current condition for logger met
             */
#if !defined(SUPPRESS_SERIAL_PRINT)
            if (!sOnlyPlotterOutput) {
                Serial.print(F("Switch off current percentage "));
                Serial.print(100 >> (sMeasurementInfo.CutoffLevel + 1));
                Serial.print(F(" % mA of "));
                Serial.print(sLastMilliampereLowPassFiltered5);
                Serial.print(F(" mA reached, I="));
                Serial.print(sMeasurementInfo.Milliampere);
                Serial.print(F(" mA, capacity="));
                Serial.print(sMeasurementInfo.CapacityMilliampereHour);
                Serial.println(F(" mAh"));
            }
#endif
            tStopConditionIsMet = true;
        }

    } else {
        uint16_t tSwitchOffVoltageMillivolt;
        if (sMeasurementInfo.CutoffLevel == CUTOFF_LEVEL_ZERO) {
            tSwitchOffVoltageMillivolt = DISCHARGE_CUTOFF_LEVEL_ZERO_MILLIVOLT;
        } else if (sMeasurementInfo.CutoffLevel == CUTOFF_LEVEL_LOW) {
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
    if (tStopConditionIsMet && sMeasurementState == STATE_SAMPLE_AND_STORE_TO_EEPROM) {
        sMeasurementWasFinished = true;
        switchToStateStoppedLCD('-');
#if defined(USE_LCD)
        _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS); // show "stopped"
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
void setBatteryTypeIndex(uint16_t aBatteryVoltageMillivolt) {

// scan all threshold voltage of all battery types
    for (uint_fast8_t tBatteryTypeIndex = 0; tBatteryTypeIndex < sizeof(BatteryTypeInfoArray) / sizeof(BatteryTypeInfoStruct) - 1;
            tBatteryTypeIndex++) {
        if (aBatteryVoltageMillivolt < BatteryTypeInfoArray[tBatteryTypeIndex].DetectionThresholdVoltageMillivolt) {
#if defined(LOCAL_TRACE)
            Serial.print(F(" Battery index="));
            Serial.print(tBatteryTypeIndex);
            Serial.print(F(" BatteryVoltageMillivolt="));
            Serial.print(aBatteryVoltageMillivolt);
            Serial.print(F(" SwitchOffVoltageMillivolt="));
            Serial.print(BatteryTypeInfoArray[tBatteryTypeIndex].CutoffVoltageMillivoltHigh);
            Serial.print(F(" LoadSwitchSettleTimeMillis="));
            Serial.println(BatteryTypeInfoArray[tBatteryTypeIndex].LoadSwitchSettleTimeMillis);
#endif
            sMeasurementInfo.LoadSwitchSettleTimeMillis = BatteryTypeInfoArray[tBatteryTypeIndex].LoadSwitchSettleTimeMillis;
            sMeasurementInfo.BatteryTypeIndex = tBatteryTypeIndex;
            return;
        }
    }
// High voltage is detected
    sMeasurementInfo.BatteryTypeIndex = sizeof(BatteryTypeInfoArray) / sizeof(BatteryTypeInfoStruct) - 1;
}

/*
 * Disables the load, measures the voltage to detecting battery type and enables the load if battery detected
 * For logger it measures the current for detection
 * @return true, if battery or logger voltage and current detected
 */
void detectBatteryOrLoggerVoltageOrCurrentLCD() {
    setLoad(NO_LOAD);
    getBatteryOrLoggerVoltageMillivolt();

    if (sInLoggerModeAndFlags) {
        getCurrent(ADC_CHANNEL_LOGGER_CURRENT, LOGGER_SHUNT_RESISTOR_MILLIOHM);
        if (sMeasurementInfo.Milliampere >= NO_LOGGER_MILLAMPERE
                || sMeasurementInfo.Voltages.Battery.NoLoadMillivolt > NO_BATTERY_MILLIVOLT) {
            // External current or voltage found
            sBatteryOrCurrentOrVoltageWasDetected = true;
            if (sMeasurementInfo.Milliampere >= NO_LOGGER_MILLAMPERE) {
                sInLoggerModeAndFlags |= LOGGER_EXTERNAL_CURRENT_DETECTED;
            }
            if (sMeasurementInfo.Voltages.Battery.NoLoadMillivolt > NO_BATTERY_MILLIVOLT) {
                sInLoggerModeAndFlags |= LOGGER_EXTERNAL_VOLTAGE_DETECTED;
            }
#if !defined(SUPPRESS_SERIAL_PRINT)
            if (!sOnlyPlotterOutput) {
                Serial.print(F("Found U="));
                Serial.print(sMeasurementInfo.Voltages.Battery.NoLoadMillivolt);
                Serial.print(F(" mV, I="));
                Serial.print(sMeasurementInfo.Milliampere);
                Serial.println(F(" mA"));
            }
#endif
        }
#if defined(USE_LCD)
        else {
            myLCD.setCursor(7, 0);
            myLCD.print(F("No U or I"));
        }
#endif
    } else {

        /*
         * Battery type detection here
         */
        setBatteryTypeIndex(sMeasurementInfo.Voltages.Battery.NoLoadMillivolt);
        sMeasurementInfo.Milliampere = 0; // avoid display old current value

        if (sLastBatteryTypeIndex != sMeasurementInfo.BatteryTypeIndex) {
            // Type changed
            sLastBatteryTypeIndex = sMeasurementInfo.BatteryTypeIndex;
            if (sMeasurementInfo.BatteryTypeIndex == TYPE_INDEX_NO_BATTERY) {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
                BlueDisplay1.writeString(F("\rNo battery      "));
#endif
#if defined(USE_LCD)
                myLCD.setCursor(7, 0);
                myLCD.print(F(" No batt."));
#endif
            } else {
                // print voltage before the delay for LCD display
                printVoltageNoLoadMillivoltWithTrailingSpaceLCD();
#if !defined(SUPPRESS_SERIAL_PRINT)
            if (!sOnlyPlotterOutput) {
                Serial.print(F(" => "));
                Serial.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].TypeName);
                Serial.println(F(" found"));
            }
#endif
#if defined(SUPPORT_BLUEDISPLAY_CHART)
                BlueDisplay1.writeString(F("\rFound "));
                BlueDisplay1.writeString(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].TypeName);
#endif
#if defined(USE_LCD)
                myLCD.setCursor(0, 1);
                myLCD.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].TypeName);
                myLCD.print(F(" found"));
// The current battery voltage is displayed, so clear "No batt." message selectively
                myLCD.setCursor(7, 0);
                myLCD.print(F("         "));
                _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
                LCDClearLine(1);
#endif
                sBatteryOrCurrentOrVoltageWasDetected = true;
            }
        }
    }
}

void printVoltageNoLoadMillivoltWithTrailingSpaceLCD() {
    uint16_t tVoltageNoLoadMillivolt = sMeasurementInfo.Voltages.Battery.NoLoadMillivolt; // saves 12 bytes programming space
    sLastVoltageNoLoadMillivoltForPrintAndCountdown = tVoltageNoLoadMillivolt;

#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        printMillisValueAsFloat(tVoltageNoLoadMillivolt);
        Serial.print(F(" V ")); // for optional following mA
    }
#endif

    if (sLastDiplayedValues.VoltageMillivolt != tVoltageNoLoadMillivolt
            && abs(sLastDiplayedValues.VoltageMillivolt - tVoltageNoLoadMillivolt) > 1) {
        sLastDiplayedValues.VoltageMillivolt = tVoltageNoLoadMillivolt;

#if defined(SUPPORT_BLUEDISPLAY_CHART)
        if ((!sInLoggerModeAndFlags || (sInLoggerModeAndFlags & LOGGER_EXTERNAL_VOLTAGE_DETECTED))
                && BlueDisplay1.isConnectionEstablished()) {
            // This 2 digit resolution value can be sent twice, because the resolution hides millivolt changes
            char tStringBuffer[8];
            dtostrf((float) tVoltageNoLoadMillivolt / 1000.0, 5, 2, tStringBuffer);
            tStringBuffer[5] = ' ';
            tStringBuffer[6] = 'V';
            tStringBuffer[7] = '\0';
//    strcat(tStringBuffer," V"); // 18 bytes longer
            BlueDisplay1.drawText(VALUES_POSITION_X, VALUES_POSITION_Y, tStringBuffer, BASE_TEXT_SIZE * 2, CHART_VOLTAGE_COLOR,
                    sBackgroundColor);
        }
#endif

#if defined(USE_LCD)
        LCDResetCursor();
        LCDPrintAsFloatWith3Decimals(tVoltageNoLoadMillivolt);
        myLCD.print(F("V "));
        sVoltageNoLoadIsDisplayedOnLCD = true;
// cursor is now at 7, 0
#endif
    }
}

void printCapacity5Digits() {
#if !defined(SUPPRESS_SERIAL_PRINT)
    // Print no newline
    if (!sOnlyPlotterOutput) {
        Serial.print(F("capacity="));
        Serial.print(sMeasurementInfo.CapacityMilliampereHour);
        Serial.print(F(" mAh"));
    }
#endif
    if (sLastDiplayedValues.CapacityMilliampereHour != sMeasurementInfo.CapacityMilliampereHour) {
        sLastDiplayedValues.CapacityMilliampereHour = sMeasurementInfo.CapacityMilliampereHour;

#if defined(SUPPORT_BLUEDISPLAY_CHART)
        if (BlueDisplay1.isConnectionEstablished()) {
            printCapacityValue();
        }
#endif
#if defined(USE_LCD)
        char tString[6];
        snprintf_P(tString, sizeof(tString), PSTR("%5u"), sMeasurementInfo.CapacityMilliampereHour);
        myLCD.print(tString);
        myLCD.print(F("mAh"));
#endif
    }
}

/*
 * Print ESR, if value has changed by more than 1
 */
void printESR() {
    uint32_t tMilliohm; // Compiler complains about initialize variable, which is wrong
    if (sMeasurementState == STATE_INITIAL_SAMPLES && sMeasurementInfo.Milliampere != 0) {
        tMilliohm = sESRHistory[0];
    } else {
        tMilliohm = sMeasurementInfo.ESRMilliohm;
    }

#if !defined(SUPPRESS_SERIAL_PRINT)
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
#endif

    if (sLastDiplayedValues.ESRMilliohm != tMilliohm && abs(sLastDiplayedValues.ESRMilliohm - tMilliohm) > 1) {
        sLastDiplayedValues.ESRMilliohm = tMilliohm;

#if defined(SUPPORT_BLUEDISPLAY_CHART)
        // Do not print in values line if after boot, or in logger mode
        if (sMeasurementState != STATE_SETUP_AND_READ_EEPROM && BlueDisplay1.isConnectionEstablished()) {
            char tString[8];
            if (tMilliohm == __UINT16_MAX__) {
                BlueDisplay1.drawText(ESR_POSITION_X, VALUES_POSITION_Y, F("overflow"), BASE_TEXT_SIZE * 2,
                CHART_ESR_COLOR, sBackgroundColor);
            } else {
                snprintf_P(tString, sizeof(tString), PSTR("%4u m\x81"), tMilliohm);
                BlueDisplay1.drawText(ESR_POSITION_X, VALUES_POSITION_Y, tString, BASE_TEXT_SIZE * 2, CHART_ESR_COLOR,
                        sBackgroundColor);
            }
        }
#endif
#if defined(USE_LCD)
        myLCD.setCursor(0, 1);
        if (sInLoggerModeAndFlags) {
            LCDPrintAsFloatWith3Decimals(sMeasurementInfo.Voltages.Logger.MaximumMillivolt);
            myLCD.print(F("V"));
        } else {
            float tOhmFloat;
            if (tMilliohm == __UINT16_MAX__) {
                myLCD.print(F("99.99")); // Overflow
            } else {
                tOhmFloat = (float) (tMilliohm) / 1000.0;
                if (tMilliohm < 10000) {
                    myLCD.print(tOhmFloat, 3);
                } else {
                    myLCD.print(tOhmFloat, 2);
                }
            }
            myLCD.print(F("\xF4 ")); // Ohm symbol
        }
#endif
    }
}

/*
 * Is called each time counter changes
 * Print to the same location as index counter for STATE_SAMPLE_AND_STORE_TO_EEPROM
 */
void printCounter(uint16_t aNumberToPrint) {
#if defined(USE_LCD)
    myLCD.setCursor(6, 0); // in case voltage was not printed

    if (aNumberToPrint < 10 && aNumberToPrint >= 0) {
        /*
         * We start with array index -1, which indicates initialization of array :-)
         * We have "-1" once, because we store values (and increment index) after print
         */
        myLCD.print(' '); // padding space for count
        if (sMeasurementState == STATE_INITIAL_SAMPLES) {
            tone(BUZZER_PIN, 2000, 40);
        }
    }
    if (aNumberToPrint < 100) {
        myLCD.print(' '); // padding space :-)
    }
    if (aNumberToPrint < 1000) {
        myLCD.print(' '); // padding space :-)
    }
    myLCD.print(aNumberToPrint);
    myLCD.print(' '); // trailing space for count (just in case mA are > 999)
#else
    // only beep here
    if (sMeasurementState == STATE_INITIAL_SAMPLES && aNumberToPrint < 10) {
        tone(BUZZER_PIN, 2000, 40);
    }
#endif
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (sMeasurementState == STATE_INITIAL_SAMPLES && BlueDisplay1.isConnectionEstablished()) {
        /*
         * Number of samples is printed by chart values > Samples
         */
        char tString[4];
        snprintf_P(tString, sizeof(tString), PSTR("%3u"), aNumberToPrint);
        BlueDisplay1.drawText(DISPLAY_WIDTH - BASE_TEXT_SIZE * 3, BASE_TEXT_SIZE * 2, tString, BASE_TEXT_SIZE, COLOR16_RED,
                sBackgroundColor);
    }
#endif
}

/*
 * Print only if changed more than 1 mA
 * Print no newline
 */
void printMilliampere4DigitsLCD() {
#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        Serial.print(sMeasurementInfo.Milliampere);
        Serial.print(F(" mA "));
        if (!sInLoggerModeAndFlags) {
            Serial.print(F("at "));
            printMillisValueAsFloat(sCurrentLoadResistorHistory[0]);
            Serial.print(F(" ohm, "));
        }
    }
#endif
    if (sLastDiplayedValues.Milliampere != sMeasurementInfo.Milliampere
            && abs(sLastDiplayedValues.Milliampere - sMeasurementInfo.Milliampere) > 1) {
        sLastDiplayedValues.Milliampere = sMeasurementInfo.Milliampere;

#if defined(SUPPORT_BLUEDISPLAY_CHART) || defined(USE_LCD)
        char tString[10];
#endif
#if defined(SUPPORT_BLUEDISPLAY_CHART)
        if ((!sInLoggerModeAndFlags || (sInLoggerModeAndFlags & LOGGER_EXTERNAL_CURRENT_DETECTED))
                && BlueDisplay1.isConnectionEstablished()) {
            snprintf_P(tString, sizeof(tString), PSTR("%4u mA"), sMeasurementInfo.Milliampere);
            BlueDisplay1.drawText(VALUES_POSITION_X + (BASE_TEXT_SIZE * 10), VALUES_POSITION_Y, tString, BASE_TEXT_SIZE * 2,
            CHART_CURRENT_COLOR, sBackgroundColor);
        }
#endif
#if defined(USE_LCD)
        myLCD.setCursor(10, 0);
        snprintf_P(tString, sizeof(tString), PSTR("%4umA"), sMeasurementInfo.Milliampere);
        myLCD.print(tString);
#endif
    }
}

/*
 * To force display of values on LCD and BlueDisplay
 */
void clearLastDiplayedValues() {
    sLastDiplayedValues.VoltageMillivolt = 0;
    sLastDiplayedValues.Milliampere = 0;
    sLastDiplayedValues.ESRMilliohm = 0;
}

/*
 * Called exclusively from setup() after readAndProcessEEPROMData()
 */
void printStoredDataLCD() {
#if defined(USE_LCD)
    myLCD.clear();
    myLCD.print(getVCCVoltage(), 1);
    myLCD.print(F("V Stored data"));
#endif
    /*
     * Print battery values, and use state STATE_SETUP_AND_READ_EEPROM for formatting
     * "0.061o h 1200mAh" using sMeasurementInfo.ESRMilliohm
     */
    printMeasurementValuesLCD();
#if defined(USE_LCD)
    _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
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
 * STATE_SAMPLE_AND_STORE_TO_EEPROM:
 * "4.030 V,  329 mA at 11.949 ohm, ESR=0.329 ohm, capacity=1200 mAh
 *  0   4   8   C  F
 * "4.030V 312 329mA" printing EEPROM array index
 * "0.392o l 1200mAh" using sMeasurementInfo.ESRMilliohm
 *
 * Called with states STATE_SETUP_AND_READ_EEPROM, STATE_INITIAL_SAMPLES and STATE_SAMPLE_AND_STORE_TO_EEPROM
 * State STATE_WAITING_FOR_BATTERY_OR_EXTERNAL is handled in loop()
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

void printMeasurementValuesLCD() {
    sInLCDPrint = true; // disable printing by button handler
    uint8_t tMeasurementState = sMeasurementState; // Because sMeasurementState is volatile
    if (tMeasurementState != STATE_SETUP_AND_READ_EEPROM) {
        /***********************************************************************************
         * First row only for state STATE_INITIAL_SAMPLES or STATE_SAMPLE_AND_STORE_TO_EEPROM
         ***********************************************************************************/
        /*
         * Print no load voltage here
         */
        auto tLastVoltageNoLoadMillivoltForPrintAndCountdown = sLastVoltageNoLoadMillivoltForPrintAndCountdown; // Must be before printVoltageNoLoadMillivoltWithTrailingSpaceLCD()
        printVoltageNoLoadMillivoltWithTrailingSpaceLCD();
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            // 4.094 V,  334 mA at 11.949 ohm, ESR=0.334 ohm, capacity=3501 mAh
            Serial.print(F(", "));
        }
#endif

        if (tMeasurementState == STATE_INITIAL_SAMPLES) {
            /*
             * Print down counter for STATE_INITIAL_SAMPLES
             * Count down only if we are in logger mode or we do not have a rapid voltage decrease (>= 6 mV per sample)
             */
            if (sInLoggerModeAndFlags
                    || sMeasurementInfo.Voltages.Battery.NoLoadMillivolt >= tLastVoltageNoLoadMillivoltForPrintAndCountdown
                    || (tLastVoltageNoLoadMillivoltForPrintAndCountdown - sMeasurementInfo.Voltages.Battery.NoLoadMillivolt) < 6) {
                sNumbersOfInitialSamplesToGo--;
            }

#if !defined(SUPPRESS_SERIAL_PRINT)
            if (!sOnlyPlotterOutput) {
                Serial.print(sNumbersOfInitialSamplesToGo);
                Serial.print(F(" s, ")); // seconds until discharging
            }
#endif

            printCounter(sNumbersOfInitialSamplesToGo);

        } else {
#if defined(USE_LCD)
            /*
             * Print counter for STATE_SAMPLE_AND_STORE_TO_EEPROM
             * Use (index + 1) to be consistent with the number of samples displayed for array
             */
            printCounter(
                    (ValuesForDeltaStorage.DeltaArrayIndex + 1) * (StartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE));
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
    if (sMeasurementState != STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
        /*
         * STATE_SETUP_AND_READ_EEPROM + STATE_SAMPLE_AND_STORE_TO_EEPROM: "0.061o h 1200mAh" using sMeasurementInfo.ESRMilliohm
         * STATE_INITIAL_SAMPLES:                               "0.061o l  0.128V" using current ESR from sESRHistory[0]
         */
        if (sInLoggerModeAndFlags) {
#if !defined(SUPPRESS_SERIAL_PRINT)
            if (!sOnlyPlotterOutput) {
                Serial.print(F(" Min="));
                printMillisValueAsFloat(sMeasurementInfo.Voltages.Logger.MinimumMillivolt);
                Serial.print(F(" V, Avg="));
                printMillisValueAsFloat(sMeasurementInfo.Voltages.Logger.AverageMillivolt);
                Serial.print(F(" V, Max="));
                printMillisValueAsFloat(sMeasurementInfo.Voltages.Logger.MaximumMillivolt);
                Serial.print(F(" V "));
            }
#endif
        }

        /*
         *  First in line
         */
        if (sInLoggerModeAndFlags) {
#if defined(USE_LCD)
            LCDPrintVCC(1);
#endif
        } else {
            printESR();
        }

        /*
         * Print cut off level character
         */
#if defined(USE_LCD)
        myLCD.setCursor(7, 1); // This avoids problems with values >= 10 ohm
        myLCD.print(getCutoffLevelAsCharacter());
#endif

        /*
         * Print voltage difference or capacity
         */
        if (tMeasurementState == STATE_INITIAL_SAMPLES) {
            if (!sInLoggerModeAndFlags) {
                /*
                 * Print voltage difference between no load and load used for ESR computation
                 */
                uint16_t tESRDeltaMillivolt = sMeasurementInfo.sESRDeltaMillivolt; // saves 4 bytes programming space
#if !defined(SUPPRESS_SERIAL_PRINT)
                if (!sOnlyPlotterOutput) {
                    printMillisValueAsFloat(tESRDeltaMillivolt);
                    Serial.print(F(" V "));
                }
#endif
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
void updateCompleteEEPROMTo_FF() {
#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        Serial.println(F("Clear EEPROM"));
    }
#endif
    for (unsigned int i = 0; i < (E2END + 1); ++i) {
        eeprom_update_byte((uint8_t*) i, EEPROM_EMPTY_VALUE);
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
 * Store values to EEPROM as 4 bit deltas between sMeasurementInfo and ValuesForDeltaStorage and write them to EEPROM every second call
 * Upper 4 bit store the first value, lower 4 bit store the second value
 */
void storeBatteryValuesToEEPROM(uint16_t aVoltageNoLoadMillivolt, uint16_t aMilliampere, uint16_t aMilliohm) {
#if defined(LOCAL_DEBUG)
    Serial.print(F("Store "));
    Serial.print(aVoltageNoLoadMillivolt);
    Serial.print(F("mV, "));
    Serial.print(aMilliampere);
    Serial.print(F("mA, "));
    Serial.print(aMilliohm);
    Serial.print(F("mOhm at "));
    Serial.print(ValuesForDeltaStorage.DeltaArrayIndex);
    Serial.println();
#endif
    if (ValuesForDeltaStorage.DeltaArrayIndex < 0) {
        ValuesForDeltaStorage.DeltaArrayIndex = 0;

        updateCompleteEEPROMTo_FF(); // this may last one or two seconds

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
        StartValues.CutoffLevel = sMeasurementInfo.CutoffLevel;
        StartValues.BatteryTypeIndex = sMeasurementInfo.BatteryTypeIndex;
        StartValues.inLoggerModeAndFlags = sInLoggerModeAndFlags;

        StartValues.LoadResistorMilliohm = sCurrentLoadResistorAverage;
        StartValues.CapacityMilliampereHour = 0; // Capacity is written at the end or computed while reading
        StartValues.NumberOfSecondsPerStorage = INITIAL_NUMBER_OF_SECONDS_PER_STORAGE;
        eeprom_update_block(&StartValues, &EEPROMStartValues, sizeof(EEPROMStartValues));

#if defined(LOCAL_TRACE)
        dumpEEPROM((uint8_t*) &EEPROMStartValues, 1);
#endif
#if defined(LOCAL_DEBUG)
        if (!sOnlyPlotterOutput) {
            Serial.print(F("Store initial values to EEPROM at 0x"));
            Serial.println((uint16_t) &EEPROMStartValues, HEX);
        }
#endif

        /*
         * Initially set up structure for delta storage
         */
        ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt = aVoltageNoLoadMillivolt;
        ValuesForDeltaStorage.lastStoredMilliampere = aMilliampere;
        ValuesForDeltaStorage.lastStoredMilliohm = aMilliohm;

    } else {
        /*
         * Append value to delta values array
         */
        EEPROMData tDeltas;

        int16_t tVoltageDelta = aVoltageNoLoadMillivolt - ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt;
        tVoltageDelta = clipDelta(tVoltageDelta);
        ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt += tVoltageDelta;
        tDeltas.DeltaMillivolt = tVoltageDelta;

        int16_t tMilliampereDelta = aMilliampere - ValuesForDeltaStorage.lastStoredMilliampere;
        tMilliampereDelta = clipDelta(tMilliampereDelta);
        ValuesForDeltaStorage.lastStoredMilliampere += tMilliampereDelta;
        tDeltas.DeltaMilliampere = tMilliampereDelta;

        int16_t tMilliohmDelta = aMilliohm - ValuesForDeltaStorage.lastStoredMilliohm;
        tMilliohmDelta = clipDelta(tMilliohmDelta);
        ValuesForDeltaStorage.lastStoredMilliohm += tMilliohmDelta;
        tDeltas.DeltaESRMilliohm = tMilliohmDelta;
        eeprom_update_block(&tDeltas, &EEPROMDataArray[ValuesForDeltaStorage.DeltaArrayIndex], sizeof(tDeltas));

#if defined(LOCAL_DEBUG)
        Serial.print(F("EEPROM values at 0x"));
        Serial.print((uint16_t) &EEPROMDataArray[ValuesForDeltaStorage.DeltaArrayIndex], HEX);
        Serial.print(F(" ="));
        Serial.print(tDeltas.DeltaMillivolt);
        Serial.print(' ');
        Serial.print(tDeltas.DeltaMilliampere);
        Serial.print(' ');
        Serial.print(tDeltas.DeltaESRMilliohm);
        Serial.println();
#endif
#if defined(LOCAL_TRACE)
        // dump 2 lines containing the 3 byte data
        dumpEEPROM((uint8_t*) ((uint16_t) &EEPROMDataArray[ValuesForDeltaStorage.DeltaArrayIndex] & 0xFFF0), 2);
#endif

        ValuesForDeltaStorage.DeltaArrayIndex++; // increase every sample

        /*
         * Check if EEPROM is full and we must compress data
         */
        if ((unsigned int) ValuesForDeltaStorage.DeltaArrayIndex >= MAX_NUMBER_OF_SAMPLES) {
#if defined(LOCAL_DEBUG)
            if (!sOnlyPlotterOutput) {
                Serial.println();
                Serial.print(F("Compress "));
                Serial.print(MAX_NUMBER_OF_SAMPLES);
                Serial.println(F(" stored values"));
            }
#endif
            /************************************************************************
             * COMPRESSION
             * Here ValuesForDeltaStorage.DeltaArrayIndex == MAX_NUMBER_OF_SAMPLES,
             * i.e. we have written MAX_NUMBER_OF_SAMPLES to EEPROM
             * Now convert the complete EEPROM array.
             * Read 2 EEPROM samples and store the resulting value as 1 sample.
             * Write MAX_NUMBER_OF_SAMPLES / 2 samples
             ************************************************************************/
            // Initialize for new compressed storage
            ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt = StartValues.initialDischargingMillivolt;
            ValuesForDeltaStorage.lastStoredMilliampere = StartValues.initialDischargingMilliampere;
            ValuesForDeltaStorage.lastStoredMilliohm = StartValues.initialDischargingMilliohm;
            ValuesForDeltaStorage.DeltaArrayIndex = 0;

            uint16_t tVoltageMillivolt = StartValues.initialDischargingMillivolt;
            uint16_t tMilliampere = StartValues.initialDischargingMilliampere;
            uint16_t tMilliohm = StartValues.initialDischargingMilliohm;

            EEPROMData tEEPROMData;
            for (uint16_t tOldEEPROMValueIndex = 0; tOldEEPROMValueIndex < MAX_NUMBER_OF_SAMPLES; tOldEEPROMValueIndex++) {
                // read and add two values
                eeprom_read_block(&tEEPROMData, &EEPROMDataArray[tOldEEPROMValueIndex], sizeof(tEEPROMData));
                tVoltageMillivolt += tEEPROMData.DeltaMillivolt;
                tMilliampere += tEEPROMData.DeltaMilliampere;
                tMilliohm += tEEPROMData.DeltaESRMilliohm;
                tOldEEPROMValueIndex++;
                eeprom_read_block(&tEEPROMData, &EEPROMDataArray[tOldEEPROMValueIndex], sizeof(tEEPROMData));
                tVoltageMillivolt += tEEPROMData.DeltaMillivolt;
                tMilliampere += tEEPROMData.DeltaMilliampere;
                tMilliohm += tEEPROMData.DeltaESRMilliohm;
                // store resulting value
                storeBatteryValuesToEEPROM(tVoltageMillivolt, tMilliampere, tMilliohm);
            }

            /*
             * Clear remaining uncompressed EEPROM values
             */
            uint8_t *tEEPROMPointer = reinterpret_cast<uint8_t*>(&EEPROMDataArray[ValuesForDeltaStorage.DeltaArrayIndex]);
            do {
                eeprom_update_byte(tEEPROMPointer++, EEPROM_EMPTY_VALUE);
            } while (tEEPROMPointer < reinterpret_cast<uint8_t*>(&EEPROMDataArray[MAX_NUMBER_OF_SAMPLES]));

            /*
             * Set value for new compression
             */
            StartValues.NumberOfSecondsPerStorage *= 2;
            eeprom_update_byte(&EEPROMStartValues.NumberOfSecondsPerStorage, StartValues.NumberOfSecondsPerStorage); // store value in EEPROM
#if defined(LOCAL_TRACE)
            if (!sOnlyPlotterOutput) {
                Serial.println(F("Conversion done"));
                Serial.println();
            }
#endif
        }
    }
}

/*
 * For testing purposes
 */
void dumpEEPROM(uint8_t *aEEPROMAdress, uint8_t aNumberOf16ByteBlocks) {
    for (uint8_t i = 0; i < aNumberOf16ByteBlocks; ++i) {
        Serial.print(F("0x"));
        Serial.print((uint16_t) aEEPROMAdress, HEX);
        Serial.print(F(": "));
        for (uint8_t j = 0; j < 16; ++j) {
            if ((uint16_t) aEEPROMAdress > E2END) {
                Serial.println();
                return;
            }
            uint8_t tEEPROMValue = eeprom_read_byte(aEEPROMAdress++);
            Serial.print(F(" 0x"));
            Serial.print(tEEPROMValue, HEX);
        }
        Serial.println();
    }
}

void storeCapacityAndCutoffLevelToEEPROM_LCD() {
    eeprom_update_word(&EEPROMStartValues.CapacityMilliampereHour, sMeasurementInfo.CapacityMilliampereHour);
    eeprom_update_byte(&EEPROMStartValues.CutoffLevel, sMeasurementInfo.CutoffLevel);
#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
// Print should be done after checkForDoublePress() in order to not disturb the double press detection
        Serial.print(F("Cut off level "));
        Serial.print(getCutoffLevelAsCharacter());
        Serial.print(F(" and capacity "));
        Serial.print(sMeasurementInfo.CapacityMilliampereHour);
        Serial.println(F(" mAh stored"));
    }
#endif
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    BlueDisplay1.writeString(F("\rCapacity stored"));
#endif
#if defined(USE_LCD)
    LCDResetCursor();
    myLCD.print(F("Capacity stored "));
    _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
#if defined(LOCAL_DEBUG)
    dumpEEPROM((uint8_t*) &EEPROMStartValues, 1);
#endif
}

/*
 * The reproduced ESR is likely to be noisy if the relation between the load resistor and the ESR is big.
 * E.g. for Li-ion we have an load resistor of 12.156 ohm and a voltage of 4.158 volt.
 * We then get an ESR of 0.109 ohm for 339 mA and 0.073 ohm for 340 mA :-(.
 *
 * @param aIsLastElement    If true, print BlueDisplay chart or Arduino plotter caption
 */
void printValuesForPlotterAndChart(uint16_t aMillivoltToPrint, uint16_t aMilliampereToPrint, uint16_t aMilliohmToPrint,
        bool aIsLastElement) {
#if defined(ARDUINO_2_0_PLOTTER_FORMAT)
            Serial.print(F("Voltage:"));
            Serial.print(aMillivoltToPrint);
            Serial.print(F(" Current:"));
            Serial.print(aMilliampereToPrint);
            Serial.print(F(" ESR:"));
            Serial.print(aMilliohmToPrint);
            if (aDoPrintSummary) {
                // Print updated plotter caption
                Serial.print(F(" Voltage="));
                printMillisValueAsFloat(StartValues.initialDischargingMillivolt);
                Serial.print(F("V->"));
                printMillisValueAsFloat(aMillivoltToPrint);
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
                * (2 * INITIAL_NUMBER_OF_SAMPLES_PER_STORAGE)/ SECONDS_IN_ONE_MINUTE;
                Serial.print(tDurationMinutes / 60);
                Serial.print(F("h_"));
                Serial.print(tDurationMinutes % 60);
                Serial.print(F("min:aMillivoltToPrint"));
            }
#else //  defined(ARDUINO_2_0_PLOTTER_FORMAT)
#  if defined(SUPPORT_BLUEDISPLAY_CHART)

#    if defined(LOCAL_TRACE)
    Serial.print(F("ChartReadValueArrayType="));
    Serial.print(sChartReadValueArrayType);
    Serial.print(F(" ChartValueArrayIndex="));
    Serial.print(sChartValueArrayIndex);
    Serial.print(F(" IsLastElement="));
    Serial.println(aIsLastElement);
#    endif

    if (BlueDisplay1.isConnectionEstablished()) {
// input is millivolt convert to 20 for one volt
        if (sChartReadValueArrayType == TYPE_VOLTAGE) {
// Voltage
            sChartValueArray[sChartValueArrayIndex] = (aMillivoltToPrint - sCompressionOffset200Millivolt) / sCompressionFactor;
        } else if (sChartReadValueArrayType == TYPE_ESR) {
            sChartValueArray[sChartValueArrayIndex] = (aMilliohmToPrint) / sCompressionFactor;
        } else {
            // TYPE_CURRENT
            sChartValueArray[sChartValueArrayIndex] = (aMilliampereToPrint) / sCompressionFactor;
        }
        sChartValueArrayIndex++;
        if (aIsLastElement) {
            if (sChartReadValueArrayType == TYPE_VOLTAGE) {
                sLastChartData.Millivolt = aMillivoltToPrint;
                /*
                 * Compute new x scale depending on the current data length
                 */
                VoltageChart.computeAndSetXLabelAndXDataScaleFactor(sChartValueArrayIndex, CHART_MAXIMUM_X_SCALE_FACTOR);
#    if defined(LOCAL_TRACE)
                Serial.print(F("sChartValueArrayIndex="));
                Serial.print(sChartValueArrayIndex);
                Serial.print(F(" mWidthX="));
                Serial.print(VoltageChart.mWidthX);
                Serial.print(F(" mXDataScaleFactor="));
                Serial.println(VoltageChart.mXDataScaleFactor);
#    endif
                clearAndDrawChart(); // now the axes parameter are set, draw axes and grid and caption
            } else if (sChartReadValueArrayType == TYPE_ESR) {
                sLastChartData.ESRMilliohm = aMilliohmToPrint;
            } else {
                // TYPE_CURRENT
                sLastChartData.Milliampere = aMilliampereToPrint;
            }
            VoltageChart.drawChartDataWithYOffset(sChartValueArray, sChartValueArrayIndex, CHART_MODE_LINE);
        }
    }
    if (!BlueDisplay1.isConnectionEstablished())
#  endif // defined(SUPPORT_BLUEDISPLAY_CHART)

    { // new test is shorter than else!
        if (aIsLastElement) {
// Print updated plotter caption
            Serial.print(F("Voltage="));
            printMillisValueAsFloat(StartValues.initialDischargingMillivolt);
            Serial.print(F("V->"));
            if (sInLoggerModeAndFlags) {
                printMillisValueAsFloat(sMeasurementInfo.Voltages.Logger.MaximumMillivolt);
            } else {
                printMillisValueAsFloat(aMillivoltToPrint);
            }
            Serial.print(F("V:"));
            Serial.print(aMillivoltToPrint);
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
            if (isStandardCapacityAvailable) {
                Serial.print(sStandardCapacityMilliampereHour);
                Serial.print('_');
            }
            Serial.print(StartValues.CapacityMilliampereHour);
            Serial.print(F("mAh Duration"));
            uint16_t tDurationMinutes = (ValuesForDeltaStorage.DeltaArrayIndex)
                    * (StartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE);
            Serial.print('=');
            Serial.print(tDurationMinutes / 60);
            Serial.print(F("h_"));
            Serial.print(tDurationMinutes % 60);
            Serial.print(F("min"));
        } else {
            Serial.print(aMillivoltToPrint);
            Serial.print(' ');
            Serial.print(aMilliampereToPrint);
            if (StartValues.initialDischargingMilliohm > 0) {
                Serial.print(' ');
                Serial.print(aMilliohmToPrint);
            }
        }
        Serial.println();
    }
#endif //  defined(ARDUINO_2_0_PLOTTER_FORMAT)
}

#define CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE            0
#define CAPACITY_STARTED                1       // Current voltage is below or equal NominalFullVoltageMillivolt and higher or equal CutoffVoltageMillivoltHigh
#define CAPACITY_COMPLETED              2       // Current voltage is below CutoffVoltageMillivoltHigh
/*
 * Reads EEPROM delta values array
 * - print data for plotter and compute ESR on the fly from voltage, current and load resistor
 * - compute capacity from current (if defined SUPPORT_CAPACITY_RESTORE)
 * @param aStoreValuesForDisplayAndAppend true, if called at setup().
 *      Data is stored in sMeasurementInfo, BatteryTypeIndex, cutoff, CapacityMilliampereHour and CapacityAccumulator
 */
void readAndProcessEEPROMData(bool aStoreValuesForDisplayAndAppend) {
    EEPROMData tEEPROMData;
    /*
     * First copy EEPROM start values to RAM
     */
    eeprom_read_block(&StartValues, &EEPROMStartValues, sizeof(EEPROMStartValues));

// search last non EEPROM_EMPTY_VALUE / 0xFF (not cleared) value
    int tLastNonWrittenIndex;
    for (tLastNonWrittenIndex = MAX_NUMBER_OF_SAMPLES - 1; tLastNonWrittenIndex >= 0; tLastNonWrittenIndex--) {
        eeprom_read_block(&tEEPROMData, &EEPROMDataArray[tLastNonWrittenIndex], sizeof(tEEPROMData));
        if (tEEPROMData.DeltaMillivolt != (int8_t) EEPROM_EMPTY_VALUE || tEEPROMData.DeltaMilliampere != (int8_t) EEPROM_EMPTY_VALUE
                || tEEPROMData.DeltaESRMilliohm != (int8_t) EEPROM_EMPTY_VALUE) {
            break;
        }
    }
#if defined(LOCAL_TRACE)
    Serial.print(F("tLastNonWrittenIndex="));
    Serial.print(tLastNonWrittenIndex);
    Serial.print(F(" &tLastNonWrittenIndex=0x"));
    Serial.println((uint16_t) &EEPROMDataArray[tLastNonWrittenIndex], HEX);
    // dump 2 lines containing the 3 byte data
    dumpEEPROM((uint8_t*) ((uint16_t) &EEPROMDataArray[tLastNonWrittenIndex] & 0xFFF0), 2);
#endif

    int tFirstNonWrittenIndex = tLastNonWrittenIndex + 1; // Convert from 0 to MAX_NUMBER_OF_SAMPLES-1 to ValuesForDeltaStorage.DeltaArrayIndex to 0 to MAX_NUMBER_OF_SAMPLES
    uint16_t tVoltageMillivolt = StartValues.initialDischargingMillivolt;

    if (aStoreValuesForDisplayAndAppend) {
        /*
         * This values can be set from start values already read
         */
        ValuesForDeltaStorage.DeltaArrayIndex = tFirstNonWrittenIndex; // for append
        sMeasurementInfo.CutoffLevel = StartValues.CutoffLevel;
        sMeasurementInfo.Voltages.Logger.MaximumMillivolt = tVoltageMillivolt; // for logger
        setBatteryTypeIndex(tVoltageMillivolt); // sets sMeasurementInfo.BatteryTypeIndex
        sMeasurementInfo.CapacityMilliampereHour = StartValues.CapacityMilliampereHour; // may be corrected later
    }
    uint8_t tCapacityMilliampereHourStandardValueState;

#if defined(SUPPORT_BLUEDISPLAY_CHART)
    sChartValueArrayIndex = 0; // Start a new chart line

#if defined(LOCAL_DEBUG)
    Serial.print(F("Store="));
    Serial.print(aStoreValuesForDisplayAndAppend);
    Serial.print(F(" Type="));
    Serial.print(sChartReadValueArrayType);
#endif
    /*
     * For 7 grids vertical and 1.4 full range, (compression * YDataFactor) = 1
     *
     * For each grid of 200 mV, mOhm, mA,
     * we need a compression of 10 to reduce 1000 mX to a display value of 100.
     * If we want to have a 20 mX resolution we need a compression of 1.
     *    Then a value of 100mX gives a display value of 100.
     */
    if (sChartReadValueArrayType == TYPE_VOLTAGE) {
        /*
         * Process Voltage
         * We must compute offset and compression factor before storing it to 8 bit compressed data array
         * Start value is on a 200 mV value and 7 grids lower.
         * This is 1.4 volt for 8 bit data
         */
        if (tVoltageMillivolt > (7 * 200)) {
            sCompressionOffset200Millivolt = ((tVoltageMillivolt / 200) - 5) * 200;
        } else {
            sCompressionOffset200Millivolt = 0;
        }
        sCompressionFactor = 10; //each grid is 200 mV
        VoltageChart.initYLabel(sCompressionOffset200Millivolt / 1000.0, 0.2, 0.01, 3, 1); //  0.2 volt per grid, 0.01 -> 100 for 1.0 (volt) , "3.5" as label
        VoltageChart.setDataColor(CHART_VOLTAGE_COLOR);
        VoltageChart.setXLabelBaseIncrementValue(
                (StartValues.NumberOfSecondsPerStorage * CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED) / SECONDS_PER_MINUTES);
    } else if (sChartReadValueArrayType == TYPE_ESR) {
        /*
         * Process ESR
         * if ESR > 1200 mOhm then each grid is 2 Ohm, if ESR > 120 mOhm then each grid is 200 mOhm else each grid is 20 mOhm
         */
        if (StartValues.initialDischargingMilliohm > 1300) {
            sCompressionFactor = 100; // each grid is 2000 mOhm
        } else if (StartValues.initialDischargingMilliohm > 130) {
            sCompressionFactor = 10; // each grid is 200 mOhm
        } else {
            sCompressionFactor = 1; // each grid is 20 mOhm
        }
        VoltageChart.setDataColor(CHART_ESR_COLOR);
//        VoltageChart.setYDataFactor(0.01); // is set for voltage
    } else {
        /*
         * Process Current
         * if mA > 1200 mA then each grid is 2 A else each grid is 200 mA
         */
        if (StartValues.initialDischargingMilliampere > 1200) {
            sCompressionFactor = 100; // each grid is 2000 mA
        } else {
            sCompressionFactor = 10; // each grid is 200 mA
        }
        VoltageChart.setDataColor(CHART_CURRENT_COLOR);
//        VoltageChart.setYDataFactor(0.01);
    }
#endif // defined(SUPPORT_BLUEDISPLAY_CHART)

    /*
     * Check if start voltage > voltage for standard capacity computation
     * Assume, that voltage is not rising, so check at first value is sufficient
     */
    isStandardCapacityAvailable = false;
    if (tVoltageMillivolt >= BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].NominalFullVoltageMillivolt) {
        tCapacityMilliampereHourStandardValueState = CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE;
    } else {
        tCapacityMilliampereHourStandardValueState = CAPACITY_STARTED;
    }
    uint32_t tCapacityAccumulatorUntilNominalFullVoltageValue = 0; // Used to compute standard capacity later

    uint16_t tMilliampere = StartValues.initialDischargingMilliampere;
    uint16_t tMilliohm = StartValues.initialDischargingMilliohm;
    uint8_t tNumberOfEEPROMValuesPerHour = 3600 / StartValues.NumberOfSecondsPerStorage;

#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        Serial.println();
// We have always the first one as uncompressed value
        if (StartValues.NumberOfSecondsPerStorage != INITIAL_NUMBER_OF_SECONDS_PER_STORAGE) {
            Serial.print(tFirstNonWrittenIndex * 2);
            Serial.print(' ');
        } else {
            Serial.print(tFirstNonWrittenIndex + 1);
            Serial.print(F(" un"));
        }
        Serial.print(F("compressed EEPROM values found"));
        if (!sInLoggerModeAndFlags) {
            Serial.print(F(" for type="));
            Serial.print(BatteryTypeInfoArray[StartValues.BatteryTypeIndex].TypeName);
        }
        Serial.print(F(", cut off level="));
        Serial.println(getCutoffLevelAsCharacter());
    }
#endif

    uint32_t tCapacityAccumulator = tMilliampere;

    /****************************************************
     * Print the initial value and no caption to plotter
     ****************************************************/
    printValuesForPlotterAndChart(tVoltageMillivolt, tMilliampere, tMilliohm, false);

    /*******************************************
     * Loop to read and print all EEPROM values
     *******************************************/
    for (int i = 0; i < tFirstNonWrittenIndex; ++i) { // tFirstNonWrittenIndex can be from 0 to MAX_NUMBER_OF_SAMPLES

        eeprom_read_block(&tEEPROMData, &EEPROMDataArray[i], sizeof(tEEPROMData));
        tVoltageMillivolt += tEEPROMData.DeltaMillivolt;
        tMilliampere += tEEPROMData.DeltaMilliampere;
        tMilliohm += tEEPROMData.DeltaESRMilliohm;
        tCapacityAccumulator += tMilliampere; // putting this into printValuesForPlotterAndChart() increases program size

#if defined(LOCAL_TRACE)
        Serial.print(F("EEPROM values="));
        Serial.print(tEEPROMData.DeltaMillivolt);
        Serial.print(' ');
        Serial.print(tEEPROMData.DeltaMilliampere);
        Serial.print(' ');
        Serial.print(tEEPROMData.DeltaESRMilliohm);
        Serial.print(F(" CapAccu="));
        Serial.print(tCapacityAccumulator);
        Serial.println();
#endif

        if (sMeasurementInfo.Voltages.Logger.MaximumMillivolt < tVoltageMillivolt)
            sMeasurementInfo.Voltages.Logger.MaximumMillivolt = tVoltageMillivolt;
        uint16_t tVoltageForPrint = tVoltageMillivolt; // To print markers for start and end of standard capacity
#if !defined(SUPPRESS_SERIAL_PRINT)
        uint8_t tPrintDelayed = 0; // to append text at values print output
#endif

        if (!sInLoggerModeAndFlags) {
            /*
             * Get "standard" capacity from NominalFullVoltageMillivolt to CutoffVoltageMillivoltHigh
             */
            if (tCapacityMilliampereHourStandardValueState == CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE
                    && tVoltageMillivolt <= BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].NominalFullVoltageMillivolt) {
                /*
                 * Store initial capacity (capacity for voltage over NominalFullVoltageMillivolt)
                 * at reaching nominal full voltage to subtract it later
                 */
                tCapacityMilliampereHourStandardValueState = CAPACITY_STARTED;
                tCapacityAccumulatorUntilNominalFullVoltageValue = tCapacityAccumulator;
#if !defined(SUPPRESS_SERIAL_PRINT)
                tPrintDelayed = 1; // print text after print of values
                if (sOnlyPlotterOutput) {
                    tVoltageForPrint += 50; // modify voltage before print of values
                }
#endif

            } else if (tCapacityMilliampereHourStandardValueState == CAPACITY_STARTED
                    && tVoltageMillivolt < BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].CutoffVoltageMillivoltHigh) {
                tCapacityMilliampereHourStandardValueState = CAPACITY_COMPLETED;
                /*
                 * We reached CutoffVoltageMillivoltHigh, now we can compute standard capacity, if we started above nominal full voltage
                 */
                if (tCapacityAccumulatorUntilNominalFullVoltageValue > 0) {
                    isStandardCapacityAvailable = true;
                    sStandardCapacityMilliampereHour = (tCapacityAccumulator - tCapacityAccumulatorUntilNominalFullVoltageValue)
                            / tNumberOfEEPROMValuesPerHour; // -> tCapacityAccumulator / 60
                }
#if !defined(SUPPRESS_SERIAL_PRINT)
                if (i != tFirstNonWrittenIndex - 1) { // do not modify last value line containing caption
                    tPrintDelayed = 2; // print text after print of values
                    if (sOnlyPlotterOutput) {
                        tVoltageForPrint += 50; // modify voltage before print of values
                    }
                }
#endif
            }
        } // End of standard capacity handling

        /*
         * Print (the second uncompressed) values
         * At last, print the caption with values from the end of the measurement cycle to plotter
         */
        printValuesForPlotterAndChart(tVoltageForPrint, tMilliampere, tMilliohm, i == (tFirstNonWrittenIndex - 1));

#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            if (tPrintDelayed == 1) {
                Serial.print(F(" - Capacity on top of standard value="));
                Serial.print(tCapacityAccumulator / tNumberOfEEPROMValuesPerHour);
                Serial.print(F(" mAh"));
            } else if (tPrintDelayed == 2) {
                Serial.print(F(" - "));
                if (isStandardCapacityAvailable) {
                    Serial.print(F("Standard "));
                }
                if (sInLoggerModeAndFlags) {
                    Serial.print(F("capacity="));
                } else {
                    Serial.print(F("capacity at high cut off="));
                }
                Serial.print(sStandardCapacityMilliampereHour);
                Serial.print(F(" mAh"));
            }
        }
        Serial.println();
#endif
    } // End of read loop

    /**********************************************************
     * Loop was processed, handle capacity and LCD display now
     **********************************************************/
    uint16_t tCurrentCapacityMilliampereHourComputed = tCapacityAccumulator / tNumberOfEEPROMValuesPerHour;

#if !defined(SUPPRESS_SERIAL_PRINT)
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
        if (!sInLoggerModeAndFlags && sStandardCapacityMilliampereHour != 0
                && sStandardCapacityMilliampereHour != tCurrentCapacityMilliampereHourComputed) {
            if (isStandardCapacityAvailable) {
                Serial.print(F("Standard "));
            }
            Serial.print(F("computed capacity between "));
            if (isStandardCapacityAvailable) {
                Serial.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].NominalFullVoltageMillivolt);
            } else {
                Serial.print(StartValues.initialDischargingMillivolt);
            }
            Serial.print(F(" mV and "));
            Serial.print(BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].CutoffVoltageMillivoltHigh);
            Serial.print(F(" mV="));
            Serial.print(sStandardCapacityMilliampereHour);
            Serial.println(F(" mAh"));
        }
    } // if (!sOnlyPlotterOutput)
#endif

    if (aStoreValuesForDisplayAndAppend) {
        /*
         * Store values acquired during loop above
         */
        // For LCD display of stored data
        sMeasurementInfo.ESRMilliohm = tMilliohm; // displayed in summary print
        sMeasurementInfo.Milliampere = tMilliampere; // To avoid overflow detection for ESRMilliohm print

        // For append functionality
        ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt = tVoltageMillivolt;
        ValuesForDeltaStorage.lastStoredMilliampere = tMilliampere;
        ValuesForDeltaStorage.lastStoredMilliohm = tMilliohm;

        /*
         * Check if capacity must be corrected, because it was not stored at end of measurement (maybe because of sudden power down)
         */
        if (sMeasurementInfo.CapacityMilliampereHour
                < tCurrentCapacityMilliampereHourComputed - (tCurrentCapacityMilliampereHourComputed / 4)) {
            sMeasurementInfo.CapacityMilliampereHour = tCurrentCapacityMilliampereHourComputed;
        }
        // restore capacity accumulator
        sMeasurementInfo.CapacityAccumulator = sMeasurementInfo.CapacityMilliampereHour
                * ((3600L * MILLIS_IN_ONE_SECOND) / SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS);
    }
}

#if defined(SUPPORT_BLUEDISPLAY_CHART)

void doRunningState(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;

    // duplicate code from handleStartStopButtonPress
    if (sMeasurementState == STATE_SAMPLE_AND_STORE_TO_EEPROM) {
        switchToStateStoppedLCD('B'); // no check for double press required here :-)
    } else if (sMeasurementState == STATE_STOPPED) {
        // start a new measurement cycle
        switchToStateWaitingForBatteryOrVoltage();
    }
}

void setRunningStateButtonText() {
    uint8_t tButtonTextIndex = sMeasurementState;
    if (sMeasurementWasFinished) {
        tButtonTextIndex++;
    }
    TouchButtonRunningState.setTextFromStringArray((const __FlashStringHelper* const*) sRunningStateButtonTextStringArray,
            tButtonTextIndex, true);
}

/*
 * Append is only visible during state STATE_INITIAL_SAMPLES
 */
void doAppend(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;
    switchToStateSampleAndStoreToEEPROM(0);
}

void doChartType(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    sChartDisplayValueArrayType = aValue;
//    readAndDrawEEPROMValues();
    Serial.println(F("Not yet implemented"));
}

void changeBrightness() {
    if (sCurrentBrightness == BRIGHTNESS_HIGH) {
// Set to dimmed background
        BlueDisplay1.setScreenBrightness(BD_SCREEN_BRIGHTNESS_MIN);
        sCurrentBrightness = BRIGHTNESS_MIDDLE;
    } else if (sCurrentBrightness == BRIGHTNESS_MIDDLE) {
// Set to dark mode
        sBackgroundColor = COLOR16_LIGHT_GREY;
        sTextColor = COLOR16_WHITE;
        VoltageChart.setLabelColor(COLOR16_WHITE);
        VoltageChart.setBackgroundColor(COLOR16_LIGHT_GREY);
        TouchButtonOnlyTextVolt.setButtonColor(COLOR16_LIGHT_GREY);
        TouchButtonOnlyTextESR.setButtonColor(COLOR16_LIGHT_GREY);
        TouchButtonOnlyTextAmpere.setButtonColor(COLOR16_LIGHT_GREY);
        sCurrentBrightness = BRIGHTNESS_LOW;
    } else {
        // (sCurrentBrightness == BRIGHTNESS_LOW)
// Back to user brightness
        sBackgroundColor = COLOR16_WHITE;
        sTextColor = COLOR16_BLACK;
        BlueDisplay1.setScreenBrightness(BD_SCREEN_BRIGHTNESS_USER);
        VoltageChart.setLabelColor(COLOR16_BLACK);
        VoltageChart.setBackgroundColor(COLOR16_WHITE);
        TouchButtonOnlyTextVolt.setButtonColor(COLOR16_WHITE);
        TouchButtonOnlyTextESR.setButtonColor(COLOR16_WHITE);
        TouchButtonOnlyTextAmpere.setButtonColor(COLOR16_WHITE);
        sCurrentBrightness = BRIGHTNESS_HIGH;
    }
}
void doBrightness(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;
    changeBrightness();
    redrawDisplay();
}

//void doRedrawChart(BDButton *aTheTouchedButton, int16_t aValue) {
//    (void) aTheTouchedButton;
//    (void) aValue;
//    readAndDrawEEPROMValues();
//}

void setBatteryLoggerButtonText(bool doDrawButton) {
    if (sInLoggerModeAndFlags) {
        TouchButtonBatteryLogger.setText(F("Logger"), doDrawButton);
    } else {
        TouchButtonBatteryLogger.setText(F("Battery"), doDrawButton);
    }
}

void doBatteryLogger(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;
    sInLoggerModeAndFlags = !sInLoggerModeAndFlags;
    sCurrentLoadResistorAverage = LOGGER_SHUNT_RESISTOR_MILLIOHM;
    setBatteryLoggerButtonText(true);
    redrawDisplay();
}

void setCutoffHighLowZeroButtonText(bool doDrawButton) {
    if (BlueDisplay1.isConnectionEstablished()) {
        char tString[20];
        if (sInLoggerModeAndFlags) {
            // CUTOFF_LEVEL_HIGH = 50%, LOW = 25% and ZERO = 12.5%
            snprintf_P(tString, sizeof(tString), PSTR("Cutoff %2u%% I"), 100 >> (sMeasurementInfo.CutoffLevel + 1));
        } else {
            /*
             * Battery mode here
             */
            if (sMeasurementInfo.BatteryTypeIndex == TYPE_INDEX_NO_BATTERY) {
                TouchButtonCutoffHighLowZero.setTextFromStringArray((const __FlashStringHelper* const*) sCutoffButtonTextStringArray,
                        sMeasurementInfo.CutoffLevel, doDrawButton);
                return;
            } else {
                uint16_t tCutoffVoltageMillivolt;
                if (sMeasurementInfo.CutoffLevel == CUTOFF_LEVEL_ZERO) {
                    tCutoffVoltageMillivolt = 50;
                } else if (sMeasurementInfo.CutoffLevel == CUTOFF_LEVEL_HIGH) {
                    tCutoffVoltageMillivolt = BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].CutoffVoltageMillivoltHigh;
                } else {
                    tCutoffVoltageMillivolt = BatteryTypeInfoArray[sMeasurementInfo.BatteryTypeIndex].CutoffVoltageMillivoltLow;
                }
                snprintf_P(tString, sizeof(tString), PSTR("Cutoff %4umV"), tCutoffVoltageMillivolt);
            }
        }
        TouchButtonCutoffHighLowZero.setText(tString, doDrawButton);
    }
}

void doCutoffHighLowZero(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;
// Toggle cut off to normal, to low and to zero
    sMeasurementInfo.CutoffLevel++;
    if (sMeasurementInfo.CutoffLevel > CUTOFF_LEVEL_ZERO) {
        sMeasurementInfo.CutoffLevel = CUTOFF_LEVEL_HIGH;
    }
    printCutoff();
}

/*
 * This handler is called after boot or reconnect
 */
void connectHandler(void) {
    Serial.println(F("connectHandler"));
    initDisplay(); // does a clear();
    initBatteryChart();
    sCurrentBrightness = BRIGHTNESS_LOW;
    changeBrightness(); // from low to high / user defined :-)
    redrawDisplay();
}

void redrawDisplay(void) {
    Serial.println(F("redrawDisplay"));
    BlueDisplay1.clearDisplay(sBackgroundColor);
    clearLastDiplayedValues();
    drawButtons();
    readAndDrawEEPROMValues(); // draws the text buttons and calls clearAndDrawChart() at the end
}

void initDisplay(void) {
#if defined(LOCAL_DEBUG)
    Serial.print(F("InitDisplay: Host W x H="));
    Serial.print(BlueDisplay1.getHostDisplayWidth());
    Serial.print(F(" x "));
    Serial.println(BlueDisplay1.getHostDisplayHeight());
#endif
    uint16_t tDisplayHeight;
    if (BlueDisplay1.getHostDisplayHeight() > BlueDisplay1.getHostDisplayWidth()) {
        // currently we have portrait orientation -> change to lanscape, which is requested below by BD_FLAG_SCREEN_ORIENTATION_LOCK_SENSOR_LANDSCAPE
        tDisplayHeight = (DISPLAY_WIDTH * BlueDisplay1.getHostDisplayWidth()) / BlueDisplay1.getHostDisplayHeight();
    } else {
        tDisplayHeight = (DISPLAY_WIDTH * BlueDisplay1.getHostDisplayHeight()) / BlueDisplay1.getHostDisplayWidth();
    }
    // Request landscape orientation
    BlueDisplay1.setFlagsAndSize(BD_FLAG_FIRST_RESET_ALL | BD_FLAG_USE_MAX_SIZE | BD_FLAG_SCREEN_ORIENTATION_LOCK_SENSOR_LANDSCAPE,
    DISPLAY_WIDTH, tDisplayHeight);
    BlueDisplay1.setCharacterMapping(0x81, 0x03A9); // Omega in UTF16
    BlueDisplay1.setCharacterMapping(0x82, 0x0394); // Delta in UTF16

    /*
     * Adjust text size if we have a wide  screen
     * The condition is just a best guess
     */
    if ((DISPLAY_WIDTH / 2) < tDisplayHeight) {
        sChartDataTextSize = BASE_TEXT_SIZE;
    } else {
        // Here we have e.g. a 20:9 display and must reduce chart data text size to avoid overflow
        sChartDataTextSize = BASE_TEXT_SIZE / 2 + BASE_TEXT_SIZE / 4;
    }

    BlueDisplay1.clearDisplay(sBackgroundColor);

// This button is not visible during state STATE_SETUP_AND_READ_EEPROM or STATE_WAITING_FOR_BATTERY_OR_EXTERNAL
    BDButton::BDButtonPGMTextParameterStruct tBDButtonPGMParameterStruct; // Saves 480 Bytes for all 5 buttons
    BDButton::setInitParameters(&tBDButtonPGMParameterStruct, BUTTONS_START_X, BlueDisplay1.getRequestedDisplayHeight() / 6,
    BUTTON_WIDTH, BASE_TEXT_SIZE + BASE_TEXT_SIZE / 3, COLOR16_GREEN, F(""), BASE_TEXT_SIZE,
    FLAG_BUTTON_DO_BEEP_ON_TOUCH, sMeasurementState != STATE_STOPPED, &doRunningState);
    TouchButtonRunningState.init(&tBDButtonPGMParameterStruct);
    setRunningStateButtonText();

    tBDButtonPGMParameterStruct.aFlags = FLAG_BUTTON_DO_BEEP_ON_TOUCH; // No FLAG_BUTTON_TYPE_TOGGLE_RED_GREEN from here

// 74 byte This button is only visible during state STATE_INITIAL_SAMPLES
    tBDButtonPGMParameterStruct.aPositionX += BUTTON_WIDTH + (BASE_TEXT_SIZE / 4);
    tBDButtonPGMParameterStruct.aOnTouchHandler = &doAppend;
    tBDButtonPGMParameterStruct.aPGMText = F("Append");
    tBDButtonPGMParameterStruct.aWidthX = BUTTON_WIDTH - BASE_TEXT_SIZE;
    TouchButtonAppend.init(&tBDButtonPGMParameterStruct);
    tBDButtonPGMParameterStruct.aPositionX = BUTTONS_START_X;

//    TouchButtonAppend.init(BUTTONS_START_X + BUTTON_WIDTH + (BASE_TEXT_SIZE / 8), tBDButtonPGMParameterStruct.aPositionY,
//    BUTTON_WIDTH - BASE_TEXT_SIZE, BASE_TEXT_SIZE + BASE_TEXT_SIZE / 3, COLOR16_GREEN, "Append", 44, FLAG_BUTTON_DO_BEEP_ON_TOUCH,
//            sInLoggerModeAndFlags, &doAppend);

// 58 bytes
    tBDButtonPGMParameterStruct.aWidthX = BUTTON_WIDTH + (BASE_TEXT_SIZE * 3);
    tBDButtonPGMParameterStruct.aPositionY += BASE_TEXT_SIZE + (BASE_TEXT_SIZE / 2);
    tBDButtonPGMParameterStruct.aOnTouchHandler = &doCutoffHighLowZero;
    TouchButtonCutoffHighLowZero.init(&tBDButtonPGMParameterStruct);
    setCutoffHighLowZeroButtonText(false);

    tBDButtonPGMParameterStruct.aWidthX = BUTTON_WIDTH;
// 90 bytes because setBatteryLoggerButtonText() will be enabled by this code
    tBDButtonPGMParameterStruct.aPositionY += BASE_TEXT_SIZE + (BASE_TEXT_SIZE / 2);
    tBDButtonPGMParameterStruct.aOnTouchHandler = &doBatteryLogger;
    TouchButtonBatteryLogger.init(&tBDButtonPGMParameterStruct);
    setBatteryLoggerButtonText(false);

//    tBDButtonPGMParameterStruct.aPositionY += BASE_TEXT_SIZE + (BASE_TEXT_SIZE / 2);
//    tBDButtonPGMParameterStruct.aButtonColor = COLOR16_YELLOW;
//    tBDButtonPGMParameterStruct.aOnTouchHandler = &doRedrawChart;
//    tBDButtonPGMParameterStruct.aPGMText = F("Redraw");
//    TouchButtonRedraw.init(&tBDButtonPGMParameterStruct);

    tBDButtonPGMParameterStruct.aWidthX = BUTTON_WIDTH + (BASE_TEXT_SIZE * 2);
    tBDButtonPGMParameterStruct.aPositionY += BASE_TEXT_SIZE + (BASE_TEXT_SIZE / 2);
    tBDButtonPGMParameterStruct.aButtonColor = COLOR16_LIGHT_GREY;
    tBDButtonPGMParameterStruct.aOnTouchHandler = &doBrightness;
    tBDButtonPGMParameterStruct.aPGMText = F("Brightness");
    TouchButtonBrightness.init(&tBDButtonPGMParameterStruct);
    TouchButtonBrightness.setButtonTextColor(COLOR16_WHITE);

    /*
     * 3 text buttons with white background
     */
    tBDButtonPGMParameterStruct.aWidthX = BUTTON_WIDTH - BASE_TEXT_SIZE;
    tBDButtonPGMParameterStruct.aPositionX = CHART_START_X + 4;
    tBDButtonPGMParameterStruct.aPositionY = BlueDisplay1.getRequestedDisplayHeight() / 6;
    tBDButtonPGMParameterStruct.aButtonColor = sBackgroundColor;
    tBDButtonPGMParameterStruct.aOnTouchHandler = &doChartType;
    tBDButtonPGMParameterStruct.aValue = TYPE_VOLTAGE;
    tBDButtonPGMParameterStruct.aPGMText = F("Volt");
    TouchButtonOnlyTextVolt.init(&tBDButtonPGMParameterStruct);
    TouchButtonOnlyTextVolt.setButtonTextColor(CHART_VOLTAGE_COLOR);

    tBDButtonPGMParameterStruct.aPositionX += (BASE_TEXT_SIZE * 4);
    tBDButtonPGMParameterStruct.aValue = TYPE_ESR;
    tBDButtonPGMParameterStruct.aPGMText = F("ESR");
    TouchButtonOnlyTextESR.init(&tBDButtonPGMParameterStruct);
    TouchButtonOnlyTextESR.setButtonTextColor(CHART_ESR_COLOR);

    tBDButtonPGMParameterStruct.aPositionX += (BASE_TEXT_SIZE * 4);
    tBDButtonPGMParameterStruct.aValue = TYPE_CURRENT;
    tBDButtonPGMParameterStruct.aPGMText = F("Ampere");
    TouchButtonOnlyTextAmpere.init(&tBDButtonPGMParameterStruct);
    TouchButtonOnlyTextAmpere.setButtonTextColor(CHART_CURRENT_COLOR);

// Settings for Text messages
    BlueDisplay1.setWriteStringPosition(VALUES_POSITION_X, MESSAGE_START_POSITION_Y);
    BlueDisplay1.setWriteStringSizeAndColorAndFlag(sChartDataTextSize, sTextColor, sBackgroundColor, false);
}

void initBatteryChart() {

    for (uint16_t i = 0; i < MAX_NUMBER_OF_SAMPLES; ++i) {
        sChartValueArray[i] = 0;
    }
    /*
     * For CHART_X_AXIS_SCALE_FACTOR_1             and at 5 minutes / pixel we have 12 hours labels each 144 pixel -> 8 segments.
     * For CHART_X_AXIS_SCALE_FACTOR_COMPRESSION_2 and at 5 minutes / pixel we have 24 hours labels each 288 pixel.
     * For CHART_X_AXIS_SCALE_FACTOR_EXPANSION_2   and at 5 minutes / pixel we have  6 hours labels each  72 pixel.
     * SECS_PER_DAY/2 increment for label value
     */
    VoltageChart.setXDataScaleFactor(CHART_MAXIMUM_X_SCALE_FACTOR);

// Label increment is 30 min for scale factor 1 so we have 5:37 for complete chart
    VoltageChart.initXLabel(0, CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED, CHART_X_AXIS_SCALE_FACTOR_1, 3, 0);
    VoltageChart.setXLabelDistance(2); // normal x scale, 1 is default
    VoltageChart.setLabelStringFunction(VoltageChart.convertMinutesToString);

    uint16_t tChartHeight = BlueDisplay1.getRequestedDisplayHeight() - (BlueDisplay1.getRequestedDisplayHeight() / 4); // 3/4 display height
    uint16_t tYGridSize = (BlueDisplay1.getRequestedDisplayHeight() / 10); // 34
//    sChartMaxValue = ((tChartHeight * (CHART_Y_LABEL_INCREMENT / CO2_COMPRESSION_FACTOR)) / (tYGridSize));

// Grid spacing is CHART_WIDTH / 8 -> 8 columns and height (261) / 6 for 5 lines from 400 to 1400
    VoltageChart.initChart(CHART_START_X, BlueDisplay1.getRequestedDisplayHeight() - (BASE_TEXT_SIZE + (BASE_TEXT_SIZE / 2)),
    CHART_WIDTH, tChartHeight, CHART_AXES_SIZE, BASE_TEXT_SIZE, CHART_DISPLAY_GRID, CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED,
            tYGridSize);

    VoltageChart.initChartColors(CHART_VOLTAGE_COLOR, CHART_AXES_COLOR, CHART_GRID_COLOR, sTextColor, sTextColor, sBackgroundColor);

    ResistanceAndCurrentChart = VoltageChart;

// 0.5 volt per grid, factor is 0.1 for an input of 10 for 1 Volt
    VoltageChart.initYLabel(0.0, 0.5, 0.1, 3, 1); //  0.5 volt per grid, 10 for input 1 (Volt) , "3.5" as label
    ResistanceAndCurrentChart.initYLabel(0.0, 200, 1, 4, 0); //  200 mA or mOhm per grid, "1200" as label
}

void clearAndDrawChart() {
    VoltageChart.clear();
    VoltageChart.drawXAxisAndLabels();
    VoltageChart.drawYAxisAndLabels();
    VoltageChart.drawGrid();
    /*
     * Text buttons are overwritten by chart
     */
    drawTextButtons();
}

/*
 * Text buttons are overwritten by chart
 */
void drawTextButtons() {
    TouchButtonOnlyTextVolt.drawButton();
    TouchButtonOnlyTextAmpere.drawButton();
    if (!sInLoggerModeAndFlags) {
        TouchButtonOnlyTextESR.drawButton();
    }
}

void drawButtons() {
    TouchButtonRunningState.drawButton();
    TouchButtonCutoffHighLowZero.drawButton();
    TouchButtonBatteryLogger.drawButton();
//    TouchButtonRedraw.drawButton();
    TouchButtonBrightness.drawButton();
    if (sMeasurementState == STATE_INITIAL_SAMPLES) {
        TouchButtonAppend.drawButton();
    }
}

void clearValueArea() {
    BlueDisplay1.fillRectRel(0, 0, BlueDisplay1.getRequestedDisplayWidth(), MESSAGE_START_POSITION_Y - 1, sBackgroundColor);
}

/*
 * Set sChartReadValueArrayType to the type of data for chart and call readAndProcessEEPROMData() to read and print it
 */
void readAndDrawEEPROMValues() {
//    VoltageChart.clear();
    sChartReadValueArrayType = TYPE_VOLTAGE; // handle voltages in next call
    readAndProcessEEPROMData(false);
    /*
     * Show ESR if we are just after boot and data stored was no logger data
     * OR we are just not after boot and in battery mode
     */
    if ((sMeasurementState == STATE_SETUP_AND_READ_EEPROM && !StartValues.inLoggerModeAndFlags)
            || (sMeasurementState != STATE_SETUP_AND_READ_EEPROM && !sInLoggerModeAndFlags)) {
        _delay(HELPFUL_DELAY_BETWEEN_DRAWING_CHART_LINES_TO_STABILIZE_USB_CONNECTION);
        sChartReadValueArrayType = TYPE_ESR; // handle ESR in next call
        readAndProcessEEPROMData(false);
    }
    _delay(HELPFUL_DELAY_BETWEEN_DRAWING_CHART_LINES_TO_STABILIZE_USB_CONNECTION);
    sChartReadValueArrayType = TYPE_CURRENT; // handle current in next call
    readAndProcessEEPROMData(sMeasurementState == STATE_SETUP_AND_READ_EEPROM); // StoreValuesForDisplayAndAppend if we are after boot

    /*
     * Print chart statistics/values, if we have more than just start data
     */
    if (ValuesForDeltaStorage.DeltaArrayIndex > 0) {
        printChartValues();
    }
}

void printCapacityValue() {
    char tString[18];
    if (isStandardCapacityAvailable) {
// 70 bytes
        snprintf_P(tString, sizeof(tString), PSTR("%5u | %u mAh"), sMeasurementInfo.CapacityMilliampereHour,
                sStandardCapacityMilliampereHour);
    } else {
        snprintf_P(tString, sizeof(tString), PSTR("%5u mAh        "), sMeasurementInfo.CapacityMilliampereHour);
    }
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, BlueDisplay1.getRequestedDisplayHeight() - (sChartDataTextSize * 9), tString,
            sChartDataTextSize, sTextColor, sBackgroundColor);
}

/*
 * Print all chart related values at the lower right
 */
void printChartValues() {
    char tStringBuffer[30];
    uint16_t tYPosition = (BlueDisplay1.getRequestedDisplayHeight() - (sChartDataTextSize * 10));

// Battery type
    const char *aBatteryTypePtr;
    if (sInLoggerModeAndFlags && StartValues.BatteryTypeIndex == TYPE_INDEX_NO_BATTERY) {
        aBatteryTypePtr = "Logger";
    } else {
        aBatteryTypePtr = BatteryTypeInfoArray[StartValues.BatteryTypeIndex].TypeName;
    }
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X + (BASE_TEXT_WIDTH * 6) - 3, tYPosition, aBatteryTypePtr, sChartDataTextSize,
            sTextColor, sBackgroundColor);

// Capacity
    printCapacityValue();

// Samples use 5u to have the same spacing as mAh
    snprintf(tStringBuffer, sizeof(tStringBuffer), "%5u Samples %u mn", ValuesForDeltaStorage.DeltaArrayIndex + 1,
            StartValues.NumberOfSecondsPerStorage / 60); // Samples + start sample
    tYPosition += 2 * sChartDataTextSize;
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, sTextColor, sBackgroundColor);

// Duration + VCC
    sVCCVoltageMillivolt = getVCCVoltageMillivolt();
    uint16_t tDurationMinutes = ValuesForDeltaStorage.DeltaArrayIndex
            * (StartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE);
    snprintf(tStringBuffer, sizeof(tStringBuffer), "%2u:%02u h       VCC", tDurationMinutes / 60, tDurationMinutes % 60);
    dtostrf(sVCCVoltageMillivolt / 1000.0, 5, 2, &tStringBuffer[8]);
    tStringBuffer[13] = ' '; // overwrite terminating null
    tYPosition += sChartDataTextSize;
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, sTextColor, sBackgroundColor);

    snprintf(tStringBuffer, sizeof(tStringBuffer), "%2u:%02u h", tDurationMinutes / 60, tDurationMinutes % 60);
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, sTextColor, sBackgroundColor);

// Load
    dtostrf(StartValues.LoadResistorMilliohm / 1000.0, 5, 2, tStringBuffer);
    snprintf(&tStringBuffer[5], sizeof(tStringBuffer), " \x81 Load"); // strcat(tStringBuffer, " m\x82 Load") requires more program space
    tYPosition += sChartDataTextSize;
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, sTextColor, sBackgroundColor);

// Voltage
    dtostrf(StartValues.initialDischargingMillivolt / 1000.0, 5, 2, tStringBuffer);
    snprintf(&tStringBuffer[5], sizeof(tStringBuffer), " V ->      V");
    dtostrf(sLastChartData.Millivolt / 1000.0, 5, 2, &tStringBuffer[10]);
    tStringBuffer[15] = ' '; // overwrite terminating null
    tYPosition += sChartDataTextSize;
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, CHART_VOLTAGE_COLOR,
            sBackgroundColor);

// ESR
    if (!sInLoggerModeAndFlags) {
        snprintf(tStringBuffer, sizeof(tStringBuffer), "%5u m\x81->%4u m\x81", StartValues.initialDischargingMilliohm,
                sLastChartData.ESRMilliohm);
        tYPosition += sChartDataTextSize;
        BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, CHART_ESR_COLOR,
                sBackgroundColor);
    }

// Current
    snprintf(tStringBuffer, sizeof(tStringBuffer), "%5u mA->%4u mA", StartValues.initialDischargingMilliampere,
            sLastChartData.Milliampere);
    tYPosition += sChartDataTextSize;
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, CHART_CURRENT_COLOR,
            sBackgroundColor);
}

#endif // defined(SUPPORT_BLUEDISPLAY_CHART)

/*
 * Version 5.1 - 3/2024
 *  - Adaptive Chart data text size.
 *  - Improved RunningState BDButton handling.
 *
 * Version 5.0 - 2/2024
 *  - Compression is now done by simply doubling the sampling period, which results in reducing the resolution from 336 of 168 samples directly after compression.
 *  - Data and chart can be displayed on (old) tablets or mobile running the BlueDisplay app https://github.com/ArminJo/Arduino-BlueDisplay.
 *  - Plotter pin logic does not depend any more on USB powering.
 *  - Tested logger function with chart.
 *
 * Version 4.0 - 2/2024
 *  - Use capacity between NominalFullVoltageMillivolt and CutoffVoltageMillivoltHigh as standard capacity to enable better comparison.
 *  - If powered by USB plotter pin logic is reversed, i.e. plotter output is enabled if NOT connected to ground.
 *  - In state detecting battery, you can toggle cut off voltage between high, low and zero (0.1 V) with stop button.
 *  - Fix bug for appending to compressed data.
 *  - Synchronizing of LCD access for button handler, avoiding corrupted display content.
 *  - Print improvements.
 *  - Support for storage period of 120 s.
 *  - Compression improved for rapidly descending voltage.
 *  - Moving seldom used function of pin 10 to pin A5.
 *  - New Logger mode with separate shunt enabled by pin 10.
 *  - Store data in an array of structure instead in 3 arrays.
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
