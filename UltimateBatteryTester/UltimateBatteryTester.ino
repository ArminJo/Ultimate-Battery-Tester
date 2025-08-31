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

#define VERSION_EXAMPLE "5.2"
// The change log is at the bottom of the file

//#define TRACE
//#define DEBUG

//#define NO_TONE_WARNING_FOR_VOLTAGE_TOO_LOW_FOR_STANDARD_CAPACITY_COMPUTATION

/*
 * If you want, you can calibrate your ADC readout by replacing this value with the voltage you measured a the AREF pin after the program started.
 * For my Nanos I measured e.g. 1060 mV and 1093 mV.
 */
#if !defined(ADC_INTERNAL_REFERENCE_MILLIVOLT)
#define ADC_INTERNAL_REFERENCE_MILLIVOLT    1100L // Change to value measured at the AREF pin. If value > real AREF voltage, measured values are > real values
#endif

/*
 * Enables output of values and the discharge graph on a tablet or mobile running the BlueDisplay app.
 * Disables many Serial output to save program memory space.
 */
//#define SUPPORT_BLUEDISPLAY_CHART
#if defined(SUPPORT_BLUEDISPLAY_CHART)
// This requires 1304 bytes program memory and exceeds 100% in Arduino IDE, because the "-mrelax" linker option is not set there.
//#define ENABLE_DISPLAY_OF_DATE_AND_TIME
#endif
//
/*
 * Activate the type of LCD you use
 * Default is parallel LCD with 2 rows of 16 characters (1602).
 * 2004 Display is and will not be supported. Use BlueDisplay app instead.
 * Serial LCD uses A4/A5 - the hardware I2C pins on Arduino
 */
//#define USE_NO_LCD
#if !defined(USE_SERIAL_LCD) && !defined(USE_PARALLEL_LCD) && !defined(USE_NO_LCD)
// Choose your default LCD type, I have a parallel LCD
#define USE_PARALLEL_LCD
//#define USE_SERIAL_LCD
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
#define CUTOFF_LEVEL_PIN            11 // If connected to ground, CUTOFF_LEVEL_LOW is taken as startup default if no battery is inserted.
#define LOAD_LOW_PIN                12 // This pin is high to switch on the low load (10 ohm). A4 is occupied by I2C for serial LCD display.
bool sLastValueOfCutoffLevelPin; // To support prints and voltage setting at changing between normal and low by using pin CUTOFF_LEVEL_PIN

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

/*
 * Flags for logger mode
 */
#define NO_LOGGER_MODE_REQUESTED            0
#define LOGGER_MODE_REQUESTED               1
#define LOGGER_EXTERNAL_CURRENT_DETECTED    2
#define LOGGER_EXTERNAL_VOLTAGE_DETECTED    4
uint8_t sInLoggerModeAndFlags = NO_LOGGER_MODE_REQUESTED; // Initially contains the (inverted) value of the pin ONLY_LOGGER_MODE_PIN

/*
 * Cutoff level
 */
#define CUTOFF_LEVEL_HIGH       0 // Switch off current percentage is 50% (shift 1 right) for logger. Is default case
#define CUTOFF_LEVEL_LOW        1 // 25% (shift 2 right) for logger.
#define CUTOFF_LEVEL_ZERO       2 // 12% (shift 3 right) for logger.

/*
 * Cutoff detection is done by comparing current mA value with low pass mA value
 * in sLastMilliampereLowPassFiltered5 shifted by 1 or 2 or 3 (divide by 2 or 4 or 8) corresponding to 50% or 25% or 12.5%
 */
uint16_t sLastMilliampereLowPassFiltered5;      // For sCutoffMilliamperePercent

/*********************
 * Measurement timing
 *********************/
#define MILLIS_IN_ONE_SECOND 1000L
#define SECONDS_IN_ONE_MINUTE 60L
#define SECONDS_IN_ONE_MINUTE_SHORT 60
#define MINUTES_IN_ONE_HOUR_SHORT 60
//#define TEST // to speed up testing the code
#if defined(TEST)
#define NUMBER_OF_INITIAL_SAMPLES               4 // 4 seconds (-2 for initial display of append message) before starting discharge and storing, to have time to just test for ESR of battery.
#define SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS   500L // The time of the activated load for one sample.
#define INITIAL_NUMBER_OF_SECONDS_PER_STORAGE   5 // 1 minute, if we have 1 sample per second
#else
#define NUMBER_OF_INITIAL_SAMPLES               30 // Before starting discharge and storing, to have time to just test for ESR of battery. 30 seconds with SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS as 1000.
#define SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS   MILLIS_IN_ONE_SECOND // 1 s. The time of the activated load for one sample.
/*
 * Using minutes instead of seconds and avoiding the divide by 60 at many places increases code size by 46 bytes :-(
 */
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

/*******************
 * Attention timing
 *******************/
#define STATE_BATTERY_DETECTION_ATTENTION_PERIOD_MILLIS     (MILLIS_IN_ONE_SECOND * SECONDS_IN_ONE_MINUTE)
#define STATE_STOP_ATTENTION_PERIOD_MILLIS                  (MILLIS_IN_ONE_SECOND * SECONDS_IN_ONE_MINUTE * 10)

/************************
 * Tester state machine
 ***********************/
#define STATE_SETUP_AND_READ_EEPROM          0
#define STATE_WAITING_FOR_BATTERY_OR_EXTERNAL 1 // Check if battery is inserted and determine type or external current or voltage is connected to start with logging
#define STATE_INITIAL_SAMPLES                2 // Only voltage (and ESR) measurement every n seconds for NUMBER_OF_INITIAL_SAMPLES samples
#define STATE_SAMPLE_AND_STORE_TO_EEPROM     3 // Main measurement state, get values and store to EEPROM
#define STATE_STOPPED                        4 // Switch off voltage reached, until removal of battery

/***************************
 * Support for BlueDisplay
 ***************************/
#if defined (DOXYGEN)
#define SUPPORT_BLUEDISPLAY_CHART // always document it :-)
#endif
#if defined(SUPPORT_BLUEDISPLAY_CHART)
//#define DO_NOT_NEED_BASIC_TOUCH_EVENTS
#define DO_NOT_NEED_LONG_TOUCH_DOWN_AND_SWIPE_EVENTS  // Disables LongTouchDown and SwipeEnd events.
#define DO_NOT_NEED_SPEAK_EVENTS            // Disables SpeakingDone event handling. Saves up to 54 bytes program memory and 18 bytes RAM.
#define ONLY_CONNECT_EVENT_REQUIRED         // Disables reorientation, redraw and SensorChange events
#define SUPPRESS_SERIAL_PRINT               // To reduce code size

#include "BlueDisplay.hpp" // part of https://github.com/ArminJo/Arduino-BlueDisplay
//#define BLUETOOTH_BAUD_RATE BAUD_115200   // Activate this, if you have reprogrammed the HC05 module for 115200
#  if !defined(BLUETOOTH_BAUD_RATE)
#define BLUETOOTH_BAUD_RATE     9600        // Default baud rate of my HC-05 modules, which is not very reactive
#  endif

#  if defined ENABLE_DISPLAY_OF_DATE_AND_TIME
#include "BDTimeHelper.hpp"
#  endif // ENABLE_DISPLAY_OF_DATE_AND_TIME

/*
 * Scale the screen such, that this fit horizontally.
 * Border - YLabels - Chart with CO2_ARRAY_SIZE / 2 - Border - Buttons for 6 big characters - Border
 * Take border as CO2_ARRAY_SIZE / 20, button width as  4 * CO2_ARRAY_SIZE / 20 and base font size as CO2_ARRAY_SIZE / 40
 */
#define DISPLAY_WIDTH   ((MAX_NUMBER_OF_SAMPLES * 33L) / 20L) // 556
#define BASE_TEXT_SIZE  (MAX_NUMBER_OF_SAMPLES / 20L) // 16
#define BASE_TEXT_WIDTH ((((MAX_NUMBER_OF_SAMPLES / 20L) * 6 ) + 4) / 10) // 10
#define BUTTON_WIDTH    (BASE_TEXT_SIZE * 5) // 80
#define CHART_START_X   (BASE_TEXT_SIZE * 3) // 48
#define CHART_WIDTH     (MAX_NUMBER_OF_SAMPLES + 1) // 337, +1 for the first sample at minute 0 -> 337, 5 hours and 36 min
#define CHART_AXES_SIZE (BASE_TEXT_SIZE / 8) // 2
#define BUTTONS_START_X ((BASE_TEXT_SIZE * 4) + CHART_WIDTH)

#define PROBE_VALUES_TEXT_SIZE      (BASE_TEXT_SIZE * 2)
#define PROBE_VALUES_POSITION_Y     (BASE_TEXT_SIZE / 2)
#define PROBE_VALUES_POSITION_X     (BASE_TEXT_SIZE * 2)
#define MESSAGE_START_POSITION_Y    ((BASE_TEXT_SIZE * 2) + (BASE_TEXT_SIZE / 2))

#define VOLTAGE_POSITION_X          (PROBE_VALUES_POSITION_X)
#define ESR_POSITION_X              (PROBE_VALUES_POSITION_X + (BASE_TEXT_SIZE * 20))
#define CURRENT_POSITION_X          (PROBE_VALUES_POSITION_X + (BASE_TEXT_SIZE * 10))

#define CHART_VALUES_POSITION_X     (CHART_START_X + CHART_WIDTH)

#define CHART_VOLTAGE_COLOR     COLOR16_RED
#define CHART_ESR_COLOR         COLOR16_GREEN
#define CHART_CURRENT_COLOR     COLOR16_BLUE

#define CHART_AXES_COLOR        COLOR16_BLUE
#define CHART_GRID_COLOR        COLOR16_YELLOW
#define CHART_DATA_COLOR        COLOR16_RED
#define CHART_TEXT_COLOR        COLOR16_BLACK

#define CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED 30L
#define SECONDS_PER_MINUTE                     60L

/*
 * Brightness handling
 */
#define BRIGHTNESS_LOW      2
#define BRIGHTNESS_MIDDLE   1
#define BRIGHTNESS_HIGH     0
#define START_BRIGHTNESS    BRIGHTNESS_HIGH
uint8_t sCurrentBrightness = START_BRIGHTNESS;
color16_t sBackgroundColor = COLOR16_WHITE; // for brightness
color16_t sTextColor = COLOR16_BLACK; // for brightness

/*
 * Chart handling
 */
Chart VoltageChart;
//Chart ResistanceAndCurrentChart; // not yet supported
uint8_t sChartDataTextSize;

#define TYPE_VOLTAGE    0
#define TYPE_ESR        1
#define TYPE_CURRENT    2
#define TYPE_NO_DATA    3 // is 8 bytes shorter than TYPE_NO_DATA    0
uint8_t sChartReadValueArrayType; // 0 = voltage, 1 = ESR, 2 = current
//uint8_t sChartDisplayValueArrayType; // 0 = voltage, 1 = ESR, 2 = current - not yet implemented

/*
 * Factor to convert the original mV, mOhm, mA to the 8 bit chart array data. The offset is subtracted before compression.
 * Compression and offset values ate transfered to host, to expand data.
 * sCompressionFactor 10 means maximum (delta) value is 2.55 V, A, Ohm.
 * We have 7 grids for Y axis and use a 1, 2, 5 scheme.
 * In order to achieve optimal resolution for factor 10, it is necessary to select a 200 mV grid.
 * This selection enables the display of a maximum of 1.4 V / 1400 mV (input is then 140).
 * For voltages that exceed 1.4 V a the next higher compression of 25 and a scale value of 500 mV grid
 * is required to ensure optimal resolution.
 * This selection enables the display of a maximum of 3.5 V.
 * sCompressionFactor 50 -> 1 V grid and 7 V maximum range.
 * Minimum sensible compression factor is 5 -> 100 mV grid, 700 mV range.
 * Maximum sensible compression factor is 250 -> 5 V grid, 35 V range.
 */
uint16_t sCompressionFactor;
uint16_t sChartCompressionFactor; // (initial) factor for mV to chart array data, required to compute sCompressionFactor for ESR and current.
uint16_t sCompressionOffsetMillivolt; // Value to subtract from millivolt before compressing

/*
 * Buttons
 */
const char MeasurementStateButtonStringBooting[] PROGMEM = "Booting";
const char MeasurementStateButtonStringWaiting[] PROGMEM = "Waiting";
const char MeasurementStateButtonStringTesting[] PROGMEM = "Testing";
const char MeasurementStateButtonStringRunning[] PROGMEM = "Running";
const char MeasurementStateButtonStringStopped[] PROGMEM = "Stopped"; // stopped manually
const char MeasurementStateButtonStringFinished[] PROGMEM = "Finished"; // stopped by detecting end condition
const char *const sMeasurementStateButtonTextStringArray[] PROGMEM = { MeasurementStateButtonStringBooting,
        MeasurementStateButtonStringWaiting, MeasurementStateButtonStringTesting, MeasurementStateButtonStringRunning,
        MeasurementStateButtonStringStopped, MeasurementStateButtonStringFinished };
BDButton TouchButtonMeasurementState;
BDButton TouchButtonAppend;

void setMeasurementStateButtonTextAndDrawButton();
void setMeasurementStateAndBDButtonText(uint8_t aMeasurementState); // calls setMeasurementStateButtonTextAndDrawButton();

const char CutoffButtonStringHigh[] PROGMEM = "Cutoff High";
const char CutoffButtonStringLow[] PROGMEM = "Cutoff Low";
const char CutoffButtonStringZero[] PROGMEM = "Cutoff Zero";
const char *const sCutoffButtonTextStringArray[] PROGMEM = { CutoffButtonStringHigh, CutoffButtonStringLow, CutoffButtonStringZero };
BDButton TouchButtonCutoffHighLowZero;
void setCutoffHighLowZeroButtonTextAndDrawButton();

BDButton TouchButtonBatteryLogger;
//BDButton TouchButtonRedraw;
BDButton TouchButtonBrightness; // Brightness handling costs 400 byte
BDButton TouchButtonOnlyTextVolt;
BDButton TouchButtonOnlyTextESR;
BDButton TouchButtonOnlyTextAmpere;

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

/*******
 * LCD
 *******/
#define LCD_MESSAGE_PERSIST_TIME_MILLIS     2000 // 2 second to view a message on LCD
#if defined(USE_SERIAL_LCD)
#define USE_SOFT_I2C_MASTER // Requires SoftI2CMaster.h + SoftI2CMasterConfig.h. Saves 2110 bytes program memory and 200 bytes RAM compared with Arduino Wire
#include "LiquidCrystal_I2C.hpp"  // This defines USE_SOFT_I2C_MASTER, if SoftI2CMasterConfig.h is available. Use only the modified version delivered with this program!
#endif
#if defined(USE_PARALLEL_LCD)
#include "LiquidCrystal.h"
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

/*********************************************
 * Battery types and their measurement values
 *********************************************/
struct BatteryTypeInfoStruct {
    const char TypeName[11];
    uint16_t DetectionThresholdVoltageMillivolt; // Type is detected if voltage is below this threshold
    uint16_t NominalFullVoltageMillivolt;        // The voltage to start the "standard" capacity computation
    uint16_t CutoffVoltageMillivoltHigh; // The voltage to stop the "standard" capacity computation. Cut off happens below this voltage
    uint16_t CutoffVoltageMillivoltLow;  // Switch off voltage for extended capacity measurement
    uint16_t CutoffVoltageMillivoltZero; // Switch off voltage for destructive capacity measurement
    uint8_t LoadType;                    // High (3 Ohm) or low (12 Ohm)
    uint16_t LoadSwitchSettleTimeMillis; // Time for voltage to settle after load switch was disabled => time of NoLoad during one sample
};

#define NO_BATTERY_MILLIVOLT                         70 // 50 mV
#define NO_LOGGER_MILLAMPERE                         12

#define LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT          4300 // Maximum Voltage if fully loaded
#define LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT     4100 // Start voltage for Li-ion standard capacity measurement
#define LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT        3400 // Switch off voltage for Li-ion standard capacity measurement
#define LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW    3000 // Switch off voltage for extended capacity measurement
#define LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO    180

#define NIMH_STANDARD_FULL_VOLTAGE_MILLIVOLT       1340 // Start voltage for NI-MH standard capacity measurement
#define NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT          1100 // Switch off voltage for NI-MH capacity measurement
#define NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW      1000 // Switch off voltage for extended capacity measurement
#define NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO      140 // witch off voltage for extended capacity measurement

#define NO_LOAD     0
#define LOW_LOAD    1 // 12 ohm
#define HIGH_LOAD   2 // 3 ohm

#define TYPE_INDEX_NO_BATTERY    0
#define TYPE_INDEX_DEFAULT       6
#define TYPE_INDEX_MAX          10
#define TYPE_INDEX_LOGGER       42

struct BatteryTypeInfoStruct BatteryTypeInfoArray[] = { { "No battery", NO_BATTERY_MILLIVOLT, 0, 0, 0, 0, NO_LOAD, 0 }, /* Below 100 mV and not below 50, to avoid toggling between no and low batt */
{ "Low batt. ", 1000, 800, 300, 100, 70, HIGH_LOAD, 100 }, /* For researching of worn out batteries. */
// @formatter:off
{ "NiCd NiMH ", 1460, NIMH_STANDARD_FULL_VOLTAGE_MILLIVOLT, NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT,
NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW, NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO, HIGH_LOAD, 100 }, /*400 mA*/
{ "Alkali    ", 1550, 1500, 1300, 1000, 70, HIGH_LOAD, 100 }, /*500 mA*/
{ "NiZn batt.", 1850, 1650, 1400, 1300, 100, HIGH_LOAD, 100 }, /*550 mA*/
{ "LiFePO4   ", 3400, 3400, 3050, 2700, 180, LOW_LOAD, 10 }, /*270 mA https://www.jackery.com/blogs/knowledge/ultimate-guide-to-lifepo4-voltage-chart*/
{ "Li-ion    ", 5000, LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT /*4100*/, LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT/*3400*/,
LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW/*3V*/, LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO /*180mV*/,
LOW_LOAD, 10 }, /*300 mA*/
{ "LiIo 2pack", 2 * LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT, 2 * LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT,
        2 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT /*7V*/, 2 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW /*6V*/,
        2 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO /*360mV*/, LOW_LOAD, 10 }, /*620 mA*/
{ "9 V Block ", 9200, 9000, 7700, 7000, 200, LOW_LOAD, 100 }, /*750 mA => external series load resistor recommended*/
{ "LiIo 3pack", 3 * LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT, 3 * LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT,
        3 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT, 3 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW,
        3 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO, LOW_LOAD, 10 }, /*925 mA*/
{ "LiIo 4pack", 4 * LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT, 4 * LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT,
        4 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT, 4 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW,
        4 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO, LOW_LOAD, 10 } /*1233 mA*/
// @formatter:on
        };

/*******************************************
 * Battery values set by getBatteryValues()
 *******************************************/
struct BatteryOrLoggerInfoStruct {
    union VoltagesUnion {
        struct BatteryVoltagesStruct {
            uint16_t NoLoadMillivolt; // initially and periodically stored to EEPROM
            uint16_t LoadMillivolt;
        } Battery;
        struct LoggerVoltagesStruct {
            uint16_t AverageMillivolt; // Same memory location as NoLoadMillivolt and therefore stored in EEPROM
#if !defined(SUPPRESS_SERIAL_PRINT)
            uint16_t MinimumMillivolt; // only used for Serial output
#endif
            uint16_t MaximumMillivolt;
        } Logger;
    } Voltages;
    uint16_t Milliampere;
    uint16_t ESRMilliohm; // Average of last 60 values. ESR - Equivalent Series Resistor | internal battery resistance.
    uint16_t ESRDeltaMillivolt; // only displayed at initial ESR testing
    uint32_t CapacityAccumulator;
    uint16_t CapacityMilliampereHour;

    uint8_t LoadState; // NO_LOAD | LOW_LOAD 12 ohm | HIGH_LOAD 3 ohm

    uint8_t BatteryTypeIndex;
    uint8_t CutoffLevel; // One of CUTOFF_LEVEL_HIGH, CUTOFF_LEVEL_LOW and CUTOFF_LEVEL_ZERO. Starts with the (inverted) value of the pin CUTOFF_LEVEL_PIN
    char CutoffLevelCharacter;
    uint16_t CutoffVoltageMillivolt;

    uint8_t LoadSwitchSettleTimeMillis; // One of CUTOFF_LEVEL_HIGH, CUTOFF_LEVEL_LOW and CUTOFF_LEVEL_ZERO. Starts with the (inverted) value of the pin CUTOFF_LEVEL_PIN
} sBatteryOrLoggerInfo;

struct lastDiplayedValuesStruct {
    uint16_t VoltageNoLoadMillivolt;
    uint16_t Milliampere;
    uint16_t ESRMilliohm;
    uint16_t CapacityMilliampereHour;
} sLastDiplayedValues;

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

struct TesterInfoStruct {
    volatile uint8_t MeasurementState; // One of STATE_SETUP_AND_READ_EEPROM, STATE_WAITING_FOR_BATTERY_OR_EXTERNAL, STATE_INITIAL_SAMPLES etc.
    bool BatteryOrCurrentOrVoltageWasDetected; // set by detectBatteryOrLoggerVoltageOrCurrentLCD_BD() if detection was successful

#if defined(SUPPORT_BLUEDISPLAY_CHART)
    bool MeasurementWasFinishedByEndCondition; // true if stopped by detecting end condition and not by user button.
#endif

    bool VoltageRangeIsLow;    // true for 2.2 V range

    /*
     * Standard capacity is from NominalFullVoltageMillivolt to CutoffVoltageMillivoltHigh.
     * It is used to compare capacities independent of initial charge voltage and cutoff voltages.
     * It is computed while reading data from EEPROM.
     * Not used for logger.
     */
    uint16_t StandardCapacityMilliampereHour;
    bool isStandardCapacityAvailable;

    uint8_t NumbersOfInitialSamplesToGo; // STATE_INITIAL_SAMPLES countdown count

    // Display control
    bool ButtonUsageMessageWasPrinted;
    bool VoltageNoLoadIsDisplayedOnLCD;

    // Sample timing
    unsigned long LastMillisOfLoggerSample;
    unsigned long LastMillisOfBatteryOrVoltageDetection;

    uint16_t SampleCountForStoring; // Storage time of samples is determined by this count and not by system time!
    unsigned long LastMillisOfSample;

    //Attention timing
    unsigned long LastMillisOfStateWaitingForBatteryOrVoltageBeep;
    unsigned long LastMillisOfStateStoppedForAttentionBeep;

} sTesterInfo;

/*
 * Load and ESR history arrays
 * Current value is in sCurrentLoadResistorHistory[0]. Used for computing and storing the average.
 */
#define HISTORY_SIZE_FOR_LOAD_RESISTOR_AVERAGE 16
uint16_t sCurrentLoadResistorHistory[HISTORY_SIZE_FOR_LOAD_RESISTOR_AVERAGE];
uint16_t sCurrentLoadResistorAverage;
/*
 * Current value is in sESRHistory[0]. The average is stored in sBatteryOrLoggerInfo.ESRMilliohm.
 * For 9V and 60 mA we have a resolution of 0.3 ohm so we need an average.
 */
#define HISTORY_SIZE_FOR_ESR_AVERAGE    60
uint16_t sESRHistory[HISTORY_SIZE_FOR_ESR_AVERAGE];

// Override defaults defined in ADCUtils.h
#define LI_ION_VCC_UNDERVOLTAGE_THRESHOLD_MILLIVOLT 3500 // 3.5 volt
#define VCC_CHECK_PERIOD_MILLIS                     (60000L) // check every minute
#define VCC_UNDERVOLTAGE_CHECKS_BEFORE_STOP         5 // Shutdown after 5 times below VCC_UNDERVOLTAGE_THRESHOLD_MILLIVOLT or below VCC_EMERGENCY_UNDERVOLTAGE_THRESHOLD_MILLIVOLT
#define LOCAL_INFO // For Serial output at isVCCUndervoltageMultipleTimes(). This is undefined after the include!
#include "ADCUtils.hpp"

/************************************************************************************************
 * EEPROM store, It seems that EEPROM is allocated top down and in called or referenced sequence
 * https://arduino.stackexchange.com/questions/93873/how-eemem-maps-the-variables-avr-eeprom-h
 ************************************************************************************************/
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
    uint8_t NumberOfSecondsPerStorage; // INITIAL_NUMBER_OF_SECONDS_PER_STORAGE (60) and multiple like 120, 240, etc.
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

struct ValuesForDeltaStorageStruct {
    uint16_t lastStoredVoltageNoLoadMillivolt;
    uint16_t lastStoredMilliampere;
    uint16_t lastStoredMilliohm;
    int DeltaArrayIndex; // The index of the next values to be written. -1 to signal, that start values must be written.
} ValuesForDeltaStorage;

#if defined(SUPPORT_BLUEDISPLAY_CHART)
// Must be below definition of MAX_NUMBER_OF_SAMPLES :-(
uint8_t sChartValueArray[MAX_NUMBER_OF_SAMPLES] __attribute__((section(".noinit"))); // must be in noinit, to be first overwritten on stack overflow
uint16_t sChartValueArrayIndex;
struct ChartValuesStruct {
    uint16_t Millivolt;
    uint16_t Milliampere;
    uint16_t ESRMilliohm;
} sLastChartData;
/*
 * Values are initialized at setup and at storing of first value to EEPROM
 */
struct ValuesForChartScaling {
    uint16_t minVoltageNoLoadMillivolt;
    uint16_t maxVoltageNoLoadMillivolt;
    uint16_t maxMilliampere;
    uint16_t maxMilliohm;
} ValuesForChartScaling;
#endif // defined(SUPPORT_BLUEDISPLAY_CHART)

#define MAX_VOLTAGE_DROP_MILLIVOLT_FOR_COUNTDOWN        6
#define VOLTAGE_DISPLAY_HYSTERESIS_MILLIVOLT_FOR_STOP   5
#define VOLTAGE_DISPLAY_HYSTERESIS_MILLIVOLT            1
#define CURRENT_DISPLAY_HYSTERESIS_MILLIAMPERE          1
#define ESR_DISPLAY_HYSTERESIS_MILLIOHM                 1

void getBatteryOrLoggerVoltageMillivolt();
void addToCapacity();
uint16_t getBatteryOrLoggerRawVoltage();
void setBatteryTypeIndex(uint8_t aBatteryTypeIndex);
bool setBatteryTypeIndexFromVoltage(uint16_t aBatteryVoltageMillivolt);
void detectBatteryOrLoggerVoltageOrCurrentLCD_BD();
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
void printStoredDataLCD_BD();
void printMilliampere4DigitsLCD_BD();
void printVoltageNoLoadMillivoltWithTrailingSpaceLCD_BD();
void clearLastDiplayedValues();
void printMeasurementValuesLCD_BD();
void printValuesForPlotterAndChart(uint16_t aMillivoltToPrint, uint16_t aMilliampereToPrint, uint16_t aMilliohmToPrint,
        bool aIsLastElement);
void printMillisValueAsFloat(uint16_t aValueInMillis);
void printCounterLCD_BD(uint16_t aNumberToPrint);

void dumpEEPROM(uint8_t *aEEPROMAdress, uint8_t aNumberOf16ByteBlocks);
void storeBatteryValuesToEEPROM(uint16_t aVoltageNoLoadMillivolt, uint16_t aMilliampere, uint16_t aMilliohm);
void storeCapacityAndCutoffLevelToEEPROM_LCD();
void readAndProcessEEPROMData(bool aInitializeValuesForDisplayAndAppend);
void handlePeriodicStoringToEEPROM();
void handleStateStopped();
void handleEndOfStateInitialSamples();
void handlePeriodicDetectionOfProbe();
void checkAndHandleVCCUndervoltage();

void getOnlyPlotterOutputPinLevel();
void setCutoffAndCutoffVoltage(uint8_t aCutoffLevel);
void setCutoffAndCutoffVoltageFromPinLevel(bool aValueOfCutoffLevelPin);
void checkAndHandleCutoffPinLevelChange();
void debugPrintCutoffInfo();

void delayAndCheckForButtonPress();
void printButtonUsageMessageLCD();
void printCutoffLevelLCD_BD();
void setNextCutoffLevelAndPrint();
void printlnIfNotPlotterOutput();
void printStateString(uint8_t aState);

void switchToStateWaitingForBatteryOrVoltage();
void switchToStateInitialSamples();
void switchToStateSampleAndStoreToEEPROM(uint16_t aInitialSampleCountForStoring);
void switchToStateStopped(char aReasonCharacter);

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
//#define LOCAL_TRACE // This enables TRACE output only for this file and not for libraries used
#endif
#if defined(DEBUG)
#define LOCAL_DEBUG
#else
//#define LOCAL_DEBUG // This enables debug output only for this file and not for libraries used
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

    // Pin settings for sTesterInfo.VoltageRangeIsLow = false;
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
    getOnlyPlotterOutputPinLevel();

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
        Serial.print(((MAX_NUMBER_OF_SAMPLES) * (INITIAL_NUMBER_OF_SECONDS_PER_STORAGE / SECONDS_IN_ONE_MINUTE)) / MINUTES_IN_ONE_HOUR_SHORT);
        Serial.print(F("h "));
        Serial.print(((MAX_NUMBER_OF_SAMPLES) * (INITIAL_NUMBER_OF_SECONDS_PER_STORAGE / SECONDS_IN_ONE_MINUTE)) % MINUTES_IN_ONE_HOUR_SHORT);
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
    /*
     * Get EEPROM data for append and scaling the chart, which is drawn below at BlueDisplay1.initCommunication() by the connectHandler.
     * true -> do not print, just initialize values, especially cutoff level from last measurement for easy append.
     */
    readAndProcessEEPROMData(true);

    if (!sOnlyPlotterOutput) {
        BlueDisplay1.initCommunication(&Serial, &connectHandler); // introduces up to 1.5 seconds delay
    }
#endif

#if defined(ENABLE_STACK_ANALYSIS)
        printRAMInfo(&Serial);
#  if !defined(BD_USE_SIMPLE_SERIAL)
        Serial.flush();
#  endif
#endif

#if defined(USE_LCD)
    _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

#  if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (!BlueDisplay1.isConnectionEstablished()) {
#  endif
        myLCD.setCursor(0, 1);
        if (sOnlyPlotterOutput) {
            myLCD.print(F("Only plotter out"));
        } else {
            myLCD.print(F("No plotter out  "));
        }
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS / 2);

        myLCD.setCursor(0, 1);
        if (sInLoggerModeAndFlags) {
#  if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Only logger mode"));
        }
#  endif
            myLCD.print(F("Only logger mode"));
            _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
        }
        LCDClearLine(1); // Clear line "No plotter out  " or "Only logger mode"
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
     * Get and print EEPROM data in any case
     */
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (!BlueDisplay1.isConnectionEstablished()) {
        // If no connection, no chart will be displayed, thus print Arduino logger data to Serial output here to serve as info.
        readAndProcessEEPROMData(false);
    }
#else
    readAndProcessEEPROMData(true); // true, initialize data for append. Print, because SUPPORT_BLUEDISPLAY_CHART is not defined.
#endif

    printStoredDataLCD_BD();
    printlnIfNotPlotterOutput(); // end of stored data

    if (sInLoggerModeAndFlags) {
        sCurrentLoadResistorAverage = LOGGER_SHUNT_RESISTOR_MILLIOHM;
        sBatteryOrLoggerInfo.ESRMilliohm = 0; // not used in logger function
    }

    /*
     * Read value to variable in order to force printing triggered by value change :-)
     */
    sLastValueOfCutoffLevelPin = digitalRead(CUTOFF_LEVEL_PIN);

    /*
     * If battery is still inserted, keep cut off level read from EEPROM data for easy append.
     * If battery was removed, cut off level can be chosen by pressing stop button.
     */
    getBatteryOrLoggerVoltageMillivolt();
    if (sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt < NO_BATTERY_MILLIVOLT) {
        // Battery / Logger is removed here, so start with cutoff value determined by pin
        sBatteryOrLoggerInfo.BatteryTypeIndex = TYPE_INDEX_NO_BATTERY; // Required for correct CutoffVoltage
        setCutoffAndCutoffVoltageFromPinLevel(sLastValueOfCutoffLevelPin);
    }
    printCutoffLevelLCD_BD(); // print actual cut off level

    if (sTesterInfo.isStandardCapacityAvailable) {
        /*
         * Display standard capacity info a few seconds later, but only at startup
         */
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            if (sTesterInfo.isStandardCapacityAvailable) {
                Serial.print(F("Standard "));
            }
            Serial.print(F("capacity="));
            Serial.print(sTesterInfo.StandardCapacityMilliampereHour);
            Serial.println(F(" mAh"));
        }
#endif
#if defined(USE_LCD)
        myLCD.setCursor(7, 1);
        myLCD.print(F("s "));
        char tString[6];
        snprintf_P(tString, sizeof(tString), PSTR("%5u"), sTesterInfo.StandardCapacityMilliampereHour);
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

    if (sTesterInfo.MeasurementState == STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
        handlePeriodicDetectionOfProbe();
    } else {
        /*
         * STATE_INITIAL_SAMPLES / Testing or STATE_SAMPLE_AND_STORE_TO_EEPROM / Running or STATE_STOPPED here
         */
        if (sInLoggerModeAndFlags) {
            handlePeriodicAccumulatingLoggerValues();
        }
        auto tMillis = millis();
        // For discharging, add LoadSwitchSettleTimeMillis to the second condition
        if ((sInLoggerModeAndFlags && sLogger1SecondAccumulator.RawSampleCount == LOGGER_SAMPLE_FREQUENCY_HZ)
                || (!sInLoggerModeAndFlags
                        && (unsigned) (tMillis - sTesterInfo.LastMillisOfSample)
                                >= (SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS + sBatteryOrLoggerInfo.LoadSwitchSettleTimeMillis))) {
            /*
             * Here sample period (one second) expired
             * sTesterInfo.MeasurementState is STATE_INITIAL_SAMPLES or STATE_SAMPLE_AND_STORE_TO_EEPROM or STATE_STOPPED
             * Do all this every second (of battery load)
             */
            sTesterInfo.LastMillisOfSample = tMillis;

            /*
             * Get values
             */
            if (sInLoggerModeAndFlags) {
                getLogger1SecondValues();
            } else {
                getBatteryValues();
            }

            if (sTesterInfo.MeasurementState == STATE_STOPPED) {
                // Print only tVoltageNoLoadMillivolt and check for periodic attention
                handleStateStopped();
                if (sInLoggerModeAndFlags) {
                    /*
                     * In logger mode continue to print voltage and current values on LCD and chart
                     */
                    if (sBatteryOrLoggerInfo.Milliampere >= NO_LOGGER_MILLAMPERE) {
#if defined(USE_LCD)
                        myLCD.setCursor(7, 0);
                        myLCD.print("---"); // Overwrite "Finished" and signal with "---", that we are stopped now
#endif
                        printMilliampere4DigitsLCD_BD();
                    }
                    if (sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt > NO_BATTERY_MILLIVOLT) {
                        printVoltageNoLoadMillivoltWithTrailingSpaceLCD_BD();
                    }
                }

            } else {
                /*
                 * STATE_INITIAL_SAMPLES or STATE_SAMPLE_AND_STORE_TO_EEPROM here
                 * Check stop or remove conditions and display values
                 */
                if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES && isVoltageOrCurrentRemoved()) {
                    switchToStateWaitingForBatteryOrVoltage(); // switch back to start and do not overwrite already displayed values
                } else {
                    // Here not stopped or just removed -> Print measurement values
                    addToCapacity();
                    printMeasurementValuesLCD_BD();

                    // For battery storing to EEPROM, we want to store the value of the stop condition in EEPROM, so do not check here.
                    if (sInLoggerModeAndFlags) {
                        checkAndHandleStopConditionLCD(); // Only Logger mode: Check every second if current drops.
                    }
                }
                /*
                 * Check for end of STATE_INITIAL_SAMPLES
                 */
                if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES && sTesterInfo.NumbersOfInitialSamplesToGo == 0) {
                    handleEndOfStateInitialSamples();
                }

                if (sTesterInfo.MeasurementState == STATE_SAMPLE_AND_STORE_TO_EEPROM) {
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

        getOnlyPlotterOutputPinLevel();
    }

    checkAndHandleCutoffPinLevelChange();

} // end of loop()

/*
 * Check for plotter mode pin change
 * Is not really required for logger and sets reference to default,
 * which requires additional 8 ms for logger to switch back to 4.4 V range
 */
void getOnlyPlotterOutputPinLevel() {
    sOnlyPlotterOutput = !digitalRead(ONLY_PLOTTER_OUTPUT_PIN);
    if (isVCCUSBPowered()) {
        sOnlyPlotterOutput = !sOnlyPlotterOutput; // reversed behavior if powered by USB
    }
}

/*
 * Check for VCC undervoltage during measurements
 */
void checkAndHandleVCCUndervoltage() {
    if (sTesterInfo.MeasurementState != STATE_STOPPED && isVCCUndervoltageMultipleTimes()) {
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
    if (millis() - sTesterInfo.LastMillisOfBatteryOrVoltageDetection >= BATTERY_OR_VOLTAGE_DETECTION_PERIOD_MILLIS) {
        sTesterInfo.LastMillisOfBatteryOrVoltageDetection = millis();

        /*
         * Check if battery was inserted or voltage connected
         */
        setLoad(NO_LOAD);
        detectBatteryOrLoggerVoltageOrCurrentLCD_BD();
        // we waited up to 2 seconds in detectBatteryOrLoggerVoltageOrCurrentLCD_BD(), so must check if mode has not changed by button press
        if (sTesterInfo.MeasurementState == STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
            if (sTesterInfo.BatteryOrCurrentOrVoltageWasDetected) {
                /*
                 * Successfully detected here -> switch state and show values detected
                 */
                switchToStateInitialSamples();

#if !defined(SUPPORT_BLUEDISPLAY_CHART)
                // If found, print button usage once at start of InitialESRMeasurement, but not, if display is attached, which has attach button
                if (!sTesterInfo.ButtonUsageMessageWasPrinted) {
                    printButtonUsageMessageLCD();
                    sTesterInfo.ButtonUsageMessageWasPrinted = true;
                    sTesterInfo.VoltageNoLoadIsDisplayedOnLCD = false;
                }
#  if defined(USE_LCD)
                LCDClearLine(1); // Clear line "append to EEPROM"
#  endif
#endif
                sTesterInfo.NumbersOfInitialSamplesToGo = NUMBER_OF_INITIAL_SAMPLES;
                memset(sESRHistory, 0, sizeof(sESRHistory));
                sLastDiplayedValues.Milliampere = 0; // to force overwrite of value
                if (sInLoggerModeAndFlags) {
                    /*
                     * Initialize logger accumulators
                     */
                    sLastMilliampereLowPassFiltered5 = sBatteryOrLoggerInfo.Milliampere;
                    clearLogger1SecondAccumulator();
                    clearLogger1MinuteAccumulator();
                } else {
                    // set load for the first call of getBatteryValues() to measure the current
                    setLoad(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].LoadType);
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
                if (sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt > NO_BATTERY_MILLIVOLT) {
                    printVoltageNoLoadMillivoltWithTrailingSpaceLCD_BD();
                    tNoLoadVoltageOrCurrentPrinted = true;
                } else if (!sTesterInfo.VoltageNoLoadIsDisplayedOnLCD) {
#if defined(USE_LCD)
                    LCDPrintVCC(0);
#endif
                }

                if (tNoLoadVoltageOrCurrentPrinted || sBatteryOrLoggerInfo.Milliampere > 0) {
                    /*
                     * Here U or I is valid, but not both
                     */
                    printMilliampere4DigitsLCD_BD();
                    tNoLoadVoltageOrCurrentPrinted = true;
                }

                if (tNoLoadVoltageOrCurrentPrinted) {
                    printlnIfNotPlotterOutput();
                }
                /*
                 * if not connected to USB, check for attention every minute
                 */
                if (!isVCCUSBPowered() && millis() - sTesterInfo.LastMillisOfStateWaitingForBatteryOrVoltageBeep >= STATE_BATTERY_DETECTION_ATTENTION_PERIOD_MILLIS) {
                    sTesterInfo.LastMillisOfStateWaitingForBatteryOrVoltageBeep = millis();
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
            && sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt
                    < BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].NominalFullVoltageMillivolt) {
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            Serial.print(F("Start voltage "));
            Serial.print(sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt);
            Serial.print(F(" V is below NominalFullVoltageMillivolt of "));
            Serial.print(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].NominalFullVoltageMillivolt);
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

// If button was not pressed again, start a new data set
    if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES) {
        /*
         * Force new data set, no append here
         */
        ValuesForDeltaStorage.DeltaArrayIndex = -1;
        sBatteryOrLoggerInfo.CapacityAccumulator = 0;
        // Must reset this values here, because values are displayed before computed again from CapacityAccumulator
        sBatteryOrLoggerInfo.CapacityMilliampereHour = 0;
        memset(sCurrentLoadResistorHistory, 0, sizeof(sCurrentLoadResistorHistory)); // Clear history array
        // Store first EEPROM value immediately, append is done by button and waits a full period
        switchToStateSampleAndStoreToEEPROM(INITIAL_NUMBER_OF_SECONDS_PER_STORAGE);
    }
}

/*
 * Print only tVoltageNoLoadMillivolt if it changed more than 5 mV and check for periodic attention
 */
void handleStateStopped() {
    if (abs( (int16_t )sLastDiplayedValues.VoltageNoLoadMillivolt
            - (int16_t )sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt) > VOLTAGE_DISPLAY_HYSTERESIS_MILLIVOLT_FOR_STOP) {
        printVoltageNoLoadMillivoltWithTrailingSpaceLCD_BD();
        printlnIfNotPlotterOutput();
    }
    /*
     * Check for attention every 10 minute, after the current measurement was finished
     */
    if (millis() - sTesterInfo.LastMillisOfStateStoppedForAttentionBeep >= STATE_STOP_ATTENTION_PERIOD_MILLIS) {
        sTesterInfo.LastMillisOfStateStoppedForAttentionBeep = millis();
        playAttentionTone();
    }
}

void handlePeriodicStoringToEEPROM() {
    sTesterInfo.SampleCountForStoring++;
//    Serial.print(F(" |"));
//    Serial.print(sTesterInfo.SampleCountForStoring);
    /*
     * Check for periodic storage to EEPROM
     */
    if (((sTesterInfo.SampleCountForStoring * MILLIS_IN_ONE_SECOND) / SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS)
            >= StartValues.NumberOfSecondsPerStorage) { // Will be optimized by compiler :-)
        sTesterInfo.SampleCountForStoring = 0;

        if (sInLoggerModeAndFlags) {
            getLogger1MinuteValues(); // get the smoother values here for storing to EEPROM
        }
        if (sTesterInfo.MeasurementState != STATE_STOPPED) {
            /*
             * Store to EEPROM, and then read from EEPROM for drawing of chart
             */
            storeBatteryValuesToEEPROM(sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt, sBatteryOrLoggerInfo.Milliampere,
                    sBatteryOrLoggerInfo.ESRMilliohm);
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
        if (sTesterInfo.MeasurementState == STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
            Serial.print(F("WAITING FOR BATTERY OR VOLTAGE"));
        } else if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES) {
            Serial.print(F("INITIAL SAMPLES"));
        } else if (sTesterInfo.MeasurementState == STATE_SAMPLE_AND_STORE_TO_EEPROM) {
            Serial.print(F("STORE TO EEPROM"));
        } else if (sTesterInfo.MeasurementState == STATE_STOPPED) {
            Serial.print(F("STOPPED"));
        }
    }
#endif
}

/*
 * sTesterInfo.MeasurementState is state before state change i.e. state, which is left
 */
void checkLeavingState() {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (BlueDisplay1.isConnectionEstablished()) {
        if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES) {
            TouchButtonAppend.removeButton(sBackgroundColor);
        }
    }
#endif
}

void setMeasurementStateAndBDButtonText(uint8_t aMeasurementState) {
    sTesterInfo.MeasurementState = aMeasurementState;
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    setMeasurementStateButtonTextAndDrawButton();
#endif
}

void switchToStateWaitingForBatteryOrVoltage() {
    setMeasurementStateAndBDButtonText(STATE_WAITING_FOR_BATTERY_OR_EXTERNAL);
    printSwitchStateString();
    printlnIfNotPlotterOutput();
    sTesterInfo.LastMillisOfStateWaitingForBatteryOrVoltageBeep = millis();
    sBatteryOrLoggerInfo.BatteryTypeIndex = TYPE_INDEX_MAX + 2; // to force display of "found ...", but do not set button text
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    sTesterInfo.MeasurementWasFinishedByEndCondition = false; // reset flag
#endif
}

/*
 * Exclusively called by handlePeriodicDetectionOfProbe()
 * Only state switching here, other values are set in handlePeriodicDetectionOfProbe()
 */
void switchToStateInitialSamples() {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (BlueDisplay1.isConnectionEstablished()) {
        TouchButtonAppend.drawButton();
    }
#endif
    setMeasurementStateAndBDButtonText(STATE_INITIAL_SAMPLES);
    printSwitchStateString();
    printlnIfNotPlotterOutput();
}

/*
 * @param aInitialSampleCountForStoring - if 0 then next sample is stored after a full sample period,
 *                                        if >= NumberOfSecondsPerStorage then sample is stored directly
 */
void switchToStateSampleAndStoreToEEPROM(uint16_t aInitialSampleCountForStoring) {
    sTesterInfo.SampleCountForStoring = aInitialSampleCountForStoring;
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    checkLeavingState(); // remove append button
    clearValueArea();    // to remove display of count
#endif
    clearLastDiplayedValues(); // To force display of all values after LCD messages
    setMeasurementStateAndBDButtonText(STATE_SAMPLE_AND_STORE_TO_EEPROM);
    printSwitchStateString();
    printlnIfNotPlotterOutput();

    // cutoff level may have been changed for append
    eeprom_update_byte(&EEPROMStartValues.CutoffLevel, sBatteryOrLoggerInfo.CutoffLevel);
}

/*
 * aWriteToLCD default is true.
 * @param aReasonCharacter, '-' for terminating condition met (regular end of measurement), U for VCC undervoltage, F for EEPROM full,
 *                           D for button double press, B for button press.
 */
void switchToStateStopped(char aReasonCharacter) {
    checkLeavingState();
    setLoad(NO_LOAD);
    auto tOldMeasurementState = sTesterInfo.MeasurementState;
    setMeasurementStateAndBDButtonText(STATE_STOPPED);

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

/*******************
 * Cutoff functions
 *******************/
void LCDPrintCutoff() {
    LCDResetCursor();
    myLCD.print(F("Cutoff "));
    if (sInLoggerModeAndFlags) {
        myLCD.print(F("is "));
        myLCD.print(100 >> (sBatteryOrLoggerInfo.CutoffLevel + 1)); // 50, 25, 12
        myLCD.print(F(" % I"));
    } else {
        auto tCutoffLevel = sBatteryOrLoggerInfo.CutoffLevel;
        if (sBatteryOrLoggerInfo.BatteryTypeIndex == TYPE_INDEX_NO_BATTERY) {
            // Long text without voltage
            myLCD.print(F("is "));
            if (tCutoffLevel == CUTOFF_LEVEL_LOW) {
                myLCD.print(F("low   "));
            } else if (tCutoffLevel == CUTOFF_LEVEL_ZERO) {
                myLCD.print(F("zero  "));
            } else { // CUTOFF_LEVEL_HIGH
                myLCD.print(F("high  "));
            }

        } else {
            uint16_t tSwitchOffVoltageMillivolt = sBatteryOrLoggerInfo.CutoffVoltageMillivolt;
            uint8_t tDecimals = 1;
            // Short text with voltage e.g. "Cutoff high 3.5V"
            if (tCutoffLevel == CUTOFF_LEVEL_LOW) {
                myLCD.print(F("low"));
                tDecimals = 2;
            } else if (tCutoffLevel == CUTOFF_LEVEL_ZERO) {
                if (tSwitchOffVoltageMillivolt < 1000) {
                    myLCD.print(F("z "));
                    // print "z  0.070V" instead of "zero 1.1V"
                    tDecimals = 3;
                } else {
                    myLCD.print(F("zero"));
                }
            } else { // CUTOFF_LEVEL_HIGH
                myLCD.print(F("high"));
            }
            if (10000 > tSwitchOffVoltageMillivolt) {
                myLCD.print(' ');
            }
            myLCD.print(tSwitchOffVoltageMillivolt / 1000.0, tDecimals);
            myLCD.print('V');
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
 * Toggle cut off to normal, to low and to zero, set cutoff voltage and print
 */
void setNextCutoffLevelAndPrint() {
    auto tCutoffLevel = sBatteryOrLoggerInfo.CutoffLevel;
    tCutoffLevel++;
    if (tCutoffLevel > CUTOFF_LEVEL_ZERO) {
        tCutoffLevel = CUTOFF_LEVEL_HIGH;
    }
    setCutoffAndCutoffVoltage(tCutoffLevel);
    printCutoffLevelLCD_BD();
}

/*
 * Set sBatteryOrLoggerInfo.CutoffLevelCharacter
 *     sBatteryOrLoggerInfo.CutoffLevel
 *     sBatteryOrLoggerInfo.CutoffVoltageMillivolt
 *     StartValues.CutoffLevel
 * Requires a valid sBatteryOrLoggerInfo.BatteryTypeIndex
 */
void setCutoffAndCutoffVoltage(uint8_t aCutoffLevel) {
    uint16_t tCutoffVoltageMillivolt;
    char tCutoffLevelCharacter;
    if (aCutoffLevel == CUTOFF_LEVEL_LOW) {
        tCutoffLevelCharacter = 'l';
        tCutoffVoltageMillivolt = BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].CutoffVoltageMillivoltLow;
    } else if (aCutoffLevel == CUTOFF_LEVEL_ZERO) {
        tCutoffLevelCharacter = 'z';
        tCutoffVoltageMillivolt = BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].CutoffVoltageMillivoltZero;
    } else {
        tCutoffLevelCharacter = 'h';
        tCutoffVoltageMillivolt = BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].CutoffVoltageMillivoltHigh;
    }
    /*
     * Update structure
     */
    sBatteryOrLoggerInfo.CutoffLevel = aCutoffLevel;
    sBatteryOrLoggerInfo.CutoffLevelCharacter = tCutoffLevelCharacter;
    sBatteryOrLoggerInfo.CutoffVoltageMillivolt = tCutoffVoltageMillivolt; // the only place CutoffVoltageMillivolt is set
}

void setCutoffAndCutoffVoltageFromPinLevel(bool aValueOfCutoffLevelPin) {
    uint8_t tCutoffLevel = CUTOFF_LEVEL_HIGH; // default
    if (!aValueOfCutoffLevelPin) {
        tCutoffLevel = CUTOFF_LEVEL_LOW;
    } else if (sInLoggerModeAndFlags) {
        // For logger mode default cut off level is zero instead of high
        tCutoffLevel = CUTOFF_LEVEL_ZERO;
    }
    setCutoffAndCutoffVoltage(tCutoffLevel);
}

void checkAndHandleCutoffPinLevelChange() {
    bool tValueOfCutoffLevelPin = digitalRead(CUTOFF_LEVEL_PIN);
    if (sLastValueOfCutoffLevelPin != tValueOfCutoffLevelPin) {
        sLastValueOfCutoffLevelPin = tValueOfCutoffLevelPin;
        setCutoffAndCutoffVoltageFromPinLevel(tValueOfCutoffLevelPin);
        printCutoffLevelLCD_BD();
    }
}
void debugPrintCutoffInfo() {
    Serial.print(F("CutoffLevel="));
    Serial.print(sBatteryOrLoggerInfo.CutoffLevel);
    Serial.print(F("|"));
    Serial.print(sBatteryOrLoggerInfo.CutoffLevelCharacter);
    Serial.print(F(" Type="));
    Serial.print(sBatteryOrLoggerInfo.BatteryTypeIndex);
    Serial.print(F(" -> "));
    Serial.print(sBatteryOrLoggerInfo.CutoffVoltageMillivolt);
    Serial.println(F("mV"));
}
/*
 * Print state of cut off level
 * One of CUTOFF_LEVEL_HIGH, CUTOFF_LEVEL_LOW and CUTOFF_LEVEL_ZERO
 * For Logger: CUTOFF_LEVEL_HIGH = 50%, LOW = 25% and ZERO = 12.5%
 */
void printCutoffLevelLCD_BD() {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    setCutoffHighLowZeroButtonTextAndDrawButton();
#endif
#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        if (sInLoggerModeAndFlags) {
            Serial.print(F("End logging below "));
            Serial.print(100 >> (sBatteryOrLoggerInfo.CutoffLevel + 1));
            Serial.println(F(" % of last current"));

        } else {
            auto tCutoffLevel = sBatteryOrLoggerInfo.CutoffLevel;
            if (tCutoffLevel == CUTOFF_LEVEL_ZERO) {
                Serial.println(F("Cut off at 50 mV"));
            } else {
                if (sBatteryOrLoggerInfo.BatteryTypeIndex == TYPE_INDEX_NO_BATTERY) {
                    if (tCutoffLevel == CUTOFF_LEVEL_LOW) {
                        Serial.println(F("Cut off at low voltage. e.g. 3000 mV for Li-ion"));
                    } else { // CUTOFF_LEVEL_HIGH
                        Serial.println(F("Cut off at high voltage. e.g. 3450 mV for Li-ion"));
                    }
                } else {
                    if (tCutoffLevel == CUTOFF_LEVEL_LOW) {
                        Serial.print(F("Cut off at low voltage "));
                        Serial.print(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].CutoffVoltageMillivoltLow);
                    } else { // CUTOFF_LEVEL_HIGH
                        Serial.print(F("Cut off at high voltage "));
                        Serial.print(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].CutoffVoltageMillivoltHigh);
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
    uint8_t tOldMeasurementState = sTesterInfo.MeasurementState;
    for (uint_fast8_t i = 0; i < 10; ++i) {
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS / 10);
        if (sTesterInfo.MeasurementState != tOldMeasurementState) {
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
    uint8_t tOldMeasurementState = sTesterInfo.MeasurementState;
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

    if (sTesterInfo.MeasurementState == tOldMeasurementState) {
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
    if (sTesterInfo.BatteryOrCurrentOrVoltageWasDetected /* battery was inserted before */
            && ((sBatteryOrLoggerInfo.Milliampere == 0 /* no current */
            && (!sInLoggerModeAndFlags || (sInLoggerModeAndFlags & LOGGER_EXTERNAL_CURRENT_DETECTED)))
                    || (sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt < NO_BATTERY_MILLIVOLT /* no voltage */
                    && (!sInLoggerModeAndFlags || (sInLoggerModeAndFlags & LOGGER_EXTERNAL_VOLTAGE_DETECTED))))) {
//#if !defined(SUPPRESS_SERIAL_PRINT) // around 100 bytes program space
        if (!sOnlyPlotterOutput) {
            Serial.print(F("Battery or voltage removing detected. U="));
            Serial.print(sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt);
            Serial.print(F(" mV I="));
            Serial.print(sBatteryOrLoggerInfo.Milliampere);
            Serial.println(F(" mA"));
        }
//#endif
        sTesterInfo.BatteryOrCurrentOrVoltageWasDetected = false;
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

    if (sTesterInfo.MeasurementState == STATE_SETUP_AND_READ_EEPROM) {
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
        printStateString(sTesterInfo.MeasurementState);
        Serial.println();
    }
#endif

    if (tIsDoublePress && sTesterInfo.MeasurementState != STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
        /*
         * Double press detected!
         * Go to STATE_STOPPED
         */
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Double press detected"));
        }
#endif
        switchToStateStopped('D');

    } else {
        /*
         * Single press here
         * Attention, this press can be the first press of a double press,
         * so we must wait 2 seconds and check for double press before processing single press
         */
        if (sTesterInfo.MeasurementState == STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
            setNextCutoffLevelAndPrint(); // In this state we can change the cutoff level by button :-)

        } else if (sTesterInfo.MeasurementState == STATE_SAMPLE_AND_STORE_TO_EEPROM) {
            switchToStateStopped('B'); // no check for double press required here :-)

        } else {
            /*
             * Only state STATE_INITIAL_SAMPLES and STATE_STOPPED left here!
             * Print "Start again" or "Append to EEPROM", and wait for 2 seconds for double press detection, which cancels the action
             */
            uint8_t tOldMeasurementState = sTesterInfo.MeasurementState;
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
                sTesterInfo.VoltageNoLoadIsDisplayedOnLCD = false;
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
             * which means sTesterInfo.MeasurementState can now also have value STATE_STOPPED here
             */
            delayAndCheckForButtonPress();

// Must check old value (before possible double press) in order to avoid switching from STATE_STOPPED to DetectingBattery at each double press.
            if (tOldMeasurementState == STATE_STOPPED) {
                // start a new measurement cycle
                switchToStateWaitingForBatteryOrVoltage();

            } else if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES) {
                /*
                 * No stop requested during 2 seconds wait -> append to EEPROM not canceled
                 * Store next sample in 60 seconds, because we assume double press directly after entering state STATE_INITIAL_SAMPLES
                 * Otherwise we would start the appended data with a short sampling period.
                 */
                setBatteryTypeIndex(StartValues.BatteryTypeIndex); // Restore original type index. This sets cutoff level accordingly
                switchToStateSampleAndStoreToEEPROM(0);
            }
        }
#if defined(LOCAL_DEBUG)
        if (!sOnlyPlotterOutput) {
            Serial.print(F("New state="));
            printStateString(sTesterInfo.MeasurementState);
            Serial.println();
        }
#endif
    }
}

void setLoad(uint8_t aNewLoadState) {
    if (sBatteryOrLoggerInfo.LoadState != aNewLoadState) {
        sBatteryOrLoggerInfo.LoadState = aNewLoadState;

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

/*
 * Set NoLoadMillivolt or LoadMillivolt depending on sBatteryOrLoggerInfo.LoadState
 */
void getBatteryOrLoggerVoltageMillivolt() {

    uint16_t tInputVoltageRaw = getBatteryOrLoggerRawVoltage();
    /*
     * Compute voltage
     */
    uint16_t tCurrentBatteryVoltageMillivolt;
    if (sTesterInfo.VoltageRangeIsLow) {
        tCurrentBatteryVoltageMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * ATTENUATION_FACTOR_VOLTAGE_LOW_RANGE)
                * tInputVoltageRaw) / 1023);
    } else {
        tCurrentBatteryVoltageMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE)
                * tInputVoltageRaw) / 1023);
    }

    if (sBatteryOrLoggerInfo.LoadState == NO_LOAD) {
        sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt = tCurrentBatteryVoltageMillivolt;
    } else {
        sBatteryOrLoggerInfo.Voltages.Battery.LoadMillivolt = tCurrentBatteryVoltageMillivolt;
    }

#if defined(LOCAL_TRACE)
    if (!sOnlyPlotterOutput) {
        Serial.print(tCurrentBatteryVoltageMillivolt);
        Serial.println(F(" mV"));
    }
#endif
}

void setToLowVoltageRange() {
    sTesterInfo.VoltageRangeIsLow = true;
    pinMode(VOLTAGE_RANGE_EXTENSION_PIN, INPUT);
    digitalWrite(VOLTAGE_RANGE_EXTENSION_PIN, LOW);
#if defined(LOCAL_TRACE)
    if (!sOnlyPlotterOutput) {
        Serial.println(F(" -> switch to 2.2 V range"));
    }
#endif
}

void setToHighVoltageRange() {
    sTesterInfo.VoltageRangeIsLow = false;
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
    if (sTesterInfo.VoltageRangeIsLow && tInputVoltageRaw >= 0x3F0) { // 1008
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
    if (!sTesterInfo.VoltageRangeIsLow) {
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
    Serial.println(sTesterInfo.VoltageRangeIsLow);
#endif
    return tInputVoltageRaw;
}

void addToCapacity() {
    if (sBatteryOrLoggerInfo.Milliampere > 1) {
        // Capacity computation
        sBatteryOrLoggerInfo.CapacityAccumulator += sBatteryOrLoggerInfo.Milliampere;
        sBatteryOrLoggerInfo.CapacityMilliampereHour = sBatteryOrLoggerInfo.CapacityAccumulator
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
//    sBatteryOrLoggerInfo.Voltages.Logger.MinimumMillivolt = UINT16_MAX;
    sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt = 0;
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
    sBatteryOrLoggerInfo.Milliampere = ((((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) / 1023L)
            * sLogger1SecondAccumulator.RawCurrentAccumulator)
            / (LOGGER_SHUNT_RESISTOR_MILLIOHM * LOGGER_SAMPLE_FREQUENCY_HZ * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT));

    sLastMilliampereLowPassFiltered5 += ((sLastMilliampereLowPassFiltered5 - sLastMilliampereLowPassFiltered5) + (1 << 4)) >> 5; // 2.5 us, alpha = 1/32 0.03125, cutoff frequency 5.13 Hz @1kHz

    /*
     * Compute voltage and avoid overflow
     * >> 8 and * 4 in divisor are a fast and short way to divide by 1024
     */
    sBatteryOrLoggerInfo.Voltages.Logger.AverageMillivolt =
            ((ADC_INTERNAL_REFERENCE_MILLIVOLT * (sLogger1SecondAccumulator.RawVoltageAccumulator >> 8))
                    / ((LOGGER_SAMPLE_FREQUENCY_HZ * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT * 4)
                            / ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE));
//    This gives overflow :-(
//    sBatteryOrLoggerInfo.Voltages.Logger.AverageMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE)
//            * (uint32_t) sLogger1SecondAccumulator.RawVoltageAccumulator >> 8)
//            / (LOGGER_SAMPLE_FREQUENCY_HZ * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT * 4L));

    sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT
            * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE) * (uint32_t) sLogger1SecondAccumulator.MaximumRawVoltage) / 1024L);
//    sBatteryOrLoggerInfo.Voltages.Logger.MinimumMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE)
//            * (uint32_t) sLogger1SecondAccumulator.MinimumRawVoltage) / 1024L);

#if defined(LOCAL_TRACE)
    Serial.print(F("cnt="));
    Serial.print(sLogger1SecondAccumulator.RawSampleCount);
    Serial.print(F(" Iacc="));
    Serial.print(sLogger1SecondAccumulator.RawCurrentAccumulator);
    Serial.print(' ');
    Serial.print(sBatteryOrLoggerInfo.Milliampere);
    Serial.print(F(" mA, Uacc="));
    Serial.print(sLogger1SecondAccumulator.RawVoltageAccumulator);
    Serial.print(F(", Umin="));
    Serial.print(sBatteryOrLoggerInfo.Voltages.Logger.MinimumMillivolt);
    Serial.print(F(" | "));
    Serial.print(sLogger1SecondAccumulator.MinimumRawVoltage);
    Serial.print(F(", Umax="));
    Serial.print(sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt);
    Serial.print(F(" | "));
    Serial.print(sLogger1SecondAccumulator.MaximumRawVoltage);
    Serial.print(F(", Uav="));
    Serial.print(sBatteryOrLoggerInfo.Voltages.Logger.AverageMillivolt);
    Serial.print(F(" mV Range="));
    Serial.println(sTesterInfo.VoltageRangeIsLow);
#endif

    clearLogger1SecondAccumulator();
}

void getLogger1MinuteValues() {
// avoid overflow
    sBatteryOrLoggerInfo.Milliampere = ((((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) / 1023L)
            * (sLogger1MinuteAccumulator.RawCurrentAccumulator / LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT))
            / (sLogger1MinuteAccumulator.RawSampleCount * LOGGER_SHUNT_RESISTOR_MILLIOHM));

    /*
     * Compute voltage and avoid overflow
     * Instead of (sLogger1MinuteRawVoltageAccumulator8ShiftRight << 8) / 1024 which would give overflow
     * we do (sLogger1MinuteRawVoltageAccumulator8ShiftRight >> 8) and divide the divisor by 2^6 (64)
     */
    sBatteryOrLoggerInfo.Voltages.Logger.AverageMillivolt = (ADC_INTERNAL_REFERENCE_MILLIVOLT
            * (sLogger1MinuteAccumulator.RawVoltageAccumulator8ShiftRight >> 8))
            / (((uint32_t) sLogger1MinuteAccumulator.RawSampleCount * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT)
                    / (ATTENUATION_FACTOR_VOLTAGE_HIGH_RANGE * 64));

#if defined(LOCAL_TRACE)
    Serial.print(F("cnt="));
    Serial.print(sLogger1MinuteRawSampleCount);
    Serial.print(F(" Iacc="));
    Serial.print(sLogger1MinuteRawCurrentAccumulator);
    Serial.print(' ');
    Serial.print(sBatteryOrLoggerInfo.Milliampere);
    Serial.print(F(" mA, Uacc="));
    Serial.print(sLogger1MinuteRawVoltageAccumulator8ShiftRight);
    Serial.print(' ');
    Serial.println(sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt);
    Serial.print(F(" mV l="));
    Serial.println(sTesterInfo.VoltageRangeIsLow);
#endif

    clearLogger1MinuteAccumulator();
}

/*
 * Sampling of 20ms takes 40.5 ms. For voltage > 4.4V it takes 48.5 ms
 */
void handlePeriodicAccumulatingLoggerValues() {
    if ((unsigned) (millis() - sTesterInfo.LastMillisOfLoggerSample) >= LOGGER_SAMPLE_PERIOD_MILLIS) {
        sTesterInfo.LastMillisOfLoggerSample = millis();
        digitalWrite(LED_BUILTIN, HIGH);

        if (sTesterInfo.MeasurementState != STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
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
                if (sTesterInfo.VoltageRangeIsLow) {
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
        if (sTesterInfo.VoltageRangeIsLow) {
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
    sBatteryOrLoggerInfo.Milliampere = (((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) * tShuntVoltageRaw)
            / (1023L * aShuntResistorMilliohm));
#if defined(LOCAL_TRACE)
    Serial.print(F("Ch "));
    Serial.print(aADCChannel);
    Serial.print(F(", Raw="));
    Serial.print(tShuntVoltageRaw);
    Serial.print(F(", "));
    Serial.print(sBatteryOrLoggerInfo.Milliampere);
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
 * ESRDeltaMillivolt
 * ESRMilliohm
 * sCurrentLoadResistorAverage
 *
 */
void getBatteryValues() {
// Do it before deactivating the load
    getCurrent(ADC_CHANNEL_CURRENT, ESR_SHUNT_RESISTOR_MILLIOHM);
    getBatteryOrLoggerVoltageMillivolt();    // get current battery load voltage (no load in case of stopped)

    if (sTesterInfo.MeasurementState == STATE_STOPPED) return; // thats all if stopped :-)

// Deactivate load and wait for voltage to settle
// During the no load period switch on the LED
    setLoad(NO_LOAD);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(sBatteryOrLoggerInfo.LoadSwitchSettleTimeMillis);
    getBatteryOrLoggerVoltageMillivolt();    // get current battery NoLoadMillivolt
// restore original load state
    setLoad(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].LoadType);
    digitalWrite(LED_BUILTIN, LOW);

    /*
     * ESR computation
     */
    sBatteryOrLoggerInfo.ESRDeltaMillivolt = sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt
            - sBatteryOrLoggerInfo.Voltages.Battery.LoadMillivolt;

    if (sBatteryOrLoggerInfo.Milliampere > 1) {
        /*
         * Compute sESRAverage
         * Shift history array to end and insert current value at [0]
         */
        uint8_t tESRAverageHistoryCounter = 1; // below we add current value too
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
        uint32_t tESRMilliohm = (sBatteryOrLoggerInfo.ESRDeltaMillivolt * 1000L) / sBatteryOrLoggerInfo.Milliampere;
        if (tESRMilliohm > __UINT16_MAX__) {
            sESRHistory[0] = __UINT16_MAX__; // indicate overflow
        } else {
            sESRHistory[0] = tESRMilliohm;
        }

        tESRAverageAccumulator += sESRHistory[0];
        sBatteryOrLoggerInfo.ESRMilliohm = (tESRAverageAccumulator + (tESRAverageHistoryCounter / 2)) / tESRAverageHistoryCounter;

#if defined(LOCAL_TRACE)
        Serial.print(sESRHistory[0]);
        Serial.print('/');
        Serial.print(tESRAverageHistoryCounter);
        Serial.print('=');
        Serial.print(sBatteryOrLoggerInfo.ESRMilliohm);
        Serial.println();
#endif

        /*
         * Compute sCurrentLoadResistorAverage if array sCurrentLoadResistorHistory is full
         * Formula is: LoadVoltage / LoadCurrent and includes the MosFet and the connector resistance.
         * Shift load resistor history array and insert current value
         */
        uint32_t tLoadResistorAverage = 0;
        for (uint_fast8_t i = HISTORY_SIZE_FOR_LOAD_RESISTOR_AVERAGE - 1; i > 0; --i) {
            tLoadResistorAverage += sCurrentLoadResistorHistory[i - 1];
            sCurrentLoadResistorHistory[i] = sCurrentLoadResistorHistory[i - 1];
        }
        sCurrentLoadResistorHistory[0] = (sBatteryOrLoggerInfo.Voltages.Battery.LoadMillivolt * 1000L
                / sBatteryOrLoggerInfo.Milliampere);
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
        // use dirty hack to compute the cutoff current by shifting by cutoff level + 1
        if (sBatteryOrLoggerInfo.Milliampere < (sLastMilliampereLowPassFiltered5 >> (sBatteryOrLoggerInfo.CutoffLevel + 1))) {
            /*
             * Switch off current condition for logger met
             */
#if !defined(SUPPRESS_SERIAL_PRINT)
            if (!sOnlyPlotterOutput) {
                Serial.print(F("Switch off current percentage "));
                Serial.print(100 >> (sBatteryOrLoggerInfo.CutoffLevel + 1));
                Serial.print(F(" % mA of "));
                Serial.print(sLastMilliampereLowPassFiltered5);
                Serial.print(F(" mA reached, I="));
                Serial.print(sBatteryOrLoggerInfo.Milliampere);
                Serial.print(F(" mA, capacity="));
                Serial.print(sBatteryOrLoggerInfo.CapacityMilliampereHour);
                Serial.println(F(" mAh"));
            }
#endif
            tStopConditionIsMet = true;
        }

    } else {
        if (sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt < sBatteryOrLoggerInfo.CutoffVoltageMillivolt) {
            /*
             * Switch off voltage condition for battery met
             */
            if (!sOnlyPlotterOutput) {
                Serial.print(F("Switch off voltage "));
                Serial.print(sBatteryOrLoggerInfo.CutoffVoltageMillivolt);
                Serial.print(F(" mV reached, capacity="));
                Serial.print(sBatteryOrLoggerInfo.CapacityMilliampereHour);
                Serial.println(F(" mAh"));
            }
            tStopConditionIsMet = true;
        }
    }
    if (tStopConditionIsMet) {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
        sTesterInfo.MeasurementWasFinishedByEndCondition = true; // To show "Finished" instead of "Stopped" on button
#endif
        switchToStateStopped('-');
#if defined(USE_LCD)
        _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS); // show "stopped"
        myLCD.setCursor(7, 0);
        myLCD.print(F(" Finished"));
#endif
// Play short melody
        playEndTone();
    }
}

void setBatteryTypeIndex(uint8_t aBatteryTypeIndex) {
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
    sBatteryOrLoggerInfo.BatteryTypeIndex = aBatteryTypeIndex;
    sBatteryOrLoggerInfo.LoadSwitchSettleTimeMillis = BatteryTypeInfoArray[aBatteryTypeIndex].LoadSwitchSettleTimeMillis;
    setCutoffAndCutoffVoltage(sBatteryOrLoggerInfo.CutoffLevel); // set CutoffVoltageMillivolt, which depends on BatteryTypeIndex
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    setCutoffHighLowZeroButtonTextAndDrawButton();
#endif
}
/*
 * search the "database" for a matching type
 * @return true, if BatteryTypeIndex changed
 */
bool setBatteryTypeIndexFromVoltage(uint16_t aBatteryVoltageMillivolt) {

// scan all threshold voltage of all battery types
    uint_fast8_t tBatteryTypeIndex = 0;
    for (; tBatteryTypeIndex < sizeof(BatteryTypeInfoArray) / sizeof(BatteryTypeInfoStruct) - 1; tBatteryTypeIndex++) {
        if (aBatteryVoltageMillivolt < BatteryTypeInfoArray[tBatteryTypeIndex].DetectionThresholdVoltageMillivolt) {
            break; // If not found -> assume high voltage is detected
        }
    }
    if (sBatteryOrLoggerInfo.BatteryTypeIndex != tBatteryTypeIndex) {
        setBatteryTypeIndex(tBatteryTypeIndex);
        return true;
    }
    return false;
}

/*
 * Disables the load, measures the voltage to detecting battery type
 * For logger it measures the current for detection
 * @return true, if battery or logger voltage and current detected
 */
void detectBatteryOrLoggerVoltageOrCurrentLCD_BD() {
    getBatteryOrLoggerVoltageMillivolt();

    if (sInLoggerModeAndFlags) {
        /*
         * Logger here
         */
        getCurrent(ADC_CHANNEL_LOGGER_CURRENT, LOGGER_SHUNT_RESISTOR_MILLIOHM);
        if (sBatteryOrLoggerInfo.Milliampere >= NO_LOGGER_MILLAMPERE
                || sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt > NO_BATTERY_MILLIVOLT) {
            // External current or voltage found
            sTesterInfo.BatteryOrCurrentOrVoltageWasDetected = true;
            if (sBatteryOrLoggerInfo.Milliampere >= NO_LOGGER_MILLAMPERE) {
                sInLoggerModeAndFlags |= LOGGER_EXTERNAL_CURRENT_DETECTED;
            }
            if (sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt > NO_BATTERY_MILLIVOLT) {
                sInLoggerModeAndFlags |= LOGGER_EXTERNAL_VOLTAGE_DETECTED;
            }
#if !defined(SUPPRESS_SERIAL_PRINT)
            if (!sOnlyPlotterOutput) {
                Serial.print(F("Found U="));
                Serial.print(sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt);
                Serial.print(F(" mV, I="));
                Serial.print(sBatteryOrLoggerInfo.Milliampere);
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
        sBatteryOrLoggerInfo.Milliampere = 0; // avoid display old current value
        if (setBatteryTypeIndexFromVoltage(sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt)) {
            /*
             * BatteryTypeIndex changed here
             */
            if (sBatteryOrLoggerInfo.BatteryTypeIndex == TYPE_INDEX_NO_BATTERY) {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
                if (!sOnlyPlotterOutput) {
                    BlueDisplay1.writeString(F("\rNo battery      "));
                }
#endif
#if defined(USE_LCD)
                myLCD.setCursor(7, 0);
                myLCD.print(F(" No batt."));
#endif
            } else {
                // print voltage before the delay for LCD display
                printVoltageNoLoadMillivoltWithTrailingSpaceLCD_BD();
#if !defined(SUPPRESS_SERIAL_PRINT)
                if (!sOnlyPlotterOutput) {
                    Serial.print(F(" => "));
                    Serial.print(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].TypeName);
                    Serial.println(F(" found"));
                }
#endif
#if defined(SUPPORT_BLUEDISPLAY_CHART)
                BlueDisplay1.writeString(F("\rFound "));
                BlueDisplay1.writeString(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].TypeName);
#endif
#if defined(USE_LCD)
                myLCD.setCursor(0, 1);
                myLCD.print(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].TypeName);
                myLCD.print(F(" found"));
// The current battery voltage is displayed, so clear "No batt." message selectively
                myLCD.setCursor(7, 0);
                myLCD.print(F("         "));
                _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
                LCDClearLine(1);
#endif
                sTesterInfo.BatteryOrCurrentOrVoltageWasDetected = true;
            }
        }
    }
}

void printVoltageNoLoadMillivoltWithTrailingSpaceLCD_BD() {
    uint16_t tVoltageNoLoadMillivolt = sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt; // saves 12 bytes programming space

#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        printMillisValueAsFloat(tVoltageNoLoadMillivolt);
        Serial.print(F(" V ")); // for optional following mA
    }
#endif

    if (abs(
            (int16_t )sLastDiplayedValues.VoltageNoLoadMillivolt
            - (int16_t )tVoltageNoLoadMillivolt) > VOLTAGE_DISPLAY_HYSTERESIS_MILLIVOLT) {
        sLastDiplayedValues.VoltageNoLoadMillivolt = tVoltageNoLoadMillivolt;

#if defined(SUPPORT_BLUEDISPLAY_CHART)
        if (BlueDisplay1.isConnectionEstablished()
                && (!sInLoggerModeAndFlags || (sInLoggerModeAndFlags & LOGGER_EXTERNAL_VOLTAGE_DETECTED))) {
            // This 2 digit resolution value may be sent even if it does not change, because the 2 decimals resolution hides display of millivolt changes
            char tStringBuffer[8];
            dtostrf((float) tVoltageNoLoadMillivolt / 1000.0, 5, 2, tStringBuffer);
            tStringBuffer[5] = ' ';
            tStringBuffer[6] = 'V';
            tStringBuffer[7] = '\0';
//    strcat(tStringBuffer," V"); // 18 bytes longer
            BlueDisplay1.drawText(VOLTAGE_POSITION_X, PROBE_VALUES_POSITION_Y, tStringBuffer, PROBE_VALUES_TEXT_SIZE,
            CHART_VOLTAGE_COLOR, sBackgroundColor);
        }
#endif

#if defined(USE_LCD)
        LCDResetCursor();
        LCDPrintAsFloatWith3Decimals(tVoltageNoLoadMillivolt);
        myLCD.print(F("V "));
        sTesterInfo.VoltageNoLoadIsDisplayedOnLCD = true;
// cursor is now at 7, 0
#endif
    }
}

void printCapacity5DigitsLCD_BD() {
#if !defined(SUPPRESS_SERIAL_PRINT)
    // Print no newline
    if (!sOnlyPlotterOutput) {
        Serial.print(F("capacity="));
        Serial.print(sBatteryOrLoggerInfo.CapacityMilliampereHour);
        Serial.print(F(" mAh"));
    }
#endif
    if (sLastDiplayedValues.CapacityMilliampereHour != sBatteryOrLoggerInfo.CapacityMilliampereHour) {
        sLastDiplayedValues.CapacityMilliampereHour = sBatteryOrLoggerInfo.CapacityMilliampereHour;

#if defined(SUPPORT_BLUEDISPLAY_CHART)
        if (BlueDisplay1.isConnectionEstablished()) {
            printCapacityValue();
        }
#endif
#if defined(USE_LCD)
        char tString[6];
        snprintf_P(tString, sizeof(tString), PSTR("%5u"), sBatteryOrLoggerInfo.CapacityMilliampereHour);
        myLCD.print(tString);
        myLCD.print(F("mAh"));
#endif
    }
}

/*
 * Print ESR, if value has changed by more than 1
 * in STATE_INITIAL_SAMPLES we print current ESR / sESRHistory[0]
 * otherwise we print average ESR.
 */
void printESR() {
    uint16_t tMilliohm; // Compiler complains about initialize variable, which is wrong
    if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES && sBatteryOrLoggerInfo.Milliampere != 0) {
        tMilliohm = sESRHistory[0];
    } else {
        tMilliohm = sBatteryOrLoggerInfo.ESRMilliohm;
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

    if (abs((int16_t)sLastDiplayedValues.ESRMilliohm - (int16_t)tMilliohm) > ESR_DISPLAY_HYSTERESIS_MILLIOHM) {
        sLastDiplayedValues.ESRMilliohm = tMilliohm;

#if defined(SUPPORT_BLUEDISPLAY_CHART)
        // Do not print in values line if after boot, or in logger mode
        if (sTesterInfo.MeasurementState != STATE_SETUP_AND_READ_EEPROM && BlueDisplay1.isConnectionEstablished()) {
            char tString[9]; // e.g. "15888 mO"
            if (tMilliohm == __UINT16_MAX__) {
                BlueDisplay1.drawText(ESR_POSITION_X, PROBE_VALUES_POSITION_Y, F("overflow"), PROBE_VALUES_TEXT_SIZE,
                CHART_ESR_COLOR, sBackgroundColor);
            } else {
                snprintf_P(tString, sizeof(tString), PSTR("%5u m\x81"), tMilliohm);
                BlueDisplay1.drawText(ESR_POSITION_X, PROBE_VALUES_POSITION_Y, tString, PROBE_VALUES_TEXT_SIZE, CHART_ESR_COLOR,
                        sBackgroundColor);
            }
        }
#endif
#if defined(USE_LCD)
        myLCD.setCursor(0, 1);
        if (sInLoggerModeAndFlags) {
            LCDPrintAsFloatWith3Decimals(sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt);
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
 * Print to the same LCD location as index counter for STATE_SAMPLE_AND_STORE_TO_EEPROM
 * Print on PROBE_VALUES_POSITION_Y to enable complete clearing clearing at state change
 */
void printCounterLCD_BD(uint16_t aNumberToPrint) {
#if defined(USE_LCD)
    myLCD.setCursor(6, 0); // in case voltage was not printed

    if (aNumberToPrint < 10) {
        /*
         * We start with array index -1, which indicates initialization of array :-)
         * We have "-1" once, because we store values (and increment index) after print
         */
        myLCD.print(' '); // padding space for count
        if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES) {
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
    if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES && aNumberToPrint < 10) {
        tone(BUZZER_PIN, 2000, 40);
    }
#endif
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES && BlueDisplay1.isConnectionEstablished()) {
        /*
         * Number of samples is printed by chart values > Samples
         */
        char tString[4];
        snprintf_P(tString, sizeof(tString), PSTR("%3u"), aNumberToPrint);
        BlueDisplay1.drawText(DISPLAY_WIDTH - BASE_TEXT_SIZE * 3, PROBE_VALUES_POSITION_Y, tString, BASE_TEXT_SIZE, COLOR16_RED,
                sBackgroundColor);
    }
#endif
}

/*
 * Print only if changed more than 1 mA
 * Print no newline
 */
void printMilliampere4DigitsLCD_BD() {
#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        Serial.print(sBatteryOrLoggerInfo.Milliampere);
        Serial.print(F(" mA "));
        if (!sInLoggerModeAndFlags) {
            Serial.print(F("at "));
            printMillisValueAsFloat(sCurrentLoadResistorHistory[0]);
            Serial.print(F(" ohm, "));
        }
    }
#endif
    if (abs(
            (int16_t )sLastDiplayedValues.Milliampere
            - (int16_t )sBatteryOrLoggerInfo.Milliampere) > CURRENT_DISPLAY_HYSTERESIS_MILLIAMPERE) {
        sLastDiplayedValues.Milliampere = sBatteryOrLoggerInfo.Milliampere;

#if defined(SUPPORT_BLUEDISPLAY_CHART) || defined(USE_LCD)
        char tString[10];
#endif
#if defined(SUPPORT_BLUEDISPLAY_CHART)
        if ((!sInLoggerModeAndFlags || (sInLoggerModeAndFlags & LOGGER_EXTERNAL_CURRENT_DETECTED))
                && BlueDisplay1.isConnectionEstablished()) {
            snprintf_P(tString, sizeof(tString), PSTR("%4u mA"), sBatteryOrLoggerInfo.Milliampere);
            BlueDisplay1.drawText(CURRENT_POSITION_X, PROBE_VALUES_POSITION_Y, tString, PROBE_VALUES_TEXT_SIZE, CHART_CURRENT_COLOR,
                    sBackgroundColor);
        }
#endif
#if defined(USE_LCD)
        myLCD.setCursor(10, 0);
        snprintf_P(tString, sizeof(tString), PSTR("%4umA"), sBatteryOrLoggerInfo.Milliampere);
        myLCD.print(tString);
#endif
    }
}

/*
 * To force display of values on LCD and BlueDisplay
 */
void clearLastDiplayedValues() {
    sLastDiplayedValues.VoltageNoLoadMillivolt = 0;
    sLastDiplayedValues.Milliampere = 0;
    sLastDiplayedValues.ESRMilliohm = 0;
}

/*
 * Called exclusively from setup() after readAndProcessEEPROMData()
 */
void printStoredDataLCD_BD() {
#if defined(USE_LCD)
    myLCD.clear();
    myLCD.print(getVCCVoltage(), 1);
    myLCD.print(F("V Stored data"));
#endif
    /*
     * Print battery values, and use state STATE_SETUP_AND_READ_EEPROM for formatting
     * "0.061o h 1200mAh" using sBatteryOrLoggerInfo.ESRMilliohm
     */
    printMeasurementValuesLCD_BD();
#if defined(USE_LCD)
    _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
}

/*
 * Evaluates sTesterInfo.MeasurementState and prints:
 *   - sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt
 *   - sBatteryOrLoggerInfo.Milliampere
 *   - sBatteryOrLoggerInfo.ESRMilliohm
 *   - optional ESRDeltaMillivolt or capacity
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
 * "0.392o l 1200mAh" using sBatteryOrLoggerInfo.ESRMilliohm
 *
 * Called with states STATE_SETUP_AND_READ_EEPROM, STATE_INITIAL_SAMPLES and STATE_SAMPLE_AND_STORE_TO_EEPROM
 * State STATE_WAITING_FOR_BATTERY_OR_EXTERNAL is handled in loop()
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

void printMeasurementValuesLCD_BD() {
    sInLCDPrint = true; // disable printing by button handler
    uint8_t tMeasurementState = sTesterInfo.MeasurementState; // Because sTesterInfo.MeasurementState is volatile
    if (tMeasurementState != STATE_SETUP_AND_READ_EEPROM) {
        /***********************************************************************************
         * First row only for state STATE_INITIAL_SAMPLES or STATE_SAMPLE_AND_STORE_TO_EEPROM
         ***********************************************************************************/

        /*
         * First check and print counter
         * Use last displayed value for comparison, it is almost (+/-1) the actual voltage and it is the value the user observes!
         */
        if (tMeasurementState == STATE_INITIAL_SAMPLES) {
            /*
             * Print down counter for STATE_INITIAL_SAMPLES
             * Count down only if we are in logger mode or we do not have a rapid voltage decrease (> 6 mV per sample)
             */
            if (sInLoggerModeAndFlags
// @formatter:off
                    || sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt >= sLastDiplayedValues.VoltageNoLoadMillivolt
// @formatter:on
                    || (sLastDiplayedValues.VoltageNoLoadMillivolt - sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt)
                            <= MAX_VOLTAGE_DROP_MILLIVOLT_FOR_COUNTDOWN) {
                sTesterInfo.NumbersOfInitialSamplesToGo--;
            }

#if !defined(SUPPRESS_SERIAL_PRINT)
            if (!sOnlyPlotterOutput) {
                Serial.print(sTesterInfo.NumbersOfInitialSamplesToGo);
                Serial.print(F(" s, ")); // seconds until discharging
            }
#endif
            printCounterLCD_BD(sTesterInfo.NumbersOfInitialSamplesToGo);

        } else {
#if defined(USE_LCD)
            /*
             * Print counter for STATE_SAMPLE_AND_STORE_TO_EEPROM
             * Use (index + 1) to be consistent with the number of samples displayed for array
             */
            printCounterLCD_BD(
                    (ValuesForDeltaStorage.DeltaArrayIndex + 1) * (StartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE));
#endif
        }

        /*
         * Print no load voltage here
         */
        printVoltageNoLoadMillivoltWithTrailingSpaceLCD_BD();
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            // 4.094 V,  334 mA at 11.949 ohm, ESR=0.334 ohm, capacity=3501 mAh
            Serial.print(F(", "));
        }
#endif

        /*
         * Print Current
         */
        printMilliampere4DigitsLCD_BD();
    }

    /**********************
     * Start of second row
     **********************/
    if (sTesterInfo.MeasurementState != STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
        /*
         * STATE_SETUP_AND_READ_EEPROM + STATE_SAMPLE_AND_STORE_TO_EEPROM: "0.061o h 1200mAh" using sBatteryOrLoggerInfo.ESRMilliohm
         * STATE_INITIAL_SAMPLES:                               "0.061o l  0.128V" using current ESR from sESRHistory[0]
         */
        if (sInLoggerModeAndFlags) {
#if !defined(SUPPRESS_SERIAL_PRINT)
            if (!sOnlyPlotterOutput) {
                Serial.print(F(" Min="));
                printMillisValueAsFloat(sBatteryOrLoggerInfo.Voltages.Logger.MinimumMillivolt);
                Serial.print(F(" V, Avg="));
                printMillisValueAsFloat(sBatteryOrLoggerInfo.Voltages.Logger.AverageMillivolt);
                Serial.print(F(" V, Max="));
                printMillisValueAsFloat(sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt);
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
        myLCD.print(sBatteryOrLoggerInfo.CutoffLevelCharacter);
#endif

        /*
         * Print voltage difference or capacity
         */
        if (tMeasurementState == STATE_INITIAL_SAMPLES) {
#if !defined(SUPPRESS_SERIAL_PRINT) || defined(USE_LCD)
            if (!sInLoggerModeAndFlags) {
                /*
                 * Print voltage difference between no load and load used for ESR computation
                 */
                uint16_t tESRDeltaMillivolt = sBatteryOrLoggerInfo.ESRDeltaMillivolt; // saves 4 bytes programming space
#  if !defined(SUPPRESS_SERIAL_PRINT)
                if (!sOnlyPlotterOutput) {
                    printMillisValueAsFloat(tESRDeltaMillivolt);
                    Serial.print(F(" V "));
                }
#  endif
#  if defined(USE_LCD)
                myLCD.print(F("  ")); // leading spaces only for voltage
                LCDPrintAsFloatWith3Decimals(tESRDeltaMillivolt);
                myLCD.print(F("V"));
#  endif
            }
#endif
        } else {
            /*
             * Print capacity
             */
            printCapacity5DigitsLCD_BD();
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
int8_t clipDelta(int32_t aDelta) {
    if (aDelta > __INT8_MAX__) {
        return __INT8_MAX__;
    } else if (aDelta < -128) {
        return -128;
    }
    return aDelta;
}
/*
 * Store values to EEPROM as 4 bit deltas between sBatteryOrLoggerInfo and ValuesForDeltaStorage and write them to EEPROM every second call
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
        StartValues.LoadResistorMilliohm = sCurrentLoadResistorAverage;
        StartValues.CutoffLevel = sBatteryOrLoggerInfo.CutoffLevel;
        StartValues.BatteryTypeIndex = sBatteryOrLoggerInfo.BatteryTypeIndex;
        StartValues.inLoggerModeAndFlags = sInLoggerModeAndFlags;
        StartValues.NumberOfSecondsPerStorage = INITIAL_NUMBER_OF_SECONDS_PER_STORAGE;
        // next two values are set in handleEndOfStateInitialSamples()
//        StartValues.CapacityMilliampereHour = 0; // Capacity is written at the end or computed while reading
//        StartValues.NumberOfSecondsPerStorage = INITIAL_NUMBER_OF_SECONDS_PER_STORAGE;
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

        // I have seen rapid changes in ESR and therefore overflow in int16_t -> use int32_t
        int32_t tMilliohmDelta = (uint32_t) aMilliohm - ValuesForDeltaStorage.lastStoredMilliohm;
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
    eeprom_update_word(&EEPROMStartValues.CapacityMilliampereHour, sBatteryOrLoggerInfo.CapacityMilliampereHour);
    eeprom_update_byte(&EEPROMStartValues.CutoffLevel, sBatteryOrLoggerInfo.CutoffLevel);
#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
// Print should be done after checkForDoublePress() in order to not disturb the double press detection
        Serial.print(F("Cut off level "));
        Serial.print(sBatteryOrLoggerInfo.CutoffLevelCharacter);
        Serial.print(F(" and capacity "));
        Serial.print(sBatteryOrLoggerInfo.CapacityMilliampereHour);
        Serial.println(F(" mAh stored"));
    }
#endif
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    BlueDisplay1.writeString(F("\rCapacity stored ")); // space to overwrite e.g. "LiIo 2pack"
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
                Serial.print(F("mOhm->"));
                Serial.print(aMilliohmToPrint);
                Serial.print(F("mOhm___LoadResistor="));
                printMillisValueAsFloat(StartValues.LoadResistorMilliohm);
                Serial.print(F("ohm__Capacity="));
                Serial.print(sBatteryOrLoggerInfo.CapacityMilliampereHour);
                Serial.print(F("mAh__Duration="));
                // We have 2 4bit values per storage byte
                uint16_t tDurationMinutes = (ValuesForDeltaStorage.DeltaArrayIndex)
                * (2 * INITIAL_NUMBER_OF_SAMPLES_PER_STORAGE)/ SECONDS_IN_ONE_MINUTE;
                Serial.print(tDurationMinutes / MINUTES_IN_ONE_HOUR_SHORT);
                Serial.print(F("h_"));
                Serial.print(tDurationMinutes % MINUTES_IN_ONE_HOUR_SHORT);
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
        if (sChartReadValueArrayType == TYPE_NO_DATA) {
            return;
        }
// input is millivolt convert to 20 for one volt
        if (sChartReadValueArrayType == TYPE_VOLTAGE) {
// Voltage
            sChartValueArray[sChartValueArrayIndex] = (aMillivoltToPrint - sCompressionOffsetMillivolt) / sCompressionFactor;
        } else if (sChartReadValueArrayType == TYPE_ESR) {
            sChartValueArray[sChartValueArrayIndex] = (aMilliohmToPrint) / sCompressionFactor;
        } else {
            // TYPE_CURRENT
            sChartValueArray[sChartValueArrayIndex] = (aMilliampereToPrint) / sCompressionFactor;
        }

        sChartValueArrayIndex++;
        if (aIsLastElement) {
#    if defined(LOCAL_DEBUG)
            Serial.println();
            Serial.print(F("ChartReadValueArrayType="));
            Serial.print(sChartReadValueArrayType);
            Serial.print(F(" sCompressionFactor="));
            Serial.print(sCompressionFactor);
            Serial.println();
#    endif
            if (sChartReadValueArrayType == TYPE_VOLTAGE) {
                sLastChartData.Millivolt = aMillivoltToPrint;
                /*
                 * Compute new x scale depending on the current data length
                 * We get the following values for compression 1, 2 and 4
                 * factors:  6   4   3   2  1.5   1      1.5    1  1.5    1
                 * Labels:  10  15  20  30  40 1:00     1:20 2:00 2:40 4:00
                 */
                int16_t tXScaleFactor = VoltageChart.computeXLabelAndXDataScaleFactor(sChartValueArrayIndex);
#    if defined(LOCAL_TRACE)
                Serial.println();
                Serial.print(F("ChartValueArrayIndex="));  // Current length of chart
                Serial.print(sChartValueArrayIndex);
                Serial.print(F(" WidthX="));
                Serial.print(VoltageChart.mWidthX);
                Serial.print(F(" XLabelBaseIncrementValue="));
                Serial.print(VoltageChart.mXLabelBaseIncrementValue);
                Serial.print(F(" XDataScaleFactor="));
                Serial.print(VoltageChart.mXDataScaleFactor);
                Serial.println();
#    endif
                float tChartMinutesPerLabelUncompressed = CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED; // 30
                uint8_t tXGridOrLabelPixelSpacing = CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED;
                if (tXScaleFactor >= 5) {
                    // Here we have 12 minutes label or less -> use scale factor 6 which is 10 minutes label
                    tXScaleFactor = 6;
                }
                if (tXScaleFactor == 1) {
                    /*
                     * Here we have labels 40, 1:20, 2:40 which is 2 * (2/3 of 30, 1:00, 2:00). 2 is the label distance.
                     * Change to labels 45, 1:30, 3:00 by setting XLabelBaseIncrementValue from 30 to 33.75
                     */
                    tChartMinutesPerLabelUncompressed = 33.75;
                    tXGridOrLabelPixelSpacing = 34; // This is not totally correct, but gives almost the desired result with only 2 pixels / 1 sample difference
                }
                VoltageChart.setXLabelAndXDataScaleFactor(tXScaleFactor);
                VoltageChart.setXGridOrLabelPixelSpacing(tXGridOrLabelPixelSpacing);

#if SECONDS_IN_ONE_MINUTE == INITIAL_NUMBER_OF_SECONDS_PER_STORAGE // Saves 4 bytes
                VoltageChart.setXLabelBaseIncrementValue(
                        (StartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE) * tChartMinutesPerLabelUncompressed); // Starts with CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED (30) for 1 minute sampling
#else
                VoltageChart.setXLabelBaseIncrementValue(
                        (StartValues.NumberOfSecondsPerStorage * tChartMinutesPerLabelUncompressed) / SECONDS_IN_ONE_MINUTE); // Starts with CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED (30) for 1 minute sampling
#endif

                clearAndDrawChart(); // now the axes parameter are set, draw axes and grid and caption
            } else if (sChartReadValueArrayType == TYPE_ESR) {
                sLastChartData.ESRMilliohm = aMilliohmToPrint;
            } else {
                // TYPE_CURRENT
                sLastChartData.Milliampere = aMilliampereToPrint;
            }
            VoltageChart.drawChartDataWithYOffset(sChartValueArray, sChartValueArrayIndex, CHART_MODE_LINE);
        }
    } else
#  endif // defined(SUPPORT_BLUEDISPLAY_CHART)

    { // new test is shorter than else!
        if (aIsLastElement) {
// Print updated plotter caption
            Serial.print(F("Voltage="));
            printMillisValueAsFloat(StartValues.initialDischargingMillivolt);
            Serial.print(F("V->"));
            if (sInLoggerModeAndFlags) {
                printMillisValueAsFloat(sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt);
            } else {
                printMillisValueAsFloat(aMillivoltToPrint);
            }
            Serial.print(F("V:"));
            Serial.print(aMillivoltToPrint);
            Serial.print(F(" Current="));
            Serial.print(StartValues.initialDischargingMilliampere);
            Serial.print(F("mA->"));
            Serial.print(aMilliampereToPrint);
            Serial.print(F("mA:"));
            Serial.print(aMilliampereToPrint);
            if (StartValues.initialDischargingMilliohm > 0) {
                Serial.print(F(" ESR="));
                Serial.print(StartValues.initialDischargingMilliohm);
                Serial.print(F("mOhm->"));
                Serial.print(aMilliohmToPrint);
                Serial.print(F("mOhm:"));
                Serial.print(aMilliohmToPrint);
                Serial.print(F(" LoadResistor="));
                printMillisValueAsFloat(StartValues.LoadResistorMilliohm);
                Serial.print(F("Ohm"));
            }
            Serial.print(F(" Capacity="));
            if (sTesterInfo.isStandardCapacityAvailable) {
                Serial.print(sTesterInfo.StandardCapacityMilliampereHour);
                Serial.print('_');
            }
            Serial.print(sBatteryOrLoggerInfo.CapacityMilliampereHour);
            Serial.print(F("mAh Duration"));
            uint16_t tDurationMinutes = (ValuesForDeltaStorage.DeltaArrayIndex)
                    * (StartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE);
            Serial.print('=');
            Serial.print(tDurationMinutes / MINUTES_IN_ONE_HOUR_SHORT);
            Serial.print(F("h_"));
            Serial.print(tDurationMinutes % MINUTES_IN_ONE_HOUR_SHORT);
            Serial.print(F("min Sample_time="));
            Serial.print(StartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE);
            Serial.print(F("min Cutoff="));
            Serial.print(sBatteryOrLoggerInfo.CutoffVoltageMillivolt);
            Serial.print(F("mV"));

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

#define CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE   0
#define CAPACITY_STARTED                1       // Current voltage is below or equal NominalFullVoltageMillivolt and higher or equal CutoffVoltageMillivoltHigh
#define CAPACITY_COMPLETED              2       // Current voltage is below CutoffVoltageMillivoltHigh
/*
 * Reads EEPROM delta values array
 * - Print data for plotter and compute ESR on the fly from voltage, current and load resistor, if aStoreValuesForDisplayAndAppend is false.
 * - Compute capacity from current (if defined SUPPORT_CAPACITY_RESTORE).
 * - Compute max and min values in ValuesForChartScaling and store them AFTER the loop.
 * @param aStoreValuesForDisplayAndAppend  - if true (called at setup()), store initial data and if SUPPORT_BLUEDISPLAY_CHART, do not print.
 *                                         - if false, compute scaling data for chart display and print / display chart
 */
void readAndProcessEEPROMData(bool aInitializeValuesForDisplayAndAppend) {
    EEPROMData tEEPROMData;
    /*
     * First copy EEPROM start values to RAM
     */
    eeprom_read_block(&StartValues, &EEPROMStartValues, sizeof(EEPROMStartValues));

    /*
     * Search last non EEPROM_EMPTY_VALUE / 0xFF (not cleared) value
     * Search from end to start
     */
    int tLastWrittenIndex;
    for (tLastWrittenIndex = MAX_NUMBER_OF_SAMPLES - 1; tLastWrittenIndex >= 0; tLastWrittenIndex--) {
        eeprom_read_block(&tEEPROMData, &EEPROMDataArray[tLastWrittenIndex], sizeof(tEEPROMData));
        if (tEEPROMData.DeltaMillivolt != (int8_t) EEPROM_EMPTY_VALUE || tEEPROMData.DeltaMilliampere != (int8_t) EEPROM_EMPTY_VALUE
                || tEEPROMData.DeltaESRMilliohm != (int8_t) EEPROM_EMPTY_VALUE) {
            break;
        }
    }
#if defined(LOCAL_TRACE)
    Serial.print(F("tLastWrittenIndex="));
    Serial.print(tLastWrittenIndex);
    Serial.print(F(" &tLastWrittenIndex=0x"));
    Serial.println((uint16_t) &EEPROMDataArray[tLastNonWrittenIndex], HEX);
    // dump 2 lines containing the 3 byte data
    dumpEEPROM((uint8_t*) ((uint16_t) &EEPROMDataArray[tLastWrittenIndex] & 0xFFF0), 2);
#endif

    int tFirstNonWrittenIndex = tLastWrittenIndex + 1;
    uint16_t tVoltageMillivolt = StartValues.initialDischargingMillivolt;

#if defined(SUPPORT_BLUEDISPLAY_CHART)
    /*
     * Initialize values for chart scaling to be stored after the loop in ValuesForChartScaling
     * Use start values, because they are not handled in loop.
     */
    struct ValuesForChartScaling tTemporaryValuesForChartScaling;
    tTemporaryValuesForChartScaling.minVoltageNoLoadMillivolt = StartValues.initialDischargingMillivolt;
    tTemporaryValuesForChartScaling.maxVoltageNoLoadMillivolt = StartValues.initialDischargingMillivolt;
    tTemporaryValuesForChartScaling.maxMilliampere = StartValues.initialDischargingMilliampere;
    tTemporaryValuesForChartScaling.maxMilliohm = StartValues.initialDischargingMilliohm;
#endif

    if (aInitializeValuesForDisplayAndAppend) {
        /*
         * Set values required for append from start values already read.
         * Is only called once at setup()!
         */
        ValuesForDeltaStorage.DeltaArrayIndex = tFirstNonWrittenIndex; // for append
        sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt = tVoltageMillivolt; // ??? why???
        sBatteryOrLoggerInfo.CapacityMilliampereHour = StartValues.CapacityMilliampereHour; // may be corrected later
        setBatteryTypeIndexFromVoltage(tVoltageMillivolt); // sets sBatteryOrLoggerInfo.BatteryTypeIndex and LoadSwitchSettleTimeMillis
        setCutoffAndCutoffVoltage(StartValues.CutoffLevel);
    }

#if defined(SUPPORT_BLUEDISPLAY_CHART)
    sChartValueArrayIndex = 0; // Start a new chart line

#  if defined(LOCAL_DEBUG)
    Serial.print(F("Store="));
    Serial.print(aInitializeValuesForDisplayAndAppend);
    Serial.print(F(" Type="));
    Serial.print(sChartReadValueArrayType);
#  endif
#endif // defined(SUPPORT_BLUEDISPLAY_CHART)

    /*
     * Check if start voltage > voltage for standard capacity computation
     * Assume, that voltage is not rising, so check with first value is sufficient
     */
    sTesterInfo.isStandardCapacityAvailable = false;
    uint8_t tCapacityMilliampereHourStandardValueState;
    auto tNominalFullVoltageMillivolt = BatteryTypeInfoArray[StartValues.BatteryTypeIndex].NominalFullVoltageMillivolt;
    if (tVoltageMillivolt >= tNominalFullVoltageMillivolt) {
        tCapacityMilliampereHourStandardValueState = CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE;
    } else {
        tCapacityMilliampereHourStandardValueState = CAPACITY_STARTED;
    }
    uint32_t tCapacityAccumulatorUntilNominalFullVoltageValue = 0; // Used to compute standard capacity later

    uint16_t tMilliampere = StartValues.initialDischargingMilliampere;
    uint32_t tCapacityAccumulator = tMilliampere;
    uint16_t tMilliohm = StartValues.initialDischargingMilliohm;
    uint8_t tNumberOfEEPROMValuesPerHour = 3600 / StartValues.NumberOfSecondsPerStorage;

#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        Serial.println();
// We have always the first one as uncompressed value
        Serial.print(tFirstNonWrittenIndex + 1);
        Serial.print(F(" EEPROM data sets each "));
        Serial.print(StartValues.NumberOfSecondsPerStorage);
        Serial.print(F(" seconds found"));
        if (!sInLoggerModeAndFlags) {
            Serial.print(F(" for type="));
            Serial.print(BatteryTypeInfoArray[StartValues.BatteryTypeIndex].TypeName);
        }
        Serial.print(F(", cut off level="));
        Serial.println(sBatteryOrLoggerInfo.CutoffLevelCharacter);
    }
#endif

    /****************************************************
     * Print the initial value with no caption to plotter
     ****************************************************/
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (!aInitializeValuesForDisplayAndAppend) {
        printValuesForPlotterAndChart(tVoltageMillivolt, tMilliampere, tMilliohm, false);
    }
#else
    printValuesForPlotterAndChart(tVoltageMillivolt, tMilliampere, tMilliohm, false); // print always at non BD mode
#endif
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

        // Don't know, what this is for, since it is overwritten
        if (sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt < tVoltageMillivolt) {
            sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt = tVoltageMillivolt;
        }

#if defined(SUPPORT_BLUEDISPLAY_CHART)
        /*
         * Get maximum and minimum for chart scaling
         */
        if (tTemporaryValuesForChartScaling.minVoltageNoLoadMillivolt > tVoltageMillivolt) {
            tTemporaryValuesForChartScaling.minVoltageNoLoadMillivolt = tVoltageMillivolt;
        }
        if (tTemporaryValuesForChartScaling.maxVoltageNoLoadMillivolt < tVoltageMillivolt) {
            tTemporaryValuesForChartScaling.maxVoltageNoLoadMillivolt = tVoltageMillivolt;
        }
        if (tTemporaryValuesForChartScaling.maxMilliampere < tMilliampere) {
            tTemporaryValuesForChartScaling.maxMilliampere = tMilliampere;
        }
        if (tTemporaryValuesForChartScaling.maxMilliohm < tMilliohm) {
            tTemporaryValuesForChartScaling.maxMilliohm = tMilliohm;
        }

#endif
        uint16_t tVoltageForPrint = tVoltageMillivolt; // To print markers for start and end of standard capacity
#if !defined(SUPPRESS_SERIAL_PRINT)
        uint8_t tPrintDelayed = 0; // to append text at values print output
#endif

        if (!sInLoggerModeAndFlags) {
            /*
             * Get "standard" capacity from NominalFullVoltageMillivolt to CutoffVoltageMillivoltHigh
             */
            if (tCapacityMilliampereHourStandardValueState == CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE
                    && tVoltageMillivolt <= tNominalFullVoltageMillivolt) {
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
                    && tVoltageMillivolt < BatteryTypeInfoArray[StartValues.BatteryTypeIndex].CutoffVoltageMillivoltHigh) {
                tCapacityMilliampereHourStandardValueState = CAPACITY_COMPLETED;
                /*
                 * We reached CutoffVoltageMillivoltHigh, now we can compute standard capacity, if we started above nominal full voltage
                 * Standard capacity is always computed from EEPROM data, while capacity is taken from StartValues.
                 * If we had power down before appending more than 12 percent discharge, this amount is missing in StartValues.capacity.
                 * In this case, capacity may be smaller than standard capacity!
                 */
                if (tCapacityAccumulatorUntilNominalFullVoltageValue > 0) {
                    sTesterInfo.isStandardCapacityAvailable = true;
                    sTesterInfo.StandardCapacityMilliampereHour = (tCapacityAccumulator
                            - tCapacityAccumulatorUntilNominalFullVoltageValue) / tNumberOfEEPROMValuesPerHour; // -> tCapacityAccumulator / 60 for 1 minute sampling
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
#if defined(SUPPORT_BLUEDISPLAY_CHART)
        if (!aInitializeValuesForDisplayAndAppend) {
            printValuesForPlotterAndChart(tVoltageForPrint, tMilliampere, tMilliohm, i == (tFirstNonWrittenIndex - 1));
        }
#else
        printValuesForPlotterAndChart(tVoltageForPrint, tMilliampere, tMilliohm, i == (tFirstNonWrittenIndex - 1)); // print always at non BD mode
#endif

#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            if (tPrintDelayed == 1) {
                Serial.print(F(" - Capacity on top of standard value="));
                Serial.print(tCapacityAccumulator / tNumberOfEEPROMValuesPerHour);
                Serial.print(F(" mAh"));
            } else if (tPrintDelayed == 2) {
                Serial.print(F(" - "));
                if (sTesterInfo.isStandardCapacityAvailable) {
                    Serial.print(F("Standard "));
                }
                if (sInLoggerModeAndFlags) {
                    Serial.print(F("capacity="));
                } else {
                    Serial.print(F("capacity at high cut off="));
                }
                Serial.print(sTesterInfo.StandardCapacityMilliampereHour);
                Serial.print(F(" mAh"));
            }
        }
        Serial.println();
#endif
    } // End of read loop

    /**************************************************************************
     * Loop was processed, handle scaling values, capacity and LCD display now
     **************************************************************************/
#if defined(SUPPORT_BLUEDISPLAY_CHART)
//Store new values for chart scaling after loop and chart display
    ValuesForChartScaling = tTemporaryValuesForChartScaling;
#endif

    uint16_t tCurrentCapacityMilliampereHourComputed = tCapacityAccumulator / tNumberOfEEPROMValuesPerHour;

#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        if (sBatteryOrLoggerInfo.CapacityMilliampereHour == 0) {
            Serial.print(F("No capacity was stored, so use computed capacity of "));
            Serial.print(tCurrentCapacityMilliampereHourComputed);
        } else {
            /*
             * The observed delta was around 1% :-)
             */
            int16_t tCurrentCapacityMilliampereHourDelta = sBatteryOrLoggerInfo.CapacityMilliampereHour
                    - tCurrentCapacityMilliampereHourComputed;
            Serial.print(F("Stored minus computed capacity="));
            Serial.print(tCurrentCapacityMilliampereHourDelta);
        }
        Serial.println(F(" mAh"));

        /*
         * Print Standard capacity a between NominalFullVoltageMillivolt and  CutoffVoltageMillivoltHigh,
         * if we have both values.
         */
        if (!sInLoggerModeAndFlags && sTesterInfo.StandardCapacityMilliampereHour != 0
                && sTesterInfo.StandardCapacityMilliampereHour != tCurrentCapacityMilliampereHourComputed) {
            if (sTesterInfo.isStandardCapacityAvailable) {
                Serial.print(F("Standard "));
            }
            Serial.print(F("computed capacity between "));
            if (sTesterInfo.isStandardCapacityAvailable) {
                Serial.print(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].NominalFullVoltageMillivolt);
            } else {
                Serial.print(StartValues.initialDischargingMillivolt);
            }
            Serial.print(F(" mV and "));
            Serial.print(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].CutoffVoltageMillivoltHigh);
            Serial.print(F(" mV="));
            Serial.print(sTesterInfo.StandardCapacityMilliampereHour);
            Serial.println(F(" mAh"));
        }
    } // if (!sOnlyPlotterOutput)
#endif

    if (aInitializeValuesForDisplayAndAppend) {
        /*
         * Store values acquired during loop above
         */
// For LCD display of stored data
        sBatteryOrLoggerInfo.ESRMilliohm = tMilliohm; // displayed in summary print
        sBatteryOrLoggerInfo.Milliampere = tMilliampere; // To avoid overflow detection for ESRMilliohm print

        /*
         * Store last values for append functionality and chart scaling
         */
        ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt = tVoltageMillivolt;
        ValuesForDeltaStorage.lastStoredMilliampere = tMilliampere;
        ValuesForDeltaStorage.lastStoredMilliohm = tMilliohm;

        /*
         * Check if capacity must be corrected, because it was not stored at end of measurement (maybe because of sudden power down)
         * If we have a power down before additional discharging 12.5 percent of capacity,
         * capacity will not be corrected and this amount is missing if we do an append.
         * In this case capacity may be less than standard capacity!
         */
        if (sBatteryOrLoggerInfo.CapacityMilliampereHour
                < tCurrentCapacityMilliampereHourComputed - (tCurrentCapacityMilliampereHourComputed / 8)) {
            sBatteryOrLoggerInfo.CapacityMilliampereHour = tCurrentCapacityMilliampereHourComputed;
        }
// restore capacity accumulator
        sBatteryOrLoggerInfo.CapacityAccumulator = sBatteryOrLoggerInfo.CapacityMilliampereHour
                * ((3600L * MILLIS_IN_ONE_SECOND) / SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS);
    }
}

#if defined(SUPPORT_BLUEDISPLAY_CHART)

void doMeasurementState(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;

    if (sTesterInfo.MeasurementState == STATE_STOPPED) {
        // start a new measurement cycle
        switchToStateWaitingForBatteryOrVoltage();
    } else {
        switchToStateStopped('B'); // 'B' for button press. No check for double press required here :-)
    }
}

void setMeasurementStateButtonTextAndDrawButton() {
    uint8_t tButtonTextIndex = sTesterInfo.MeasurementState;
    if (sTesterInfo.MeasurementWasFinishedByEndCondition) {
        tButtonTextIndex++; // switch from "Stopped"; // stopped manually, to "Finished"; // stopped by detecting end condition
    }
    if (!sOnlyPlotterOutput) {
        if (BlueDisplay1.isConnectionEstablished()) {
            TouchButtonMeasurementState.setPGMTextFromPGMArray(sMeasurementStateButtonTextStringArray, tButtonTextIndex, true);
        }
    }
}

/*
 * Append is only visible during state STATE_INITIAL_SAMPLES
 */
void doAppend(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;
    setBatteryTypeIndex(StartValues.BatteryTypeIndex); // Restore original type index. This sets cutoff level accordingly
    switchToStateSampleAndStoreToEEPROM(0);
}

void doChartType(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;
//    sChartDisplayValueArrayType = aValue;
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

void setBatteryLoggerButtonTextAndDrawButton() {
    if (sInLoggerModeAndFlags) {
        TouchButtonBatteryLogger.setText(F("Logger"), true);
    } else {
        TouchButtonBatteryLogger.setText(F("Battery"), true);
    }
}

void doBatteryLogger(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;
    sInLoggerModeAndFlags = !sInLoggerModeAndFlags;
    sCurrentLoadResistorAverage = LOGGER_SHUNT_RESISTOR_MILLIOHM;
    redrawDisplay();
}

void setCutoffHighLowZeroButtonTextAndDrawButton() {
    if (BlueDisplay1.isConnectionEstablished()) {

        char tString[20];
        if (sInLoggerModeAndFlags) {
            // CUTOFF_LEVEL_HIGH = 50%, LOW = 25% and ZERO = 12.5%
            snprintf_P(tString, sizeof(tString), PSTR("Cutoff %2u%% I"), 100 >> (sBatteryOrLoggerInfo.CutoffLevel + 1));
        } else {
            /*
             * Battery mode here
             */
//        debugPrintCutoffInfo();
            if (sBatteryOrLoggerInfo.BatteryTypeIndex == TYPE_INDEX_NO_BATTERY) {
                TouchButtonCutoffHighLowZero.setPGMTextFromPGMArray(sCutoffButtonTextStringArray, sBatteryOrLoggerInfo.CutoffLevel,
                        true);
                return;
            } else {
                snprintf_P(tString, sizeof(tString), PSTR("Cutoff %4umV"), sBatteryOrLoggerInfo.CutoffVoltageMillivolt);
            }
        }
        TouchButtonCutoffHighLowZero.setText(tString, true);
    }
}

void doCutoffHighLowZero(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;
    setNextCutoffLevelAndPrint();
}

/*
 * This handler is called after boot or reconnect
 */
void connectHandler(void) {
#if defined(LOCAL_DEBUG)
    Serial.println(F("connectHandler"));
#endif
    initDisplay(); // does a clear();
    initBatteryChart();

    /*
     * Set brightness to to high / user defined :-)
     * Use a hack for it to save duplicate code
     */
    sCurrentBrightness = BRIGHTNESS_LOW;
    changeBrightness();

    /*
     * As redrawDisplay() does not initially read the max and min voltages for scaling,
     * scaling may be incorrect after a long time without connection.
     */
    sChartReadValueArrayType = TYPE_NO_DATA;
    readAndProcessEEPROMData(false); // first read the max and min voltages
    redrawDisplay();
}

void redrawDisplay(void) {
#if defined(LOCAL_DEBUG)
    Serial.println(F("redrawDisplay"));
#endif
    BlueDisplay1.clearDisplay(sBackgroundColor);
    clearLastDiplayedValues();
    drawButtons();
    readAndDrawEEPROMValues(); // draws the charts and calls printChartValues() and printTimeAtOneLine() at the end
}

void initDisplay(void) {
#if defined ENABLE_DISPLAY_OF_DATE_AND_TIME
    initLocalTimeHandling();
#endif

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
    FLAG_BUTTON_DO_BEEP_ON_TOUCH, sTesterInfo.MeasurementState != STATE_STOPPED, &doMeasurementState);
    TouchButtonMeasurementState.init(&tBDButtonPGMParameterStruct);

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

    tBDButtonPGMParameterStruct.aWidthX = BUTTON_WIDTH;
// 90 bytes because setBatteryLoggerButtonTextAndDrawButton() will be enabled by this code
    tBDButtonPGMParameterStruct.aPositionY += BASE_TEXT_SIZE + (BASE_TEXT_SIZE / 2);
    tBDButtonPGMParameterStruct.aOnTouchHandler = &doBatteryLogger;
    TouchButtonBatteryLogger.init(&tBDButtonPGMParameterStruct);

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
    tBDButtonPGMParameterStruct.aPGMText = F("Ohm");
    TouchButtonOnlyTextESR.init(&tBDButtonPGMParameterStruct);
    TouchButtonOnlyTextESR.setButtonTextColor(CHART_ESR_COLOR);

    tBDButtonPGMParameterStruct.aPositionX += (BASE_TEXT_SIZE * 4);
    tBDButtonPGMParameterStruct.aValue = TYPE_CURRENT;
    tBDButtonPGMParameterStruct.aPGMText = F("Ampere");
    TouchButtonOnlyTextAmpere.init(&tBDButtonPGMParameterStruct);
    TouchButtonOnlyTextAmpere.setButtonTextColor(CHART_CURRENT_COLOR);

// Settings for Text messages
    BlueDisplay1.setWriteStringPosition(PROBE_VALUES_POSITION_X, MESSAGE_START_POSITION_Y);
    BlueDisplay1.setWriteStringSizeAndColorAndFlag(sChartDataTextSize, sTextColor, sBackgroundColor, false);
}

void initBatteryChart() {

    for (uint16_t i = 0; i < MAX_NUMBER_OF_SAMPLES; ++i) {
        sChartValueArray[i] = 0;
    }
    /*
     * Chart width is CHART_WIDTH (337) pixel corresponding to data entry 0 -> 337, 5 hours and 36 min.
     * with we have 11 segments
     * For CHART_X_AXIS_SCALE_FACTOR_1 and at 1 minute (sample) per pixel and 1 hour labels (1:00) each 60 pixel.
     */
// Label increment is 30 min for scale factor 1 so we have 5:37 for complete chart
    VoltageChart.initXLabel(0, CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED, CHART_X_AXIS_SCALE_FACTOR_1, 3, 0);
    VoltageChart.setXLabelDistance(2); // Label is displayed each 2. grid!
    VoltageChart.setLabelStringFunction(VoltageChart.convertMinutesToString);

    uint16_t tChartHeight = BlueDisplay1.getRequestedDisplayHeight() - (BlueDisplay1.getRequestedDisplayHeight() / 4); // 3/4 display height
    uint16_t tYGridSize = (BlueDisplay1.getRequestedDisplayHeight() / 10); // 34

//    initChart(48, Height - 24, 337, 3/4 Height, 2, 16, true, 30, 34);
    VoltageChart.initChart(CHART_START_X, BlueDisplay1.getRequestedDisplayHeight() - (BASE_TEXT_SIZE + (BASE_TEXT_SIZE / 2)),
    CHART_WIDTH, tChartHeight, CHART_AXES_SIZE, BASE_TEXT_SIZE, CHART_DISPLAY_GRID, CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED,
            tYGridSize);

    VoltageChart.initChartColors(CHART_VOLTAGE_COLOR, CHART_AXES_COLOR, CHART_GRID_COLOR, sTextColor, sTextColor, sBackgroundColor);
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
    setMeasurementStateButtonTextAndDrawButton();
    setCutoffHighLowZeroButtonTextAndDrawButton();
    setBatteryLoggerButtonTextAndDrawButton();
//    TouchButtonRedraw.drawButton();
    TouchButtonBrightness.drawButton();
    if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES) {
        TouchButtonAppend.drawButton();
    }
}

/*
 * Message area is not cleared here, it is cleared by overwrite
 */
void clearValueArea() {
    BlueDisplay1.fillRectRel(0, 0, BlueDisplay1.getRequestedDisplayWidth(), MESSAGE_START_POSITION_Y - 1, sBackgroundColor);
}

#define GRIDS_VERTICAL_Y  7
/*
 * Set sChartReadValueArrayType to the types of data for chart and repeatedly
 * call readAndProcessEEPROMData() to read and print the different charts
 *
 * For 7 grids vertical and 1.4 full range, (compression * YDataFactor) = 1
 *
 * For each grid of 200 mV, mOhm, mA,
 * we need a compression of 10 to reduce 1000 mX to a display value of 100.
 * If we want to have a 20 mX resolution we need a compression of 1.
 *    Then a value of 100mX gives a display value of 100.
 */
void readAndDrawEEPROMValues() {
    /*
     * Process VOLTAGE Y label and offset
     * We must compute offset and compression factor before storing it to 8 bit compressed data array for chart display
     * This is 1.4 volt for 8 bit data
     * To show range in 7 grids we must divide by 6 to have room for upper and lower margin to grid line
     */
    uint16_t tDeltaMillivoltPerGrid = (ValuesForChartScaling.maxVoltageNoLoadMillivolt
            - ValuesForChartScaling.minVoltageNoLoadMillivolt) / (GRIDS_VERTICAL_Y - 1);
    uint16_t tMillivoltPerGrid = 100; // 100, 200, 500, 1000, 2000 -> CompressionFactor 5, 10, 25, 50, 100

    /*
     * Compute which tMillivoltPerGrid Y range to choose for full display of voltage
     * Use 1,2,5 raster
     */
    while (true) {
        if (tDeltaMillivoltPerGrid < tMillivoltPerGrid) {
            break; // < 100, 1000 etc.
        }
        if (tDeltaMillivoltPerGrid < tMillivoltPerGrid * 2) {
            tMillivoltPerGrid *= 2; // < 200, 2000 etc.
            break;
        }
        tMillivoltPerGrid *= 5;
        if (tDeltaMillivoltPerGrid < tMillivoltPerGrid) {
            break; // < 500, 5000 etc.
        }
        tMillivoltPerGrid *= 2;
    }

    sCompressionFactor = tMillivoltPerGrid / 20;
    sChartCompressionFactor = sCompressionFactor;

    /*
     * Compute Y offset, such, that voltage starts at top of chart
     */
    uint8_t tYGridNumber = ValuesForChartScaling.maxVoltageNoLoadMillivolt / tMillivoltPerGrid;
    if (tYGridNumber > (GRIDS_VERTICAL_Y - 1)) { // > 6
        tYGridNumber -= GRIDS_VERTICAL_Y - 1; // -= 6
    } else {
        tYGridNumber = 0; // start at 0
    }
    sCompressionOffsetMillivolt = tYGridNumber * tMillivoltPerGrid; // adjust offset to grid boundary

// Parameter: YFactor = sCompressionFactor/1000 = 10/1000 = 0.01 = * 100 -> input is 100 for 1.0 (volt) and resolution is 10 mV
// Parameter: 4, 1 gives e.g. " 3.5" as label
    VoltageChart.initYLabel(sCompressionOffsetMillivolt / 1000.0, tMillivoltPerGrid / 1000.0, sCompressionFactor / 1000.0, 4, 1);

    VoltageChart.setDataColor(CHART_VOLTAGE_COLOR);
    sChartReadValueArrayType = TYPE_VOLTAGE; // process voltages
    readAndProcessEEPROMData(false);

    /*
     * Show ESR if we are just after boot and data stored was battery data
     * OR we are just not after boot and in battery mode
     */
    if ((sTesterInfo.MeasurementState == STATE_SETUP_AND_READ_EEPROM && !StartValues.inLoggerModeAndFlags)
            || (sTesterInfo.MeasurementState != STATE_SETUP_AND_READ_EEPROM && !sInLoggerModeAndFlags)) {

        _delay(HELPFUL_DELAY_BETWEEN_DRAWING_CHART_LINES_TO_STABILIZE_USB_CONNECTION);
        /*
         * Process ESR
         * Find right compression factor
         * Compression factor of 5 gives maximum of display of 700 mX, 10 -> 1400 mX.
         * To have full scale we can compare with sChartCompressionFactor * 140
         * We choose 120 to avoid that the maximum ESR chart value is higher than the maximum chart value of voltage.
         */
        if (ValuesForChartScaling.maxMilliohm > (sChartCompressionFactor * 1200L)) {
            sCompressionFactor = sChartCompressionFactor * 100; // E.g. for low battery voltage and high ESR
        } else if (ValuesForChartScaling.maxMilliohm > ((uint16_t) sChartCompressionFactor * 120)) {
            sCompressionFactor = sChartCompressionFactor * 10;
        } else if ((ValuesForChartScaling.maxMilliohm > ((uint16_t) sChartCompressionFactor * 12))
                || sChartCompressionFactor < 10) {
            sCompressionFactor = sChartCompressionFactor;
        } else {
            sCompressionFactor = sChartCompressionFactor / 10;
        }
        VoltageChart.setDataColor(CHART_ESR_COLOR);
        sChartReadValueArrayType = TYPE_ESR; // process ESR
        readAndProcessEEPROMData(false);
    }

    /*
     * Process CURRENT
     * Find right compression factor
     */
    _delay(HELPFUL_DELAY_BETWEEN_DRAWING_CHART_LINES_TO_STABILIZE_USB_CONNECTION);
    if (ValuesForChartScaling.maxMilliampere > ((uint16_t) sChartCompressionFactor * 120)) {
        sCompressionFactor = sChartCompressionFactor * 10;
    } else if ((ValuesForChartScaling.maxMilliampere > ((uint16_t) sChartCompressionFactor * 12)) || sChartCompressionFactor < 10) {
        sCompressionFactor = sChartCompressionFactor;
    } else {
        sCompressionFactor = sChartCompressionFactor / 10;
    }
    VoltageChart.setDataColor(CHART_CURRENT_COLOR);
    sChartReadValueArrayType = TYPE_CURRENT; // process current
    readAndProcessEEPROMData(false);

    /*
     * Print chart statistics/values, if we have more than just start data
     */
    if (ValuesForDeltaStorage.DeltaArrayIndex > 0) {
        printChartValues();
    }
#if defined ENABLE_DISPLAY_OF_DATE_AND_TIME
    printTimeAtOneLine(BUTTONS_START_X, BlueDisplay1.getRequestedDisplayHeight() - BASE_TEXT_SIZE, (BASE_TEXT_SIZE * 2) / 3,
    COLOR16_RED, sBackgroundColor);
#endif
}

void printCapacityValue() {
    char tString[18];
    if (sTesterInfo.isStandardCapacityAvailable) {
// 70 bytes
        snprintf_P(tString, sizeof(tString), PSTR("%5u | %u mAh"), sBatteryOrLoggerInfo.CapacityMilliampereHour,
                sTesterInfo.StandardCapacityMilliampereHour);
    } else {
        snprintf_P(tString, sizeof(tString), PSTR("%5u mAh        "), sBatteryOrLoggerInfo.CapacityMilliampereHour);
    }
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, BlueDisplay1.getRequestedDisplayHeight() - (sChartDataTextSize * 9), tString,
            sChartDataTextSize, sTextColor, sBackgroundColor);
}

/*
 * Print all chart related values at the lower right of screen display
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
            StartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE_SHORT); // Samples + start sample
    tYPosition += 2 * sChartDataTextSize;
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, sTextColor, sBackgroundColor);

// Duration + VCC
    sVCCVoltageMillivolt = getVCCVoltageMillivolt();
    uint16_t tDurationMinutes = ValuesForDeltaStorage.DeltaArrayIndex
            * (StartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE);
    snprintf(tStringBuffer, sizeof(tStringBuffer), "%2u:%02u h       VCC", tDurationMinutes / MINUTES_IN_ONE_HOUR_SHORT,
            tDurationMinutes % MINUTES_IN_ONE_HOUR_SHORT);
    dtostrf(sVCCVoltageMillivolt / 1000.0, 5, 2, &tStringBuffer[8]);
    tStringBuffer[13] = ' '; // overwrite terminating null
    tYPosition += sChartDataTextSize;
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, sTextColor, sBackgroundColor);

// Load and cutoff
    dtostrf(StartValues.LoadResistorMilliohm / 1000.0, 5, 2, tStringBuffer);
    snprintf(&tStringBuffer[5], sizeof(tStringBuffer), " \x81 Load   "); // strcat(tStringBuffer, " m\x82 Load") requires more program space
    tStringBuffer[14] = sBatteryOrLoggerInfo.CutoffLevelCharacter;
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
 * Version 5.2 - 8/2025
 *  - Improved chart scaling.
 *  - Display of date and time for BlueDisplay mode.
 *  - Refactoring.
 *  - Added battery specific zero value.
 *
 * Version 5.1 - 3/2025
 *  - Adaptive Chart data text size.
 *  - Improved MeasurementState BDButton handling.
 *
 * Version 5.0 - 2/2025
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
