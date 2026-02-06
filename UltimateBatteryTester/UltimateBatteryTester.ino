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
 *  The current and voltage is sampled 769 times per second.
 *  Displayed and stored voltage is average of all samples, but for future use maximum and minimum are also available.
 *  If voltage or current is available logging starts.
 *  Both values are written to EEPROM. Zero ESR value is written too, but is not displayed in logger mode.
 *
 *
 *  Copyright (C) 2021-2026  Armin Joachimsmeyer
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

#define VERSION_EXAMPLE "7.0"
// The change log is at the bottom of the file

//#define TRACE
//#define DEBUG

//#define NO_TONE_WARNING_FOR_VOLTAGE_TOO_LOW_FOR_STANDARD_CAPACITY_COMPUTATION

/*
 * If you want, you can calibrate your ADC readout by replacing this value with the voltage you measured a the AREF pin after the program started.
 * For my Nanos I measured e.g. 1060 mV and 1093 mV.
 */
#if !defined(ADC_INTERNAL_REFERENCE_MILLIVOLT)
#define ADC_INTERNAL_REFERENCE_MILLIVOLT    1100 // Change to value measured at the AREF pin. If value > real AREF voltage, measured values are > real values
#endif

/*
 * In WOKWI, we cannot simulate voltage division with resistors, so me must tweak it
 */
//#define IN_WOKWI
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
 * Choose the type of LCD you use
 * Default is parallel LCD with 2 rows of 16 characters (1602).
 * 2004 Display is and will not be supported. Use BlueDisplay app instead.
 * Serial LCD uses A4/A5 - the hardware I2C pins on Arduino
 */
//#define USE_PARALLEL_1602_LCD // Is default and set by LCDPrintUtils.hpp
//#define USE_SERIAL_1602_LCD
//#define USE_NO_LCD
#if !defined(USE_NO_LCD)
#define USE_LCD
#include "LCDPrintUtils.hpp" // sets USE_PARALLEL_LCD or USE_SERIAL_LCD
#  if defined(USE_SERIAL_LCD)
#define USE_SOFT_I2C_MASTER // Requires SoftI2CMaster.h + SoftI2CMasterConfig.h. Saves 2110 bytes program memory and 200 bytes RAM compared with Arduino Wire
#include "LiquidCrystal_I2C.hpp"  // This defines USE_SOFT_I2C_MASTER, if SoftI2CMasterConfig.h is available. Use only the modified version delivered with this program!
LiquidCrystal_I2C myLCD(0x27, LCD_COLUMNS, LCD_ROWS);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#  else
#include "LiquidCrystal.h"
//LiquidCrystal myLCD(2, 3, 4, 5, 6, 7);
//LiquidCrystal myLCD(7, 8, A0, A1, A2, A3);
LiquidCrystal myLCD(7, 8, 3, 4, 5, 6);
#  endif
#endif //!defined(USE_NO_LCD)
#define LCD_MESSAGE_PERSIST_TIME_MILLIS     2000 // 2 second to view a message on LCD - is also used in delayAndCheckForButtonPress()

/*
 * Pin and ADC definitions
 *
 * Pin 2 / INT0 is used for Start/Stop button
 * Pin 3 to 8 are used for parallel LCD connection
 * Pins A1 to A5 are used as digital outputs for range and load switching (and buzzer)
 */
#define ADC_CHANNEL_FOR_VOLTAGE          0 // Pin A0 / PC0 for Uno. This is the ADC channel, not the pin for voltage measurement!
// the next Analog pins are used as digital outputs for range and load switching (and buzzer)
#define VOLTAGE_RANGE_EXTENSION_1_PIN   A1 // PC1 This pin is low to extend the voltage range from 2.2 volt to 4.4 volt
#define VOLTAGE_RANGE_EXTENSION_2_PIN   A2 // PC2 This pin is low to extend the voltage range from 2.2 volt to 17.6 volt
#define VOLTAGE_RANGE_EXTENSION_3_PIN   A3 // PC3 This pin is low to extend the voltage range from 2.2 volt to  volt
#define ADC_CHANNEL_CURRENT                     6 // Pin A6 for Nano. This is the ADC channel, not the pin for current measurement!
#define ADC_CHANNEL_LOGGER_CURRENT              7 // Pin A7 for Nano. This is the ADC channel, not the pin for current measurement for Logger!
#define HIGHEST_ASSEMBLED_VOLTAGE_RANGE_MASK    0x08 // A3 - This pin must be actually assembled with a resistor
#define HIGHEST_ASSEMBLED_VOLTAGE_RANGE         3 // Channel
#define LOWEST_VOLTAGE_RANGE_PORT_MASK_DUMMY    0x01 // Mask for PC0! not applied to hardware.
#define VOLTAGE_RANGE_PORT_MASK                 0xE0 // PC1 to PC3

#if defined(USE_SERIAL_LCD)
// A4 + A5, the hardware I2C pins on Arduino, are used for Serial LCD, so we must redefine this 2 pins
#define LOAD_HIGH_PIN                           4 // This pin is high to switch on the high load (3 ohm)
#define BUZZER_PIN                              3
#define BUZZER_PIN_MASK                         0 // no Buzzer at Analog pins
#define OTHER_OUTPUTS_PIN_MASK                  0 // Alternate functions for Pin 4 + 5
#else
// Parallel LCD here
#define LOAD_HIGH_PIN                           A4 // PC4 This pin is high to switch on the high load (3 ohm)
#define LOAD_HIGH_PIN_MASK                      0x10 // Output PC4
#define BUZZER_PIN                              A5 // PC5
#define BUZZER_PIN_MASK                         0x20 // Output PC4
#define OTHER_OUTPUTS_PIN_MASK                  (LOAD_HIGH_PIN_MASK | BUZZER_PIN_MASK)
#endif // defined(USE_SERIAL_LCD)

// Mode pins
#define ONLY_PLOTTER_OUTPUT_PIN      9 // Verbose output to Arduino Serial Monitor is disabled, if connected to ground. This is intended for Arduino Plotter mode.
#define ONLY_LOGGER_MODE_PIN        10 // If connected to ground, current is measured at the shunt at channel 4 / A4 and voltage still at channel 0 / pin A0.
#define CUTOFF_LEVEL_PIN            11 // If connected to ground, CUTOFF_LEVEL_LOW is taken as startup default if no battery is inserted at startup.
#define LOAD_LOW_PIN                12 // This pin is high to switch on the low load (10 ohm). A4 is occupied by I2C for serial LCD display.
bool sLastValueOfCutoffLevelPin; // To support prints and voltage setting at changing between normal and low by using pin CUTOFF_LEVEL_PIN

/*
 * External circuit definitions
 */
#if !defined(LOGGER_SHUNT_RESISTOR_MILLIOHM)
#define LOGGER_SHUNT_RESISTOR_MILLIOHM          200    // 0.2 ohm -> Resolution of 5 mA, but we display the average of 769 values per second
#endif
#if !defined(ESR_SHUNT_RESISTOR_MILLIOHM)
#define ESR_SHUNT_RESISTOR_MILLIOHM             2000   // 2 ohm
#endif
#define LOAD_LOW_MILLIOHM                       (1000 + ESR_SHUNT_RESISTOR_MILLIOHM) // Additional 1 ohm
#define LOAD_HIGH_MILLIOHM                      (10 * 1000 + ESR_SHUNT_RESISTOR_MILLIOHM) // Additional 10 ohm
#define ATTENUATION_FACTOR_VOLTAGE_RANGE_0      2      // Divider with 100 kOhm and 100 kOhm -> 2.2 V range
#define ATTENUATION_FACTOR_VOLTAGE_RANGE_1      4      // Divider with 100 kOhm and 33.333 kOhm -> 4.4 V range
#define ATTENUATION_FACTOR_VOLTAGE_RANGE_2      16     // Divider with 100 kOhm and 6.666 kOhm -> 17.6 V range
#define ATTENUATION_FACTOR_VOLTAGE_RANGE_3      59     // Divider with 100 kOhm and 1.7 kOhm -> 64.9 V range - 59 to avoid 16 bit overflow for ...Millivolt

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
 * Cutoff level
 */
#define CUTOFF_LEVEL_HIGH       0 // Switch off current percentage is 50% (shift 1 right) for logger. Is default case
#define CUTOFF_LEVEL_LOW        1 // 25% (shift 2 right) for logger.
#define CUTOFF_LEVEL_ZERO       2 // 12% (shift 3 right) for logger.

/*********************
 * Measurement timing
 *********************/
//#define TEST // to speed up testing the code
#if defined(TEST)
#define NUMBER_OF_INITIAL_SAMPLES               4 // 4 seconds (-2 for initial display of append message) before starting discharge and storing, to have time to just test for ESR of battery.
#define SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS   500 // The time of the activated load for one sample.
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
#define LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT 769 // for ADC_PRESCALE32 and 20 ms (50 Hz)
#define LOGGER_SAMPLE_PERIOD_MILLIS             50 // 20 Hz.
#define LOGGER_SAMPLE_FREQUENCY_HZ              (MILLIS_IN_ONE_SECOND / LOGGER_SAMPLE_PERIOD_MILLIS) // 20 Hz

#define MAX_VALUES_DISPLAYED_IN_PLOTTER         500 // The Arduino 1.8 Plotter displays 500 values before scrolling
#define BATTERY_OR_VOLTAGE_DETECTION_PERIOD_MILLIS         (MILLIS_IN_ONE_SECOND / 2) // 500 ms

/*******************
 * Attention timing
 *******************/
#define STATE_BATTERY_DETECTION_ATTENTION_PERIOD_MILLIS     (MILLIS_IN_ONE_SECOND * SECONDS_IN_ONE_MINUTE) // 60000 fits in uint16_t
#define STATE_STOP_ATTENTION_PERIOD_MILLIS                  ((uint32_t)MILLIS_IN_ONE_SECOND * SECONDS_IN_ONE_MINUTE * 10)

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
#  if !defined(ALLOW_SERIAL_PRINT_STOP_CONDITION)
#define SUPPRESS_SERIAL_PRINT_STOP_CONDITION // To reduce code size
#  endif
#  if !defined(ALLOW_SERIAL_PRINT_FOUND_REMOVING_CONDITION)
//#define SUPPRESS_SERIAL_PRINT_FOUND_REMOVING_CONDITION // To reduce code size. - not required yet
#  endif

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
#define DISPLAY_WIDTH   ((MAX_NUMBER_OF_SAMPLES * 33L) / 20) // 556
#define BASE_TEXT_SIZE  (MAX_NUMBER_OF_SAMPLES / 20) // 16 - #define TEXT_SIZE_16_HEIGHT 18, #define TEXT_SIZE_16_WIDTH 10
#define BASE_TEXT_WIDTH ((((MAX_NUMBER_OF_SAMPLES / 20) * 6 ) + 4) / 10) // 10
#define BUTTON_WIDTH    (BASE_TEXT_SIZE * 5) // 80
#define CHART_START_X   (BASE_TEXT_SIZE * 3) // 48
#define CHART_WIDTH     (MAX_NUMBER_OF_SAMPLES + 1) // 337, +1 for the first sample at minute 0 -> 337, 5 hours and 36 min
#define CHART_END_X     (CHART_START_X + CHART_WIDTH)
#define CHART_AXES_SIZE (BASE_TEXT_SIZE / 8) // 2

#define BUTTONS_START_X (CHART_END_X + BASE_TEXT_SIZE)

#define CHART_VALUES_POSITION_X     (CHART_END_X)

#define PROBE_VALUES_TEXT_SIZE      (BASE_TEXT_SIZE * 2)
#define PROBE_VALUES_POSITION_X     (BASE_TEXT_SIZE * 2)
#define PROBE_VALUES_POSITION_Y     (BASE_TEXT_SIZE / 2)
#define MESSAGE_START_POSITION_Y    ((BASE_TEXT_SIZE * 2) + (BASE_TEXT_SIZE / 2))

#define VOLTAGE_POSITION_X          (PROBE_VALUES_POSITION_X)
#define ESR_POSITION_X              (PROBE_VALUES_POSITION_X + (BASE_TEXT_SIZE * 20))
#define CURRENT_POSITION_X          (PROBE_VALUES_POSITION_X + (BASE_TEXT_SIZE * 10))

#define CHART_VOLTAGE_COLOR     COLOR16_RED
#define CHART_ESR_COLOR         COLOR16_GREEN
#define CHART_CURRENT_COLOR     COLOR16_BLUE

#define CHART_AXES_COLOR        COLOR16_BLUE
#define CHART_GRID_COLOR        COLOR16_YELLOW
#define CHART_DATA_COLOR        COLOR16_RED
#define CHART_TEXT_COLOR        COLOR16_BLACK

#define CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED 30

/*
 * Brightness handling
 */
#define BRIGHTNESS_LOW      2
#define BRIGHTNESS_MIDDLE   1
#define BRIGHTNESS_HIGH     0
#define START_BRIGHTNESS    BRIGHTNESS_HIGH // 0 is smaller code to initialize
unsigned long sMillisOfLastRefreshOrChangeBrightness;
#define TIMEOUT_FOR_BRIGHTNESS_MILLIS      4000 // Before 4 seconds the next touch is interpreted as brightness change request
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
 * Factor to convert the original mV, mOhm, mA to the 8 bit chart array data. The offset (for voltage) is subtracted before compression.
 * Compression and offset values are transfered to host, to expand data.
 *
 * sCompressionFactor 10 means maximum (delta) value is 2.55 V, A, Ohm.
 * We have 7 grids for Y axis and use a 1, 2, 5 scheme.
 * In order to achieve optimal resolution for factor 10, it is necessary to select a 200 mV grid.
 * This selection enables the display of a maximum of 1.4 V / 1400 mV (input is then 140).
 * sCompressionFactor 25 and a scale value of 500 mV grid is required to ensure optimal resolution for 1.4 V to 3.5 V.
 *
 * sCompressionFactor  10 -> 0.2 V grid and 1.4 V maximum range at value  1400/10 = 140.
 * sCompressionFactor  25 -> 0.5 V grid and 3.5 V maximum range at value  3500/25 = 140.
 * sCompressionFactor  50 ->   1 V grid and   7 V maximum range at value  7000/50 = 140.
 * sCompressionFactor 100 ->   2 V grid and  14 V maximum range at value 14000/100= 140.
 * Minimum sensible compression factor for voltage is 5 -> 100 mV grid, 700 mV range.
 * Maximum sensible compression factor for voltage is 500 -> 10 V grid, 70 V range.
 *
 */
uint16_t sCompressionFactorTimes10; // Times 10 because we must avoid sChartCompressionFactor being 25 and sCompressionFactor = sChartCompressionFactor / 10.
uint16_t sVoltageChartCompressionFactor; // 50 to 5000 (initial) factor for mV to chart array data, required to compute sCompressionFactorTimes10 for ESR and current.
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

#if !defined(MILLIS_IN_ONE_SECOND)
#define MILLIS_IN_ONE_SECOND    1000U

#if !defined(SECONDS_IN_ONE_MINUTE)
#define SECONDS_IN_ONE_MINUTE   60U
#endif
#if !defined(SECONDS_IN_ONE_HOUR)
#define SECONDS_IN_ONE_HOUR     3600U
#endif
#if !defined(SECONDS_IN_ONE_DAY)
#define SECONDS_IN_ONE_DAY      86400U
#endif
#if !defined(MINUTES_IN_ONE_HOUR)
#define MINUTES_IN_ONE_HOUR     60U
#endif
#if !defined(MINUTES_IN_ONE_DAY)
#define MINUTES_IN_ONE_DAY      1440U
#endif
#if !defined(HOURS_IN_ONE_DAY)
#define HOURS_IN_ONE_DAY        24U
#endif
#endif

//#define ENABLE_STACK_ANALYSIS
#if defined(ENABLE_STACK_ANALYSIS)
#include "AVRUtils.h" // include for initStackFreeMeasurement() and printRAMAndStackInfo()
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
#define LI_ION_SWITCH_OFF_SETTLING_MILLIS            20 // Time to switch of load to before measuring no load voltage - heuristic value

#define NIMH_STANDARD_FULL_VOLTAGE_MILLIVOLT       1340 // Start voltage for NI-MH standard capacity measurement
#define NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT          1100 // Switch off voltage for NI-MH capacity measurement
#define NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW      1000 // Switch off voltage for extended capacity measurement
#define NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO      140 // witch off voltage for extended capacity measurement
#define OTHER_SWITCH_OFF_SETTLING_MILLIS            100 // Time to switch of load to before measuring no load voltage / settling time

#define NO_LOAD     0
#define LOW_LOAD    1 // 12 ohm
#define HIGH_LOAD   2 // 3 ohm

#define TYPE_INDEX_NO_BATTERY    0
#define TYPE_INDEX_DEFAULT       6
#define TYPE_INDEX_MAX          10
#define TYPE_INDEX_LOGGER       42

// @formatter:off
struct BatteryTypeInfoStruct BatteryTypeInfoArray[] = {
    { "No battery", NO_BATTERY_MILLIVOLT, 0, 0, 0, 0, NO_LOAD, 0 }, /* Below 100 mV and not below 50, to avoid toggling between no and low batt */
    { "Low batt. ", 1000, 800, 300, 100, 70, HIGH_LOAD, OTHER_SWITCH_OFF_SETTLING_MILLIS }, /* For researching of worn out batteries. */
    { "NiCd NiMH ", 1460, NIMH_STANDARD_FULL_VOLTAGE_MILLIVOLT, NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT,
            NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW, NIMH_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO, HIGH_LOAD, OTHER_SWITCH_OFF_SETTLING_MILLIS }, /*400 mA*/
    { "Alkali    ", 1550, 1500, 1300, 1000, 70, HIGH_LOAD, OTHER_SWITCH_OFF_SETTLING_MILLIS }, /*500 mA*/
    { "NiZn batt.", 1850, 1650, 1400, 1300, 100, HIGH_LOAD, OTHER_SWITCH_OFF_SETTLING_MILLIS }, /*550 mA*/
    { "LiFePO4   ", 3400, 3400, 3050, 2700, 180, LOW_LOAD, LI_ION_SWITCH_OFF_SETTLING_MILLIS }, /*270 mA https://www.jackery.com/blogs/knowledge/ultimate-guide-to-lifepo4-voltage-chart*/
    { "Li-ion    ", 5000, LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT /*4100*/, LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT/*3400*/,
            LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW/*3V*/, LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO /*180mV*/,
            LOW_LOAD, LI_ION_SWITCH_OFF_SETTLING_MILLIS }, /*300 mA*/
    { "LiIo 2pack", 2 * LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT/*8.6V*/, 2 * LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT,
            2 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT /*7V*/, 2 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW /*6V*/,
            2 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO /*360mV*/, LOW_LOAD, LI_ION_SWITCH_OFF_SETTLING_MILLIS }, /*620 mA*/
    { "9 V Block ", 9200, 9000, 7700, 7000, 200, LOW_LOAD, OTHER_SWITCH_OFF_SETTLING_MILLIS }, /*750 mA => external series load resistor recommended*/
    { "LiIo 3pack", 3 * LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT/*12.9V*/, 3 * LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT,
            3 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT, 3 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW,
            3 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO, LOW_LOAD, LI_ION_SWITCH_OFF_SETTLING_MILLIS }, /*925 mA*/
    { "LiIo 4pack", 4 * LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT/*17.2V*/, 4 * LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT,
            4 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT, 4 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW,
            4 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO, LOW_LOAD, LI_ION_SWITCH_OFF_SETTLING_MILLIS }, /*1233 mA*/
    {"LiIo 5pack", 5 * LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT/*21.5V*/, 5 * LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT,
            5 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT, 5 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW,
            5 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO, LOW_LOAD, LI_ION_SWITCH_OFF_SETTLING_MILLIS },
    {"LiIo 6pack", 6 * LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT/*25.8V*/, 6 * LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT,
            6 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT, 6 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW,
            6 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO, LOW_LOAD, LI_ION_SWITCH_OFF_SETTLING_MILLIS },
    {"LiIo Xpack", 7 * LI_ION_MAX_FULL_VOLTAGE_MILLIVOLT/*30.1V*/, 7 * LI_ION_STANDARD_FULL_VOLTAGE_MILLIVOLT,
            7 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT, 7 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_LOW,
            7 * LI_ION_SWITCH_OFF_VOLTAGE_MILLIVOLT_ZERO, LOW_LOAD, LI_ION_SWITCH_OFF_SETTLING_MILLIS }
};
// @formatter:on

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
            uint16_t MinimumMillivolt; // only used for Serial output
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

    uint8_t LoadSwitchSettleTimeMillis; // Time for the battery to reach its unloaded voltage
} sBatteryOrLoggerInfo;

struct lastDiplayedValuesStruct {
    uint16_t VoltageNoLoadMillivolt;
    uint16_t Milliampere;
    uint16_t ESRMilliohm;
    uint16_t CapacityMilliampereHour;
} sLastDiplayedValues;

struct Logger1SecondAccumulatorStruct {
    uint32_t RawVoltageAccumulator; // normalized for 1.1 V range - maximum is 2 (AttenuationFactor) * 1023 (MaxRawVoltage) * 769 (samples) * 20 (Hz) = 31,467,480
    uint32_t RawCurrentAccumulator;    // maximum is 1023 (MaxRawCurrent) * 769 (samples) * 20 (Hz) = 15,733,740
    uint16_t RawSampleCount;           // Number of 769 samples. 20 for 20 Hz
    uint16_t MinimumRawVoltage;
    uint16_t MaximumRawVoltage;
} sLogger1SecondAccumulator;

struct Logger1MinuteAccumulatorStruct {
    uint16_t RawSampleCount;          // Number of 769 samples. Is 1200 every minute for 20 Hz
    uint32_t RawVoltageAccumulator8ShiftRight; // Unshifted maximum at 2.2 V range is 1023 (MaxRawVoltage) * 769 (samples) * 20 (Hz) * 60 (sec) = 944,024,400 => 8.8 V range would be OK for unshifted value
    uint32_t RawCurrentAccumulator;  // Unshifted maximum is 1023 (MaxRawCurrent) * 769 (samples) * 20 (Hz) * 60 (sec) = 944,024,400
} sLogger1MinuteAccumulator;

/*
 * Flags for logger mode
 */
#define NO_LOGGER_MODE_REQUESTED            0
#define LOGGER_MODE_REQUESTED               1
#define LOGGER_EXTERNAL_CURRENT_DETECTED    2 // one time flag during measurement
#define LOGGER_EXTERNAL_VOLTAGE_DETECTED    4 // one time flag during measurement
#define LOGGER_MODE_MASK                    LOGGER_MODE_REQUESTED

struct TesterInfoStruct {
    uint8_t inLoggerModeAndFlags; // Initially contains the (inverted) value of the pin ONLY_LOGGER_MODE_PIN

    volatile uint8_t MeasurementState; // One of STATE_SETUP_AND_READ_EEPROM, STATE_WAITING_FOR_BATTERY_OR_EXTERNAL, STATE_INITIAL_SAMPLES etc.
    bool MeasurementWasFinishedByEndCondition; // true if stopped by detecting end condition and not by user button.

    uint8_t VoltageRange;           // 0 to 3, for 2.2 V, 4.4 V, 17.6 V, x.x V and
    uint8_t VoltageRangePortMask;   // 0x01 to 0x08, for 2.2 V, 4.4 V, 17.6 V, x.x V and

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
#define HISTORY_SIZE_FOR_ESR_AVERAGE    (INITIAL_NUMBER_OF_SECONDS_PER_STORAGE * 2)
uint16_t sESRHistory[HISTORY_SIZE_FOR_ESR_AVERAGE]; //  Current value is in sESRHistory[0].
uint8_t sESRHistoryLength; // keep track of number of valid inputs used for append, where history array is not filled up completely

// Override defaults defined in ADCUtils.h
#define LI_ION_VCC_UNDERVOLTAGE_THRESHOLD_MILLIVOLT 3500 // 3.5 volt
#define VCC_CHECK_PERIOD_MILLIS                     60000 // check every minute
#define VCC_UNDERVOLTAGE_CHECKS_BEFORE_STOP         5 // Shutdown after 5 times below VCC_UNDERVOLTAGE_THRESHOLD_MILLIVOLT or below VCC_EMERGENCY_UNDERVOLTAGE_THRESHOLD_MILLIVOLT
#if !defined(SUPPRESS_SERIAL_PRINT)
#define LOCAL_INFO // For Serial output at isVCCUndervoltageMultipleTimes(). This is undefined after the include!
#endif
#include "ADCUtils.hpp"

/************************************************************************************************
 * EEPROM store, It seems that EEPROM is allocated top down and in called or referenced sequence
 * https://arduino.stackexchange.com/questions/93873/how-eemem-maps-the-variables-avr-eeprom-h
 ************************************************************************************************/
// The start values for the delta array
struct EEPROMStartValuesStruct {
    uint16_t initialMillivolt;
    uint16_t initialMilliampere;
    uint16_t initialDischargingMilliohm;
    uint16_t LoadResistorMilliohm;
    uint16_t CapacityMilliampereHour; // Is set at end of measurement or by store button
    uint8_t CutoffLevel; // One of CUTOFF_LEVEL_HIGH, CUTOFF_LEVEL_LOW and CUTOFF_LEVEL_ZERO.
    uint8_t BatteryTypeIndex;
    uint8_t inLoggerModeAndFlags; // 0 or 1 - is sTesterInfo.inLoggerModeAndFlags & LOGGER_MODE_MASK
    uint8_t NumberOfSecondsPerStorage; // INITIAL_NUMBER_OF_SECONDS_PER_STORAGE (60) and multiple like 120, 240, etc.
} ChartStartValues;
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

void getVoltageMillivolt();
void addToCapacity();
uint8_t getVoltageAttenuationFactor();
uint16_t getVoltageRaw();
void setBatteryTypeIndex(uint8_t aBatteryTypeIndex);
bool setBatteryTypeIndexFromVoltage(uint16_t aBatteryVoltageMillivolt);
bool detectBatteryOrLoggerVoltageOrCurrentLCD_BD();
void clearLogger1SecondAccumulator();
void clearLogger1MinuteAccumulator();
void getLogger1SecondValues();
void getLogger1MinuteValues();
void getPeriodicAccumulatingLoggerValues();

void getCurrent(uint8_t aADCChannel, uint16_t aShuntResistorMilliohm);
void getBatteryValues();
void checkAndHandleStopConditionLCD();
void detectVoltageOrCurrentRemoved();
void playEndTone();
void playUndervoltageAttentionTone();
void setLoad(uint8_t aNewLoadState);
void printStoredDataLCD_BD();
bool printMilliampere4DigitsLCD_BD();
void printVoltageNoLoadMillivoltWithTrailingSpaceLCD_BD();
void forceDisplayOfCurrentValues();
void printMeasurementValuesLCD_BD();
void printEEPROMChartAndPlotterGraph(uint16_t aMillivoltToPrint, uint16_t aMilliampereToPrint, uint16_t aMilliohmToPrint,
        uint16_t aChartValueArrayIndex, bool aIsLastElement);
void printMillisValueAsFloat(uint16_t aValueInMillis);
void printCounterLCD_BD(uint16_t aNumberToPrint);

void dumpEEPROM(uint8_t *aEEPROMAdress, uint8_t aNumberOf16ByteBlocks);
void storeBatteryValuesToEEPROM(uint16_t aVoltageNoLoadMillivolt, uint16_t aMilliampere, uint16_t aMilliohm);
void storeCapacityAndCutoffLevelToEEPROM_LCD();
void readAndProcessEEPROMData(bool aInitializeValuesForDisplayAndAppend);
void handlePeriodicStoringToEEPROM();
void handleStateStoppedLCD_BD();
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
void LCDPrintVCC(uint8_t aLCDLine);
#endif

#if !defined(uintDifferenceAbs)
#define uintDifferenceAbs(a, b) ((a >= b) ? a - b : b - a)
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
    pinMode(LOAD_LOW_PIN, OUTPUT);
    pinMode(ONLY_PLOTTER_OUTPUT_PIN, INPUT_PULLUP);
    pinMode(CUTOFF_LEVEL_PIN, INPUT_PULLUP);
    pinMode(ONLY_LOGGER_MODE_PIN, INPUT_PULLUP);
    setLoad(NO_LOAD);

    // Settings for highest voltage range
    pinMode(VOLTAGE_RANGE_EXTENSION_3_PIN, OUTPUT);
    sTesterInfo.VoltageRangePortMask = HIGHEST_ASSEMBLED_VOLTAGE_RANGE_MASK;
    sTesterInfo.VoltageRange = HIGHEST_ASSEMBLED_VOLTAGE_RANGE;
    PORTC = 0; // All A0 to A5 pins are initial low, if activated as outputs
    DDRC = HIGHEST_ASSEMBLED_VOLTAGE_RANGE_MASK | OTHER_OUTPUTS_PIN_MASK; // Set attenuator pin to output

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
        sTesterInfo.inLoggerModeAndFlags = LOGGER_MODE_REQUESTED;
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
        Serial.print(
                ((MAX_NUMBER_OF_SAMPLES) * (INITIAL_NUMBER_OF_SECONDS_PER_STORAGE / SECONDS_IN_ONE_MINUTE))
                        / MINUTES_IN_ONE_HOUR);
        Serial.print(F("h "));
        Serial.print(
                ((MAX_NUMBER_OF_SAMPLES) * (INITIAL_NUMBER_OF_SECONDS_PER_STORAGE / SECONDS_IN_ONE_MINUTE))
                        % MINUTES_IN_ONE_HOUR);
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
#  else
    myLCD.begin(LCD_COLUMNS, LCD_ROWS);
#  endif

    /*
     * LCD print program, version and date
     */
    LCDResetCursor(&myLCD);
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
    printRAMAndStackInfo(&Serial);
#  if !defined(BD_USE_SIMPLE_SERIAL)
        Serial.flush();
#  endif
#endif

#if defined(USE_LCD)
    _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);

#  if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (!BlueDisplay1.isConnectionEstablished()) {
#  endif
        /*
         * Do not print if connected to BlueDisplay screen, which shows chart and indirect also the mode by ("no U or I" or "no battery")
         */
        myLCD.setCursor(0, 1);
        if (sOnlyPlotterOutput) {
            myLCD.print(F("Only plotter out"));
        } else {
            myLCD.print(F("No plotter out  "));
        }
        delay(LCD_MESSAGE_PERSIST_TIME_MILLIS / 2);

        myLCD.setCursor(0, 1);
        if (sTesterInfo.inLoggerModeAndFlags) {
#  if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            Serial.println(F("Only logger mode"));
        }
#  endif
            myLCD.print(F("Only logger mode"));
            _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
        }
        LCDClearLine(&myLCD, 1); // Clear line "No plotter out  " or "Only logger mode"
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    }
#endif

#  if ((INITIAL_NUMBER_OF_SECONDS_PER_STORAGE * SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS) != 60000)
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
    readAndProcessEEPROMData(true); // First call in setup() and parameter is true. Initialize data for append and print
#endif

    printStoredDataLCD_BD();
    printlnIfNotPlotterOutput(); // end of stored data

    if (sTesterInfo.inLoggerModeAndFlags) {
        sCurrentLoadResistorAverage = LOGGER_SHUNT_RESISTOR_MILLIOHM;
    }

    /*
     * Read value to variable in order to force printing triggered by value change :-)
     */
    sLastValueOfCutoffLevelPin = digitalRead(CUTOFF_LEVEL_PIN);

    /*
     * If battery is still inserted, keep cut off level read from EEPROM data for easy append.
     * If battery was removed, cut off level can be chosen by pressing stop button.
     * If battery was changed, we still have the old BatteryTypeIndex here, showing the old cutoff voltage
     */
    getVoltageMillivolt();
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
        if (sTesterInfo.inLoggerModeAndFlags) {
            getPeriodicAccumulatingLoggerValues();
        }
        auto tMillis = millis();
        auto tTimeDifference = tMillis - sTesterInfo.LastMillisOfSample; // compute with full 32 bit -
        // For discharging, add LoadSwitchSettleTimeMillis to the second condition
        if ((sTesterInfo.inLoggerModeAndFlags && sLogger1SecondAccumulator.RawSampleCount == LOGGER_SAMPLE_FREQUENCY_HZ)
                || (!sTesterInfo.inLoggerModeAndFlags && (uint16_t) tTimeDifference // - and compare only 16 bit values, because it cannot exceed 1 second much
                >= (SAMPLE_PERIOD_OF_LOAD_ACIVATED_MILLIS + sBatteryOrLoggerInfo.LoadSwitchSettleTimeMillis))) {
            /*
             * Here sample period of one second expired.
             * The first period after switching to this states is undetermined, because LastMillisOfSample is undetermined
             * sTesterInfo.MeasurementState is STATE_INITIAL_SAMPLES or STATE_SAMPLE_AND_STORE_TO_EEPROM or STATE_STOPPED
             * Do all this every second (of battery load)
             */
            sTesterInfo.LastMillisOfSample = tMillis;

            /*
             * Get values
             */
            if (sTesterInfo.inLoggerModeAndFlags) {
                getLogger1SecondValues();
            } else {
                getBatteryValues();
            }

            if (sTesterInfo.MeasurementState == STATE_STOPPED) {
                // Print only tVoltageNoLoadMillivolt and check for periodic attention
                handleStateStoppedLCD_BD();
            } else {
                /*
                 * STATE_INITIAL_SAMPLES or STATE_SAMPLE_AND_STORE_TO_EEPROM here
                 * Check removed battery in STATE_INITIAL_SAMPLES
                 * else display values
                 */
                if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES) {
                    detectVoltageOrCurrentRemoved();
                }
                // check again for not removing detected
                if (sTesterInfo.MeasurementState != STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
                    // Here no removing detected -> add current to capacity and print measurement values
                    addToCapacity();
                    printMeasurementValuesLCD_BD();
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

    if (!sTesterInfo.inLoggerModeAndFlags) {
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
}

/*
 * Check for VCC undervoltage during measurements
 * isVCCUndervoltageMultipleTimes() checks VCC only every 10 seconds
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
    /*
     * Reset ADC reference source to internal 1.1 V
     */
    checkAndWaitForReferenceAndChannelToSwitch(ADC_CHANNEL_CURRENT, INTERNAL);
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
    auto tTimeDifference = millis() - sTesterInfo.LastMillisOfBatteryOrVoltageDetection; // compute with full 32 bit -
    if ((uint16_t) tTimeDifference >= BATTERY_OR_VOLTAGE_DETECTION_PERIOD_MILLIS) { // - and compare only 16 bit values
        // initial delay introduced by truncating to 16 bit is not critical here :-)
        sTesterInfo.LastMillisOfBatteryOrVoltageDetection = millis();

        /*
         * Check if battery was inserted or voltage connected
         */
        setLoad(NO_LOAD);
        bool tBatteryOrCurrentOrVoltageWasDetected = detectBatteryOrLoggerVoltageOrCurrentLCD_BD();
        // we waited up to 2 seconds in detectBatteryOrLoggerVoltageOrCurrentLCD_BD(), so must check if mode has not changed by button press
        if (sTesterInfo.MeasurementState == STATE_WAITING_FOR_BATTERY_OR_EXTERNAL) {
            if (tBatteryOrCurrentOrVoltageWasDetected) {
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
                LCDClearLine(&myLCD, 1); // Clear line "append to EEPROM"
#  endif
#endif
                sTesterInfo.NumbersOfInitialSamplesToGo = NUMBER_OF_INITIAL_SAMPLES;
                memset(sESRHistory, 0, sizeof(sESRHistory));
                sESRHistoryLength = 0;

                if (sTesterInfo.inLoggerModeAndFlags) {
                    /*
                     * Initialize logger accumulators
                     */
                    clearLogger1SecondAccumulator();
                    clearLogger1MinuteAccumulator();
                } else {
                    // set load for the first call of getBatteryValues() to measure the current
                    setLoad(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].LoadType);
                }

            } else {
                /*
                 * Not detected here
                 *
                 * Print VCC voltage, but not if voltage was detected or battery was inserted before,
                 * to not overwrite battery/logger voltage printed in state STATE_INITIAL_SAMPLES
                 *
                 * All:     "<Volt ><Message>" message starting at 6 with at least one space
                 * Logger:  "3.98V  No U or I"  3.98V is VCC
                 * Battery: "3.98V   No batt."  3.98V is VCC
                 * Battery: "1.500V  No batt.", if we displayed battery voltage before.
                 */
                if (!sTesterInfo.VoltageNoLoadIsDisplayedOnLCD) {
#if defined(USE_LCD)
                    LCDPrintVCC(0);
#endif
                } else {
                    printVoltageNoLoadMillivoltWithTrailingSpaceLCD_BD();
                    printlnIfNotPlotterOutput();
                }

                /*
                 * Check for attention every minute
                 */
                if (millis() - sTesterInfo.LastMillisOfStateWaitingForBatteryOrVoltageBeep >= STATE_BATTERY_DETECTION_ATTENTION_PERIOD_MILLIS) {
                    sTesterInfo.LastMillisOfStateWaitingForBatteryOrVoltageBeep = millis();
                    playUndervoltageAttentionTone();
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
    if (!sTesterInfo.inLoggerModeAndFlags
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
        LCDResetCursor(&myLCD);
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
        ChartStartValues.NumberOfSecondsPerStorage = INITIAL_NUMBER_OF_SECONDS_PER_STORAGE; // reset sample timing for first sample
        sBatteryOrLoggerInfo.CapacityAccumulator = 0;
        // Must reset this values here, because values are displayed before computed again from CapacityAccumulator
        sBatteryOrLoggerInfo.CapacityMilliampereHour = 0;
        memset(sCurrentLoadResistorHistory, 0, sizeof(sCurrentLoadResistorHistory)); // Clear history array
        // Store first EEPROM value immediately, append is done by button and waits a full period
        switchToStateSampleAndStoreToEEPROM(INITIAL_NUMBER_OF_SECONDS_PER_STORAGE);
    }
}

/*
 * Exclusively called by loop() in state STATE_STOPPED
 * Print only NoLoadMillivolt if it changed more than 5 mV
 * Print current if changed more than 1 mA
 * Check for periodic attention
 */
void handleStateStoppedLCD_BD() {
    if (abs( (int16_t )sLastDiplayedValues.VoltageNoLoadMillivolt
            - (int16_t )sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt) > VOLTAGE_DISPLAY_HYSTERESIS_MILLIVOLT_FOR_STOP) {
        printVoltageNoLoadMillivoltWithTrailingSpaceLCD_BD(); // this sets sLastDiplayedValues.VoltageNoLoadMillivolt :-)
        printlnIfNotPlotterOutput();
    }
    if (sTesterInfo.inLoggerModeAndFlags) {
        /*
         * In logger mode continue to print current value on LCD and chart
         * If current is low and measurement was finished, do not print current
         */
        if (printMilliampere4DigitsLCD_BD()) {
#if !defined(SUPPRESS_SERIAL_PRINT)
            Serial.println();
#endif
#if defined(USE_LCD)
            myLCD.setCursor(7, 0);
            if (sTesterInfo.MeasurementWasFinishedByEndCondition) {
                myLCD.print("fin"); // Overwrite "Finished" and signal with "fin", that we are finished now
            } else {
                myLCD.print("stp"); // Overwrite "Stopped" and signal with "stp", that we are stopped now
            }
#endif
        }
    }
    /*
     * Check for attention every 10 minute, after the current measurement was finished
     */
    if (millis() - sTesterInfo.LastMillisOfStateStoppedForAttentionBeep >= STATE_STOP_ATTENTION_PERIOD_MILLIS) {
        sTesterInfo.LastMillisOfStateStoppedForAttentionBeep = millis();
        playUndervoltageAttentionTone();
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
            >= ChartStartValues.NumberOfSecondsPerStorage) { // Will be optimized by compiler :-)
        sTesterInfo.SampleCountForStoring = 0;

        if (sTesterInfo.inLoggerModeAndFlags) {
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
                if (sTesterInfo.inLoggerModeAndFlags) {
                    setCutoffHighLowZeroButtonTextAndDrawButton(); // For logger, print cutoff as milliampere
                }
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

/*
 * Calls forceDisplayOfCurrentValues()
 */
void setMeasurementStateAndBDButtonText(uint8_t aMeasurementState) {
    sTesterInfo.MeasurementState = aMeasurementState;
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    setMeasurementStateButtonTextAndDrawButton();
#endif
    forceDisplayOfCurrentValues(); // To force display of all values after state change
}

void switchToStateWaitingForBatteryOrVoltage() {
    checkLeavingState();
    setMeasurementStateAndBDButtonText(STATE_WAITING_FOR_BATTERY_OR_EXTERNAL);
    printSwitchStateString();
    printlnIfNotPlotterOutput();
    sBatteryOrLoggerInfo.BatteryTypeIndex = TYPE_INDEX_MAX + 2; // to force display of "found ...", but do not set button text
    sTesterInfo.LastMillisOfStateWaitingForBatteryOrVoltageBeep = millis();

    sTesterInfo.inLoggerModeAndFlags &= ~(LOGGER_EXTERNAL_CURRENT_DETECTED | LOGGER_EXTERNAL_VOLTAGE_DETECTED); // reset detected flags
    sTesterInfo.MeasurementWasFinishedByEndCondition = false; // reset flag
}

/*
 * Exclusively called by handlePeriodicDetectionOfProbe()
 * Only state switching here, other values are set in handlePeriodicDetectionOfProbe()
 */
void switchToStateInitialSamples() {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (BlueDisplay1.isConnectionEstablished()) {
        // append is only possible, if logger modes are equal
        if ((sTesterInfo.inLoggerModeAndFlags & LOGGER_MODE_MASK) == ChartStartValues.inLoggerModeAndFlags) {
            TouchButtonAppend.drawButton();
        }
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
    checkLeavingState();            // remove append button
    forceDisplayOfCurrentValues();  // to remove display of count
#endif
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
    LCDResetCursor(&myLCD);
    myLCD.print(F("Stop measurement"));
    _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
    LCDClearLine(&myLCD, 0);
#endif
    if (tOldMeasurementState == STATE_SAMPLE_AND_STORE_TO_EEPROM) {
        storeCapacityAndCutoffLevelToEEPROM_LCD(); // Store capacity and cut off level
    }

#if defined(USE_LCD)
    LCDResetCursor(&myLCD);
    myLCD.print(F("       Stopped "));
    myLCD.print(aReasonCharacter);
#endif
}

#if defined(USE_LCD)
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
    LCDResetCursor(&myLCD);
    myLCD.print(F("Cutoff "));
    if (sTesterInfo.inLoggerModeAndFlags) {
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
                myLCD.print(F("low "));
                tDecimals = 2;
            } else if (tCutoffLevel == CUTOFF_LEVEL_ZERO) {
                if (tSwitchOffVoltageMillivolt < 1000) {
                    myLCD.print(F("z  "));
                    // print "z   0.070V" instead of "zero 1.1V"
                    tDecimals = 3;
                } else {
                    myLCD.print(F("zero ")); // tDecimals = 1;
                }
            } else { // CUTOFF_LEVEL_HIGH
                myLCD.print(F("high ")); // tDecimals = 1;
            }
            if (tSwitchOffVoltageMillivolt >= 10000) {
                tDecimals--;
            }
            myLCD.print(tSwitchOffVoltageMillivolt / 1000.0, tDecimals);
            myLCD.print('V');
        }

    }
    _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
    LCDClearLine(&myLCD, 0);
    forceDisplayOfCurrentValues();
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
 *     ChartStartValues.CutoffLevel
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
    } else if (sTesterInfo.inLoggerModeAndFlags) {
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
        if (sTesterInfo.inLoggerModeAndFlags) {
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
    LCDResetCursor(&myLCD);
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
        // append is only possible, if logger modes are equal
        if ((ChartStartValues.inLoggerModeAndFlags & LOGGER_MODE_MASK) == sTesterInfo.inLoggerModeAndFlags) {
            LCDResetCursor(&myLCD);
            myLCD.print(F("Press button to "));
            myLCD.setCursor(0, 1);
            myLCD.print(F("append to EEPROM"));
        }
        /*
         * and wait for 2 seconds for button press
         */
        delayAndCheckForButtonPress();
        myLCD.clear();
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
            LCDResetCursor(&myLCD);
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

            } else if ((sTesterInfo.inLoggerModeAndFlags & LOGGER_MODE_MASK) == ChartStartValues.inLoggerModeAndFlags) {
                // STATE_INITIAL_SAMPLES here and logger modes are equal
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

            } else if (sTesterInfo.MeasurementState == STATE_INITIAL_SAMPLES
                    && ((sTesterInfo.inLoggerModeAndFlags & LOGGER_MODE_MASK) == ChartStartValues.inLoggerModeAndFlags)) {
                /*****************************************************
                 * APPEND by button press is done here
                 * append is only possible, if logger modes are equal
                 *****************************************************/
                /*
                 * No stop requested during 2 seconds wait -> append to EEPROM not canceled
                 * Store next sample in 60 seconds, because we assume double press directly after entering state STATE_INITIAL_SAMPLES
                 * Otherwise we would start the appended data with a short sampling period.
                 */
                setBatteryTypeIndex(ChartStartValues.BatteryTypeIndex); // Restore original type index. This sets cutoff level accordingly
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
void getVoltageMillivolt() {
    uint16_t tInputVoltageRaw = getVoltageRaw();
    /*
     * Compute voltage from RawVoltage and AttenuationFactor
     */
    uint16_t tCurrentBatteryVoltageMillivolt = (((uint32_t) (ADC_INTERNAL_REFERENCE_MILLIVOLT * getVoltageAttenuationFactor())
            * tInputVoltageRaw) / 1023);

    if (sBatteryOrLoggerInfo.LoadState == NO_LOAD) {
        sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt = tCurrentBatteryVoltageMillivolt;
    } else {
        sBatteryOrLoggerInfo.Voltages.Battery.LoadMillivolt = tCurrentBatteryVoltageMillivolt;
    }

#if defined(LOCAL_TRACE)
    if (!sOnlyPlotterOutput) {
        Serial.print(F("VoltageRaw="));
        Serial.print(tInputVoltageRaw);
        Serial.print(F(" Range="));
        Serial.print(sTesterInfo.VoltageRange);
        Serial.print(F(" AttenuationFactor="));
        Serial.print(getVoltageAttenuationFactor());
        Serial.print(F(" -> "));
        Serial.print(tCurrentBatteryVoltageMillivolt);
        Serial.println(F(" mV"));
    }
#endif
}

uint8_t getVoltageAttenuationFactor() {
    uint8_t tVoltageAttenuationFactor = ATTENUATION_FACTOR_VOLTAGE_RANGE_0; // Factor 2, 2.2 V range
    if (sTesterInfo.VoltageRange == 1) {
        tVoltageAttenuationFactor = ATTENUATION_FACTOR_VOLTAGE_RANGE_1;
#if HIGHEST_ASSEMBLED_VOLTAGE_RANGE >= 2
    } else if (sTesterInfo.VoltageRange == 2) {
        tVoltageAttenuationFactor = ATTENUATION_FACTOR_VOLTAGE_RANGE_2;
#  if HIGHEST_ASSEMBLED_VOLTAGE_RANGE >= 3
    } else if (sTesterInfo.VoltageRange == 3) {
        tVoltageAttenuationFactor = ATTENUATION_FACTOR_VOLTAGE_RANGE_3;
#  endif
#endif
    }
    return tVoltageAttenuationFactor;
}

/*
 * Sets NoLoadMillivolt or LoadMillivolt
 * Provides automatic range switch between 2.2, 4.4 and 14 (up to 20 with 5V VCC) volt range
 * The ranges are realized by a
 * divider with 100 kOhm and 100 kOhm -> 2.2 V range and 2.15 mV resolution
 * divider with 100 kOhm and 33.333 kOhm (3 * 100 kOhm parallel) -> 4.4 V range and 4.3 mV resolution
 * divider with 100 kOhm and 6.666 kOhm (15 * 100 kOhm parallel or 100k + 10k + (10+15k) parallel) -> 17.6 V (1.1*16) range and 17 mV resolution
 * divider with 100 kOhm and 1,71 kOhm (100k + 1.8k + 52k parallel) -> 64.9V range and 63 mV resolution
 * Does not affect the loads
 */
uint16_t getVoltageRaw() {
    setADCChannelForNextConversionAndWaitUsingInternalReference(ADC_CHANNEL_FOR_VOLTAGE);
    uint16_t tInputVoltageRaw = readADCChannel();
#if defined(IN_WOKWI)
    tInputVoltageRaw = analogRead(A0); // use VCC reference
    sTesterInfo.VoltageRange = tInputVoltageRaw / 256; // from 0 to 3
    tInputVoltageRaw = (tInputVoltageRaw % 256) * 4; // 4 times from 0 to 1023
//    if (!sOnlyPlotterOutput) {
//        Serial.print(F("Range="));
//        Serial.print(sTesterInfo.VoltageRange);
//        Serial.print(F(", VoltageRaw="));
//        Serial.println(tInputVoltageRaw);
//    }
    return tInputVoltageRaw;
#endif
    bool tCheckAgainForRangeDecrease = true;
    /*
     * Automatic range
     * First check if we have an overflow and can increase the range
     * Second check if we have an underflow and can decrease the range
     */
    while (tInputVoltageRaw >= 0x3F0 && sTesterInfo.VoltageRange < HIGHEST_ASSEMBLED_VOLTAGE_RANGE) { // 1008
        // switch to higher voltage range - 600 us per switch
        sTesterInfo.VoltageRangePortMask <<= 1; // Shift to next attenuator pin
        DDRC = sTesterInfo.VoltageRangePortMask | OTHER_OUTPUTS_PIN_MASK; // Set bit for A1/PC1 to A3/PC3
        sTesterInfo.VoltageRange++;
        tCheckAgainForRangeDecrease = false;
        // no wait, since last reading was same channel, same reference
        tInputVoltageRaw = readADCChannel();
#if defined(LOCAL_DEBUG)
        if (!sOnlyPlotterOutput) {
            Serial.print(F("Range++="));
            Serial.print(sTesterInfo.VoltageRange);
            Serial.print(F(", VoltageRaw="));
            Serial.println(tInputVoltageRaw);
        }
#endif
    }

    while (tCheckAgainForRangeDecrease) {
        if (sTesterInfo.VoltageRange == 1) {
            tCheckAgainForRangeDecrease = false;
            // Here we have factor 2 between attenuator range 2.2 V and 4.4 V
            if (tInputVoltageRaw < (((0x3F0L * ATTENUATION_FACTOR_VOLTAGE_RANGE_0) / ATTENUATION_FACTOR_VOLTAGE_RANGE_1) - 0x10)) {
                // switch to lowest (2.2 V) voltage range at values below 488 in 4.4 V range by deactivating all range extension resistors
                sTesterInfo.VoltageRange = 0;
                sTesterInfo.VoltageRangePortMask = LOWEST_VOLTAGE_RANGE_PORT_MASK_DUMMY; // Dummy in order to be able to shift for higher attenuator factors
                DDRC = OTHER_OUTPUTS_PIN_MASK; // Set bit for A1/PC1 to A3/PC3
                tInputVoltageRaw = readADCChannel();
            }
        } else {
            /*
             * Here we have factor 4 between attenuator ranges
             * Voltage range 3 -> 3; (x.x V) -> (17.6 V)
             * Voltage range 2 -> 1; (17.6 V) -> (4.4 V)
             */
            while (tInputVoltageRaw < (((0x3F0L * ATTENUATION_FACTOR_VOLTAGE_RANGE_1) / ATTENUATION_FACTOR_VOLTAGE_RANGE_2) - 0x10)
                    && sTesterInfo.VoltageRange > 1) {
                sTesterInfo.VoltageRangePortMask >>= 1; // Shift to next attenuator pin
                DDRC = sTesterInfo.VoltageRangePortMask | OTHER_OUTPUTS_PIN_MASK; // Set bit for A1/PC1 to A3/PC3
                sTesterInfo.VoltageRange--;
                // no wait, since last reading was same channel, same reference
                tInputVoltageRaw = readADCChannel();
#if defined(LOCAL_DEBUG)
                if (!sOnlyPlotterOutput) {
                    Serial.print(F("Range--="));
                    Serial.print(sTesterInfo.VoltageRange);
                    Serial.print(F(", VoltageRaw="));
                    Serial.println(tInputVoltageRaw);
                }
#endif
            }
            if (sTesterInfo.VoltageRange != 1) {
                tCheckAgainForRangeDecrease = false; // no need to check for underflow for range 1
            }
        }
    }
#if defined(LOCAL_TRACE)
    Serial.print(F("VoltageRaw="));
    Serial.print(tInputVoltageRaw);
    Serial.print(F(" Range="));
    Serial.print(sTesterInfo.VoltageRange);
    Serial.print(F(" Mask=0x"));
    Serial.print(sTesterInfo.VoltageRangePortMask, HEX);
    Serial.println();
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
    sBatteryOrLoggerInfo.Voltages.Logger.MinimumMillivolt = UINT16_MAX;
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
    sLogger1MinuteAccumulator.RawCurrentAccumulator += sLogger1SecondAccumulator.RawCurrentAccumulator;
    sLogger1MinuteAccumulator.RawSampleCount += LOGGER_SAMPLE_FREQUENCY_HZ;

    /*
     * Compute Milliampere and avoid overflow
     */
    sBatteryOrLoggerInfo.Milliampere = ((((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) / 1023)
            * sLogger1SecondAccumulator.RawCurrentAccumulator)
            / (LOGGER_SHUNT_RESISTOR_MILLIOHM * LOGGER_SAMPLE_FREQUENCY_HZ * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT)); // / 3,076,000 for 200 mOhm
    if (sBatteryOrLoggerInfo.Milliampere >= NO_LOGGER_MILLAMPERE) {
        sTesterInfo.inLoggerModeAndFlags |= LOGGER_EXTERNAL_CURRENT_DETECTED; // set it, if once detected here
    }
    /*
     * Compute voltage and avoid overflow
     * >> 8 and * 4 in divisor are a fast and short way to divide by 1024
     * AverageMillivolt is a union with NoLoadMillivolt and used for display
     */
    sBatteryOrLoggerInfo.Voltages.Logger.AverageMillivolt = ((sLogger1SecondAccumulator.RawVoltageAccumulator >> 8)
            / ((LOGGER_SAMPLE_FREQUENCY_HZ * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT * 4L) / ADC_INTERNAL_REFERENCE_MILLIVOLT));
    if (sBatteryOrLoggerInfo.Voltages.Logger.AverageMillivolt > NO_BATTERY_MILLIVOLT) {
        sTesterInfo.inLoggerModeAndFlags |= LOGGER_EXTERNAL_VOLTAGE_DETECTED; // set it, if once detected here
    }
    sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT)
            * (uint32_t) sLogger1SecondAccumulator.MaximumRawVoltage) / 1024);
    sBatteryOrLoggerInfo.Voltages.Logger.MinimumMillivolt = (((ADC_INTERNAL_REFERENCE_MILLIVOLT)
            * (uint32_t) sLogger1SecondAccumulator.MinimumRawVoltage) / 1024);

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
    Serial.println(F(" mV"));
#endif

    clearLogger1SecondAccumulator();
}

void getLogger1MinuteValues() {
// avoid overflow
    sBatteryOrLoggerInfo.Milliampere = ((((ADC_INTERNAL_REFERENCE_MILLIVOLT * 1000L) / 1023)
            * (sLogger1MinuteAccumulator.RawCurrentAccumulator / LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT))
            / ((uint32_t) sLogger1MinuteAccumulator.RawSampleCount * LOGGER_SHUNT_RESISTOR_MILLIOHM)); // / 240.000

    /*
     * Compute voltage and avoid overflow
     * Instead of (sLogger1MinuteRawVoltageAccumulator8ShiftRight << 8) / 1024 which gives overflow
     * we do (sLogger1MinuteRawVoltageAccumulator8ShiftRight >> 8) and divide the divisor by 2^6 (64)
     */
    sBatteryOrLoggerInfo.Voltages.Logger.AverageMillivolt = (ADC_INTERNAL_REFERENCE_MILLIVOLT
            * (sLogger1MinuteAccumulator.RawVoltageAccumulator8ShiftRight >> 8))
            / (((uint32_t) sLogger1MinuteAccumulator.RawSampleCount * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT) / 64);

#if defined(LOCAL_TRACE)
Serial.println();
Serial.print(F("cnt="));
Serial.print(sLogger1MinuteAccumulator.RawSampleCount);
Serial.print(F(" Iacc="));
Serial.print(sLogger1MinuteAccumulator.RawCurrentAccumulator);
Serial.print(' ');
Serial.print(sBatteryOrLoggerInfo.Milliampere);
Serial.print(F(" mA, Uacc="));
Serial.print(sLogger1MinuteAccumulator.RawVoltageAccumulator8ShiftRight);
Serial.print(' ');
Serial.print(sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt);
Serial.println(F(" mV"));
#endif

    clearLogger1MinuteAccumulator();
}

/*
 * First read 769 current values in 20 ms, then read 769 voltage values in 20 ms
 * complete reading takes 40.5 ms. For voltage > 4.4V it takes 48.5 ms
 */
void getPeriodicAccumulatingLoggerValues() {
    auto tTimeDifference = millis() - sTesterInfo.LastMillisOfSample; // compute with full 32 bit -
    if ((unsigned int) tTimeDifference >= LOGGER_SAMPLE_PERIOD_MILLIS) { // - and compare only 16 bit values
        sTesterInfo.LastMillisOfLoggerSample = millis();
        digitalWrite(LED_BUILTIN, HIGH);

#if defined(IN_WOKWI)
        /*
         * Fill accumulator only one time
         */
        if (sLogger1SecondAccumulator.RawCurrentAccumulator == 0) {
            uint32_t tInputCurrentRaw = analogRead(A7); // use VCC reference
            sLogger1SecondAccumulator.RawCurrentAccumulator += tInputCurrentRaw * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT
                    * LOGGER_SAMPLE_FREQUENCY_HZ;
        }
        if (sLogger1SecondAccumulator.RawVoltageAccumulator == 0) {
            uint32_t tInputVoltageRaw = analogRead(A0); // use VCC reference
            tInputVoltageRaw *= 16; // simulate "Normalize to 1.1 V"
            sLogger1SecondAccumulator.RawVoltageAccumulator += tInputVoltageRaw * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT
                    * LOGGER_SAMPLE_FREQUENCY_HZ;
            sLogger1SecondAccumulator.RawSampleCount += LOGGER_SAMPLE_FREQUENCY_HZ;
        }
        digitalWrite(LED_BUILTIN, LOW);
        return;
#endif

// switch channel and reference
#if defined(LOCAL_TRACE)
//        uint8_t tOldADMUX =
#endif
        setADCChannelForNextConversionAndWaitUsingInternalReference(ADC_CHANNEL_LOGGER_CURRENT);
        /*
         * Read 769 current values in 20ms.
         * maximum value of readADCChannelWithReferenceAndPrescalerMultiSamples(...769) is 800 000
         * So we can have 5 k of it in a 32 bit integer
         * LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT = 769 for ADC_PRESCALE32 / 26us and 20 ms
         */
        uint32_t tRawCurrentValue = readADCChannelMultiSamples(ADC_PRESCALE32, LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT);
        digitalWrite(LED_BUILTIN, LOW);
        sLogger1SecondAccumulator.RawCurrentAccumulator += tRawCurrentValue;
#if defined(LOCAL_TRACE)
    if (!sOnlyPlotterOutput) {
//            Serial.print(F("OldADMUX=0x"));
//            Serial.print(tOldADMUX, HEX);
        Serial.print(F(" AVGRawCurrent="));
        Serial.println(tRawCurrentValue / LOGGER_NUMBER_OF_SAMPLES_PER_MEASUREMENT);
    }
#endif

        /*
         * Read 769 voltage values in 20 ms
         * tRawVoltageValue is normalized for a value of 1023 at 4.4 V.
         */
        getVoltageRaw(); // this handles automatic range setting

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
            tRawVoltageValue += tInputRawVoltage;
            /*
             * Find minimum and maximum
             */
            if (tInputMinimumRawVoltage > tInputRawVoltage) {
                tInputMinimumRawVoltage = tInputRawVoltage;
            }
            if (tInputMaximumRawVoltage < tInputRawVoltage) {
                tInputMaximumRawVoltage = tInputRawVoltage;
            }
            i++; // To enable re-initialization of loop by i=0 above
        }
        ADCSRA &= ~_BV(ADATE); // Disable auto-triggering (free running mode)

        /*
         * Normalize to 1.1 V
         */
        auto tVoltageAttenuationFactor = getVoltageAttenuationFactor();
        tRawVoltageValue *= tVoltageAttenuationFactor;
        tInputMaximumRawVoltage *= tVoltageAttenuationFactor;
        tInputMinimumRawVoltage *= tVoltageAttenuationFactor;

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
    }
}

/*
 * Maximal current for a 0.2 ohm shunt resistor is 5.5 A, and resolution is 5.4 mA.
 * Maximal current for the 2 ohm battery shunt resistor is 550 mA, and resolution is 0.54 mA.
 */
void getCurrent(uint8_t aADCChannel, uint16_t aShuntResistorMilliohm) {
    setADCChannelForNextConversionAndWaitUsingInternalReference(aADCChannel);
    uint16_t tShuntVoltageRaw = readADCChannel();
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
    getVoltageMillivolt(); // get current battery load voltage (no load in case of stopped)

    if (sTesterInfo.MeasurementState == STATE_STOPPED) return; // thats all if stopped :-)

// Deactivate load and wait for voltage to settle
// During the no load period switch on the LED
    setLoad(NO_LOAD);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(sBatteryOrLoggerInfo.LoadSwitchSettleTimeMillis);
    getVoltageMillivolt(); // get current battery NoLoadMillivolt
// restore original load state
    setLoad(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].LoadType);
    digitalWrite(LED_BUILTIN, LOW);

    if (sBatteryOrLoggerInfo.Milliampere > 1) {
        if (sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt > sBatteryOrLoggerInfo.Voltages.Battery.LoadMillivolt) {
            /*
             * ESR computation
             */
            sBatteryOrLoggerInfo.ESRDeltaMillivolt = sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt
                    - sBatteryOrLoggerInfo.Voltages.Battery.LoadMillivolt;
        } else {
            sBatteryOrLoggerInfo.ESRDeltaMillivolt = 0; // avoid negative / unsigned overflow for ESR
        }
        /*
         * Compute sESRAverage
         * Shift history array to end and insert current value at [0]
         */
        uint32_t tESRMilliohm = (sBatteryOrLoggerInfo.ESRDeltaMillivolt * 1000L) / sBatteryOrLoggerInfo.Milliampere;
        if (tESRMilliohm > __UINT16_MAX__) {
            tESRMilliohm = __UINT16_MAX__; // indicate overflow
        }
        uint32_t tESRAverageAccumulator = tESRMilliohm;
        for (uint_fast8_t i = HISTORY_SIZE_FOR_ESR_AVERAGE; i > 0; --i) {
            // shift i-1 to i and add to average
#if defined(LOCAL_TRACE)
            Serial.print(sESRHistory[i - 1]);
            Serial.print('+');
#endif
            tESRAverageAccumulator += sESRHistory[i - 1];
            sESRHistory[i] = sESRHistory[i - 1];
        }

        // keep track of number of valid inputs
        if (sESRHistoryLength < HISTORY_SIZE_FOR_ESR_AVERAGE) {
            sESRHistoryLength++;
        }
        // insert current value in free space
        sESRHistory[0] = tESRMilliohm;

#if defined(LOCAL_TRACE)
    Serial.print(tESRMilliohm);
    Serial.print('=');
    Serial.print(sBatteryOrLoggerInfo.ESRDeltaMillivolt);
    Serial.print(F("*1000/"));
    Serial.print(sBatteryOrLoggerInfo.Milliampere);
    Serial.println();
#endif

        sBatteryOrLoggerInfo.ESRMilliohm = (tESRAverageAccumulator + (sESRHistoryLength / 2)) / sESRHistoryLength;

#if defined(LOCAL_TRACE)
    Serial.print(sESRHistory[0]);
    Serial.print('/');
    Serial.print(sESRHistoryLength);
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

void playUndervoltageAttentionTone() {
    tone(BUZZER_PIN, NOTE_C7, 40);
    delay(100);
    tone(BUZZER_PIN, NOTE_C7, 40);
    delay(100);
}

/*
 * Called exclusively from handlePeriodicStoringToEEPROM()
 * Check for switch off voltage or current reached -> end of measurement
 * Switch to state STATE_STOPPED if stop condition met
 */
void checkAndHandleStopConditionLCD() {
    bool tStopConditionIsMet = false;
    if (sTesterInfo.inLoggerModeAndFlags) {
        if (sTesterInfo.inLoggerModeAndFlags & LOGGER_EXTERNAL_CURRENT_DETECTED) {
            // use dirty hack to compute the cutoff current by shifting by cutoff level + 1
            if (sBatteryOrLoggerInfo.Milliampere
                    < (ChartStartValues.initialMilliampere >> (sBatteryOrLoggerInfo.CutoffLevel + 1))) {
                /*
                 * Switch off current condition for logger met
                 */
#if !defined(SUPPRESS_SERIAL_PRINT_STOP_CONDITION)
                if (!sOnlyPlotterOutput) {
                    Serial.println();
                    Serial.print(F("Switch off current percentage "));
                    Serial.print(100 >> (sBatteryOrLoggerInfo.CutoffLevel + 1));
                    Serial.print(F(" % mA of "));
                    Serial.print(ChartStartValues.initialMilliampere);
                    Serial.print(F(" mA reached, I="));
                    Serial.print(sBatteryOrLoggerInfo.Milliampere);
                    Serial.print(F(" mA, capacity="));
                    Serial.print(sBatteryOrLoggerInfo.CapacityMilliampereHour);
                    Serial.println(F(" mAh"));
                }
#endif
                tStopConditionIsMet = true;
            }
        }
        /*
         * Since we use AverageMillivolt here, a full storage period of low voltage is required to stop.
         * Disconnecting during a storage period leads to a lower AverageMillivolt, but this is usually not below the threshold.
         */
        if (sTesterInfo.inLoggerModeAndFlags & LOGGER_EXTERNAL_VOLTAGE_DETECTED) {
            if (sBatteryOrLoggerInfo.Voltages.Logger.AverageMillivolt < NO_BATTERY_MILLIVOLT) {
                /*
                 * Switch off voltage condition for logger met
                 */
#if !defined(SUPPRESS_SERIAL_PRINT_STOP_CONDITION)
                if (!sOnlyPlotterOutput) {
                    Serial.println();
                    Serial.print(sBatteryOrLoggerInfo.Voltages.Logger.AverageMillivolt);
                    Serial.println(F(" mV is lower than switch off voltage " STR(NO_BATTERY_MILLIVOLT) " mV "));
                }
#endif
                tStopConditionIsMet = true;
            }
        }

    } else {
        if (sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt < sBatteryOrLoggerInfo.CutoffVoltageMillivolt) {
            /*
             * Switch off voltage condition for battery met
             */
#if !defined(SUPPRESS_SERIAL_PRINT_STOP_CONDITION)
            if (!sOnlyPlotterOutput) {
                Serial.println();
                Serial.print(F("Switch off voltage "));
                Serial.print(sBatteryOrLoggerInfo.CutoffVoltageMillivolt);
                Serial.print(F(" mV reached, capacity="));
                Serial.print(sBatteryOrLoggerInfo.CapacityMilliampereHour);
                Serial.println(F(" mAh"));
            }
#endif
            tStopConditionIsMet = true;
        }
    }
    if (tStopConditionIsMet) {
        sTesterInfo.MeasurementWasFinishedByEndCondition = true; // To show "Finished" instead of "Stopped" on button

        switchToStateStopped('-'); // calls storeCapacityAndCutoffLevelToEEPROM_LCD()
        sLastDiplayedValues.Milliampere = 0; // Disable display of stop current for logger if current is still 0

#if defined(USE_LCD)
        _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS); // show "stopped"
        myLCD.setCursor(7, 0);
        myLCD.print(F(" Finished"));
        _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS); // show "Finished" - at least for logger, which overwrites it directly
#endif
// Play short melody
        playEndTone();
    }
}

void setBatteryTypeIndex(uint8_t aBatteryTypeIndex) {
    sBatteryOrLoggerInfo.BatteryTypeIndex = aBatteryTypeIndex;
    sBatteryOrLoggerInfo.LoadSwitchSettleTimeMillis = BatteryTypeInfoArray[aBatteryTypeIndex].LoadSwitchSettleTimeMillis;
    setCutoffAndCutoffVoltage(sBatteryOrLoggerInfo.CutoffLevel); // set CutoffVoltageMillivolt, which depends on BatteryTypeIndex
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    setCutoffHighLowZeroButtonTextAndDrawButton();
#endif
#if defined(LOCAL_TRACE)
Serial.print(F(" Battery index="));
Serial.print(aBatteryTypeIndex);
Serial.print(F(" SwitchOffVoltageMillivolt="));
Serial.print(sBatteryOrLoggerInfo.CutoffVoltageMillivolt);
Serial.print(F(" LoadSwitchSettleTimeMillis="));
Serial.println(sBatteryOrLoggerInfo.LoadSwitchSettleTimeMillis);
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
 * For logger it measures the current and voltage for detection
 * @return true, if battery or logger voltage and current detected
 */
bool detectBatteryOrLoggerVoltageOrCurrentLCD_BD() {
    getVoltageMillivolt(); // sets sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt

    bool tBatteryOrCurrentOrVoltageWasDetected = false;
    if (sTesterInfo.inLoggerModeAndFlags) {
        /*
         * Logger here
         */
        getCurrent(ADC_CHANNEL_LOGGER_CURRENT, LOGGER_SHUNT_RESISTOR_MILLIOHM);
        if (sBatteryOrLoggerInfo.Milliampere >= NO_LOGGER_MILLAMPERE) {
            sTesterInfo.inLoggerModeAndFlags |= LOGGER_EXTERNAL_CURRENT_DETECTED;
            tBatteryOrCurrentOrVoltageWasDetected = true;
        }
        if (sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt > NO_BATTERY_MILLIVOLT) {
            sTesterInfo.inLoggerModeAndFlags |= LOGGER_EXTERNAL_VOLTAGE_DETECTED;
            tBatteryOrCurrentOrVoltageWasDetected = true;
        }
        if (tBatteryOrCurrentOrVoltageWasDetected) {
#if !defined(SUPPRESS_SERIAL_PRINT_FOUND_REMOVING_CONDITION)
            if (!sOnlyPlotterOutput) {
                Serial.print(F("Found U="));
                Serial.print(sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt);
                Serial.print(F(" mV, I="));
                Serial.print(sBatteryOrLoggerInfo.Milliampere);
                Serial.println(F(" mA"));
            }
#endif
        } else {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
            if (!sOnlyPlotterOutput) { // do not disturb Arduino plotter with BD output
                BlueDisplay1.writeString(F("\rNo U or I       "));
            }
#endif
#if defined(USE_LCD)
            myLCD.setCursor(7, 0);
            myLCD.print(F("No U or I"));
#endif
        }
    } else {

        /*
         * Battery type detection here
         */
        sBatteryOrLoggerInfo.Milliampere = 0; // Avoid the display of the old current value
        if (setBatteryTypeIndexFromVoltage(sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt)) {
            /*
             * BatteryTypeIndex changed here
             */
            if (sBatteryOrLoggerInfo.BatteryTypeIndex == TYPE_INDEX_NO_BATTERY) {
#if defined(SUPPORT_BLUEDISPLAY_CHART)
                if (!sOnlyPlotterOutput) { // do not disturb Arduino plotter with BD output
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
                    // The same info is printed below by writeString(F("\rFound ")); ...
                    Serial.print(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].TypeName);
                    Serial.println(F(" found"));
                }
#endif
#if defined(SUPPORT_BLUEDISPLAY_CHART)
                BlueDisplay1.writeString(F("\rFound "));
                BlueDisplay1.writeString(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].TypeName);
#endif
#if defined(USE_LCD)
                // The current battery voltage is displayed, so clear "No batt." message selectively
                myLCD.setCursor(7, 0);
                myLCD.print(F("         "));

                myLCD.setCursor(0, 1);
                myLCD.print(F("Found "));
                myLCD.print(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].TypeName);
                _delay(LCD_MESSAGE_PERSIST_TIME_MILLIS);
                LCDClearLine(&myLCD, 1);
#endif
                tBatteryOrCurrentOrVoltageWasDetected = true;
            }
        }
    }
    return tBatteryOrCurrentOrVoltageWasDetected;
}

/*
 * Check for no current or voltage below NO_BATTERY_MILLIVOLT
 * @return true if removed
 */
void detectVoltageOrCurrentRemoved() {
// check only if battery was inserted before
    if (((sBatteryOrLoggerInfo.Milliampere == 0 /* no current */
    && (!sTesterInfo.inLoggerModeAndFlags || (sTesterInfo.inLoggerModeAndFlags & LOGGER_EXTERNAL_CURRENT_DETECTED)))
            || (sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt < NO_BATTERY_MILLIVOLT /* no voltage */
            && (!sTesterInfo.inLoggerModeAndFlags || (sTesterInfo.inLoggerModeAndFlags & LOGGER_EXTERNAL_VOLTAGE_DETECTED))))) {
        if (sTesterInfo.NumbersOfInitialSamplesToGo == NUMBER_OF_INITIAL_SAMPLES) {
            // no current after voltage detection, discharge resistor not connected?
#if defined(SUPPORT_BLUEDISPLAY_CHART)
            if (!sOnlyPlotterOutput) { // do not disturb Arduino plotter with BD output
                BlueDisplay1.writeString(F("\rNo current !    "));
            }
#endif
#if defined(USE_LCD)
            myLCD.setCursor(0, 1);
            myLCD.print(F("No current ! "));
            _delay(2 * LCD_MESSAGE_PERSIST_TIME_MILLIS);
#endif
            if (!sOnlyPlotterOutput) {
                Serial.println(F("No current, discharge resistor not connected?"));
            }
        } else {
#if !defined(SUPPRESS_SERIAL_PRINT_FOUND_REMOVING_CONDITION) // around 100 bytes program space
            if (!sOnlyPlotterOutput) {
                Serial.print(F("Battery or voltage removing detected. U="));
                Serial.print(sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt);
                Serial.print(F(" mV I="));
                Serial.print(sBatteryOrLoggerInfo.Milliampere);
                Serial.println(F(" mA"));
            }
#endif
        }
        switchToStateWaitingForBatteryOrVoltage(); // switch back to start and do not overwrite already displayed values
    }
}

void printVoltageNoLoadMillivoltWithTrailingSpaceLCD_BD() {
    uint16_t tVoltageNoLoadMillivolt = sBatteryOrLoggerInfo.Voltages.Battery.NoLoadMillivolt; // saves 12 bytes programming space

#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        printMillisValueAsFloat(tVoltageNoLoadMillivolt);
        Serial.print(F(" V"));
    }
#endif

    if (abs(
            (int16_t )sLastDiplayedValues.VoltageNoLoadMillivolt
            - (int16_t )tVoltageNoLoadMillivolt) > VOLTAGE_DISPLAY_HYSTERESIS_MILLIVOLT) {
        sLastDiplayedValues.VoltageNoLoadMillivolt = tVoltageNoLoadMillivolt;

#if defined(SUPPORT_BLUEDISPLAY_CHART)
        if (BlueDisplay1.isConnectionEstablished()) {
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
        LCDResetCursor(&myLCD);
        LCDPrintAsFloatAs5CharacterString_2_3_Decimals(&myLCD, tVoltageNoLoadMillivolt);
        myLCD.print('V'); // no trailing space because of counters which can be more than 1000
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

    if (uintDifferenceAbs(sLastDiplayedValues.ESRMilliohm, tMilliohm) > ESR_DISPLAY_HYSTERESIS_MILLIOHM) {
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
        if (sTesterInfo.inLoggerModeAndFlags) {
            LCDPrintAsFloatAs5CharacterString_2_3_Decimals(&myLCD, sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt);
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
    myLCD.setCursor(6, 0); // for numbers up to 9999
// snprintf_P requires 46 bytes more program space :-(
//    char tString[5];
//    snprintf_P(tString, sizeof(tString), PSTR("%4"), aNumberToPrint);
//    myLCD.print(tString);

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
 * @return true if printed
 */
bool printMilliampere4DigitsLCD_BD() {
    bool tReturnValue = false;
    if (abs(
            (int16_t )sLastDiplayedValues.Milliampere
            - (int16_t )sBatteryOrLoggerInfo.Milliampere) > CURRENT_DISPLAY_HYSTERESIS_MILLIAMPERE) {
        sLastDiplayedValues.Milliampere = sBatteryOrLoggerInfo.Milliampere;

#if defined(SUPPORT_BLUEDISPLAY_CHART) || defined(USE_LCD)
        char tString[10];
#endif
#if defined(SUPPORT_BLUEDISPLAY_CHART)
        if ((!sTesterInfo.inLoggerModeAndFlags || (sTesterInfo.inLoggerModeAndFlags & LOGGER_EXTERNAL_CURRENT_DETECTED))
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
        tReturnValue = true;
    }
#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput && (tReturnValue || sTesterInfo.MeasurementState != STATE_STOPPED)) {
        // is called in state stop periodically to determine if Milliampere changed
        Serial.print(sBatteryOrLoggerInfo.Milliampere);
        Serial.print(F(" mA "));
        if (!sTesterInfo.inLoggerModeAndFlags) {
            Serial.print(F("at "));
            printMillisValueAsFloat(sCurrentLoadResistorHistory[0]);
            Serial.print(F(" ohm, "));
        }
    }
#endif
    return tReturnValue;
}

/*
 * To force display of values on LCD and BlueDisplay
 */
void forceDisplayOfCurrentValues() {
    sLastDiplayedValues.VoltageNoLoadMillivolt = __INT16_MAX__;
    sLastDiplayedValues.Milliampere = __INT16_MAX__;
    sLastDiplayedValues.ESRMilliohm = __INT16_MAX__;
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    clearValueArea();
#endif
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
 * "4.030V 18  329mA" NoLoadMillivolt, NumbersOfInitialSamplesToGo, Milliampere
 * "0.061o l  0.128V" ESRMilliohm (from sESRHistory[0]), CutoffLevelCharacter, ESRDeltaMillivolt
 *
 * STATE_SAMPLE_AND_STORE_TO_EEPROM:
 * "4.030 V,  329 mA at 11.949 ohm, ESR=0.329 ohm, capacity=1200 mAh
 *  0   4   8   C  F
 * "4.030V 312 329mA" NoLoadMillivolt, DeltaArrayIndex + 1, Milliampere
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
        /*************************************************************************************
         * First row only for state STATE_INITIAL_SAMPLES or STATE_SAMPLE_AND_STORE_TO_EEPROM
         *************************************************************************************/

        /*
         * First check and print counter
         * Use last displayed value for comparison, it is almost (+/-1) the actual voltage and it is the value the user observes!
         */
        if (tMeasurementState == STATE_INITIAL_SAMPLES) {
            /*
             * Print down counter for STATE_INITIAL_SAMPLES
             * Count down only if we are in logger mode or we do not have a rapid voltage decrease (> 6 mV per sample)
             */
            if (sTesterInfo.inLoggerModeAndFlags
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
                    (ValuesForDeltaStorage.DeltaArrayIndex + 1)
                            * (ChartStartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE));
#endif
        } // END of print counter handling

        /*
         * Print no load voltage / average voltage here
         */
        printVoltageNoLoadMillivoltWithTrailingSpaceLCD_BD();
#if !defined(SUPPRESS_SERIAL_PRINT)
        if (!sOnlyPlotterOutput) {
            // 4.094 V, 334 mA at 11.949 ohm, ESR=0.334 ohm, capacity=3501 mAh
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
        if (sTesterInfo.inLoggerModeAndFlags) {
#if !defined(SUPPRESS_SERIAL_PRINT)
            if (!sOnlyPlotterOutput && sTesterInfo.MeasurementState != STATE_SETUP_AND_READ_EEPROM) {
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
        if (sTesterInfo.inLoggerModeAndFlags) {
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
            if (!sTesterInfo.inLoggerModeAndFlags) {
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
                LCDPrintAsFloatAs5CharacterString_2_3_Decimals(&myLCD, tESRDeltaMillivolt);
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
        ChartStartValues.initialMillivolt = aVoltageNoLoadMillivolt;
        ChartStartValues.initialMilliampere = aMilliampere;
        ChartStartValues.initialDischargingMilliohm = aMilliohm;
        ChartStartValues.LoadResistorMilliohm = sCurrentLoadResistorAverage;
        ChartStartValues.CutoffLevel = sBatteryOrLoggerInfo.CutoffLevel;
        ChartStartValues.BatteryTypeIndex = sBatteryOrLoggerInfo.BatteryTypeIndex;
        ChartStartValues.inLoggerModeAndFlags = sTesterInfo.inLoggerModeAndFlags & LOGGER_MODE_MASK;

        // sBatteryOrLoggerInfo.CapacityMilliampereHour is set to 0 in handleEndOfStateInitialSamples()
        ChartStartValues.CapacityMilliampereHour = 0; // Overwrite old capacity value of last EEPROM data set. Capacity is also written at the end
        eeprom_update_block(&ChartStartValues, &EEPROMStartValues, sizeof(EEPROMStartValues));
#if defined(SUPPORT_BLUEDISPLAY_CHART)
        redrawDisplay();
#endif

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
        forceDisplayOfCurrentValues();

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
            ValuesForDeltaStorage.lastStoredVoltageNoLoadMillivolt = ChartStartValues.initialMillivolt;
            ValuesForDeltaStorage.lastStoredMilliampere = ChartStartValues.initialMilliampere;
            ValuesForDeltaStorage.lastStoredMilliohm = ChartStartValues.initialDischargingMilliohm;
            ValuesForDeltaStorage.DeltaArrayIndex = 0;

            uint16_t tVoltageMillivolt = ChartStartValues.initialMillivolt;
            uint16_t tMilliampere = ChartStartValues.initialMilliampere;
            uint16_t tMilliohm = ChartStartValues.initialDischargingMilliohm;

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
                /*
                 * recursive call to store resulting value
                 */
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
            ChartStartValues.NumberOfSecondsPerStorage *= 2;
            eeprom_update_byte(&EEPROMStartValues.NumberOfSecondsPerStorage, ChartStartValues.NumberOfSecondsPerStorage); // store value in EEPROM
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
    LCDResetCursor(&myLCD);
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
 * no println() at the end, to enable appending of text like " - Capacity on top of standard value=..."
 * @param aChartValueArrayIndex  The index of sChartValueArray where the current processed EEPROM value is written to
 * @param aIsLastElement    If true, print BlueDisplay chart or Arduino plotter caption
 */
void printEEPROMChartAndPlotterGraph(uint16_t aMillivoltToPrint, uint16_t aMilliampereToPrint, uint16_t aMilliohmToPrint,
        uint16_t aChartValueArrayIndex, bool aIsLastElement) {
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
    printMillisValueAsFloat(ChartStartValues.initialMillivolt);
    Serial.print(F("V->"));
    printMillisValueAsFloat(aMillivoltToPrint);
    Serial.print(F("V__Current="));
    Serial.print(ChartStartValues.initialMilliampere);
    Serial.print(F("mA->"));
    Serial.print(aMilliampereToPrint);
    Serial.print(F("mA__ESR="));
    Serial.print(ChartStartValues.initialDischargingMilliohm);
    Serial.print(F("mOhm->"));
    Serial.print(aMilliohmToPrint);
    Serial.print(F("mOhm___LoadResistor="));
    printMillisValueAsFloat(ChartStartValues.LoadResistorMilliohm);
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
Serial.print(aChartValueArrayIndex);
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
            sChartValueArray[aChartValueArrayIndex] = (aMillivoltToPrint - sCompressionOffsetMillivolt)
                    / sVoltageChartCompressionFactor;
        } else if (sChartReadValueArrayType == TYPE_ESR) {
            sChartValueArray[aChartValueArrayIndex] = (aMilliohmToPrint * 10) / sCompressionFactorTimes10;
        } else {
            // TYPE_CURRENT
            sChartValueArray[aChartValueArrayIndex] = (aMilliampereToPrint * 10) / sCompressionFactorTimes10;
        }

        if (aIsLastElement) {
            /*
             * Here the array is filled up, so draw chart at BlueDisplay Host
             */
#    if defined(LOCAL_DEBUG)
        Serial.println();
        Serial.print(F("ChartReadValueArrayType="));
        Serial.print(sChartReadValueArrayType);
        Serial.print(F(" sCompressionFactor*10="));
        Serial.print(sCompressionFactorTimes10);
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
                int16_t tXScaleFactor = VoltageChart.computeXLabelAndXDataScaleFactor(aChartValueArrayIndex);
#    if defined(LOCAL_TRACE)
            Serial.println();
            Serial.print(F("ChartValueArrayIndex="));  // Current length of chart
            Serial.print(aChartValueArrayIndex);
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
                        (ChartStartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE) * tChartMinutesPerLabelUncompressed); // Starts with CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED (30) for 1 minute sampling
#else
            VoltageChart.setXLabelBaseIncrementValue(
                    (ChartStartValues.NumberOfSecondsPerStorage * tChartMinutesPerLabelUncompressed) / SECONDS_IN_ONE_MINUTE); // Starts with CHART_MINUTES_PER_X_LABEL_UNCOMPRESSED (30) for 1 minute sampling
#endif

                clearAndDrawChart(); // now the axes parameter are set, draw axes and grid and caption
            } else if (sChartReadValueArrayType == TYPE_ESR) {
                sLastChartData.ESRMilliohm = aMilliohmToPrint;
            } else {
                // TYPE_CURRENT
                sLastChartData.Milliampere = aMilliampereToPrint;
            }
            /*
             * Draw the graph
             */
            VoltageChart.drawChartDataWithYOffset(sChartValueArray, aChartValueArrayIndex + 1, CHART_MODE_LINE);
        }
    } else
#  endif // defined(SUPPORT_BLUEDISPLAY_CHART)

    { // new test is shorter than else!
        if (aIsLastElement) {
// Print updated plotter caption
            Serial.print(F("Voltage="));
            printMillisValueAsFloat(ChartStartValues.initialMillivolt);
            Serial.print(F("V->"));
            if (sTesterInfo.inLoggerModeAndFlags) {
                printMillisValueAsFloat(sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt);
                Serial.print(F("V->"));
                printMillisValueAsFloat(sBatteryOrLoggerInfo.Voltages.Logger.MinimumMillivolt);
            } else {
                printMillisValueAsFloat(aMillivoltToPrint);
            }
            Serial.print(F("V:"));
            Serial.print(aMillivoltToPrint); // after ":" we have value for plotter
            Serial.print(F(" Current="));
            Serial.print(ChartStartValues.initialMilliampere);
            Serial.print(F("mA->"));
            Serial.print(aMilliampereToPrint);
            Serial.print(F("mA:"));
            Serial.print(aMilliampereToPrint); // after ":" we have value for plotter
            if (ChartStartValues.initialDischargingMilliohm > 0) {
                Serial.print(F(" ESR="));
                Serial.print(ChartStartValues.initialDischargingMilliohm);
                Serial.print(F("mOhm->"));
                Serial.print(aMilliohmToPrint);
                Serial.print(F("mOhm:"));
                Serial.print(aMilliohmToPrint);
                Serial.print(F(" LoadResistor="));
                printMillisValueAsFloat(ChartStartValues.LoadResistorMilliohm);
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
                    * (ChartStartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE);
            Serial.print('=');
            Serial.print(tDurationMinutes / MINUTES_IN_ONE_HOUR);
            Serial.print(F("h_"));
            Serial.print(tDurationMinutes % MINUTES_IN_ONE_HOUR);
            Serial.print(F("min Sample_time="));
            Serial.print(ChartStartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE);
            Serial.print(F("min Cutoff="));
            Serial.print(sBatteryOrLoggerInfo.CutoffVoltageMillivolt);
            Serial.print(F("mV"));
            if (ChartStartValues.inLoggerModeAndFlags) {
                Serial.print(F("_Logger"));
            }

        } else {
            Serial.print(aMillivoltToPrint);
            Serial.print(' ');
            Serial.print(aMilliampereToPrint);
            if (ChartStartValues.initialDischargingMilliohm > 0) {
                Serial.print(' ');
                Serial.print(aMilliohmToPrint);
            }
        }
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
 * @param aInitializeValuesForDisplayAndAppend  - if true (called at setup()), store initial data and if SUPPORT_BLUEDISPLAY_CHART, do not print.
 */
void readAndProcessEEPROMData(bool aInitializeValuesForDisplayAndAppend) {
    EEPROMData tEEPROMData;
    /*
     * First copy EEPROM start values to RAM
     */
    eeprom_read_block(&ChartStartValues, &EEPROMStartValues, sizeof(EEPROMStartValues));

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
    int tFirstNonWrittenIndex = tLastWrittenIndex + 1;

#if defined(LOCAL_TRACE)
Serial.print(F("tLastWrittenIndex="));
Serial.print(tLastWrittenIndex);
Serial.print(F(" &tLastWrittenIndex=0x"));
Serial.println((uint16_t) &EEPROMDataArray[tLastWrittenIndex], HEX);
// dump 2 lines containing the 3 byte data
dumpEEPROM((uint8_t*) ((uint16_t) &EEPROMDataArray[tLastWrittenIndex] & 0xFFF0), 2);
#endif

    uint16_t tVoltageMillivolt = ChartStartValues.initialMillivolt;

#if defined(SUPPORT_BLUEDISPLAY_CHART)
    /*
     * Initialize values for chart scaling to be stored after the loop in ValuesForChartScaling
     * Use start values, because they are not handled in loop.
     */
    struct ValuesForChartScaling tTemporaryValuesForChartScaling;
    tTemporaryValuesForChartScaling.minVoltageNoLoadMillivolt = ChartStartValues.initialMillivolt;
    tTemporaryValuesForChartScaling.maxVoltageNoLoadMillivolt = ChartStartValues.initialMillivolt;
    tTemporaryValuesForChartScaling.maxMilliampere = ChartStartValues.initialMilliampere;
    tTemporaryValuesForChartScaling.maxMilliohm = ChartStartValues.initialDischargingMilliohm;
#endif

    if (aInitializeValuesForDisplayAndAppend) {
        /*
         * Set values required for append from start values already read.
         * Is only called once at setup()!
         */
        ValuesForDeltaStorage.DeltaArrayIndex = tFirstNonWrittenIndex; // for append
//        sBatteryOrLoggerInfo.Voltages.Logger.MaximumMillivolt = tVoltageMillivolt; // ??? why???
        sBatteryOrLoggerInfo.CapacityMilliampereHour = ChartStartValues.CapacityMilliampereHour; // may be corrected after reading all data
        setBatteryTypeIndexFromVoltage(tVoltageMillivolt); // sets sBatteryOrLoggerInfo.BatteryTypeIndex and LoadSwitchSettleTimeMillis
        setCutoffAndCutoffVoltage(ChartStartValues.CutoffLevel);
    }

#if defined(SUPPORT_BLUEDISPLAY_CHART)
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
    auto tNominalFullVoltageMillivolt = BatteryTypeInfoArray[ChartStartValues.BatteryTypeIndex].NominalFullVoltageMillivolt;
    if (tVoltageMillivolt >= tNominalFullVoltageMillivolt) {
        tCapacityMilliampereHourStandardValueState = CAPACITY_WAITING_FOR_NOMINAL_FULL_VOLTAGE;
    } else {
        tCapacityMilliampereHourStandardValueState = CAPACITY_STARTED;
    }
    uint32_t tCapacityAccumulatorUntilNominalFullVoltageValue = 0; // Used to compute standard capacity later

    uint16_t tMilliampere = ChartStartValues.initialMilliampere;
    uint32_t tCapacityAccumulator = tMilliampere;
    uint16_t tMilliohm = ChartStartValues.initialDischargingMilliohm;
    uint8_t tNumberOfEEPROMValuesPerHour = 3600 / ChartStartValues.NumberOfSecondsPerStorage;

#if !defined(SUPPRESS_SERIAL_PRINT)
    if (!sOnlyPlotterOutput) {
        Serial.println();
// We have always the first one as uncompressed value
        Serial.print(tFirstNonWrittenIndex + 1);
        Serial.print(F(" EEPROM data sets each "));
        Serial.print(ChartStartValues.NumberOfSecondsPerStorage);
        Serial.print(F(" seconds found"));
        if (!sTesterInfo.inLoggerModeAndFlags) {
            Serial.print(F(" for type="));
            Serial.print(BatteryTypeInfoArray[ChartStartValues.BatteryTypeIndex].TypeName);
        }
        Serial.print(F(", cut off level="));
        Serial.println(sBatteryOrLoggerInfo.CutoffLevelCharacter);
    }
#endif

    /****************************************************
     * Print the initial value with no caption to plotter
     ****************************************************/
#if defined(SUPPORT_BLUEDISPLAY_CHART)
    if (!aInitializeValuesForDisplayAndAppend) {  // Check is only required for SUPPORT_BLUEDISPLAY_CHART
        printEEPROMChartAndPlotterGraph(tVoltageMillivolt, tMilliampere, tMilliohm, 0, false); // store at ChartValueArrayIndex 0
        Serial.println();  // for first line from printEEPROMChartAndPlotterGraph
    }
#else
    printEEPROMChartAndPlotterGraph(tVoltageMillivolt, tMilliampere, tMilliohm, 0, false); // print always at non BD mode
    Serial.println(); // for first line from printEEPROMChartAndPlotterGraph
#endif

    /*******************************************
     * Loop to read and print all EEPROM values
     *******************************************/
    // tChartValueArrayIndex is not really required for non BD / only plotter mode, but it is used as parameter for printEEPROMChartAndPlotterGraph(), so keep it.
    uint16_t tChartValueArrayIndex = 1; // The index of sChartValueArray where the current processed EEPROM value is written to
    for (int i = 0; i < tFirstNonWrittenIndex; ++i) { // tFirstNonWrittenIndex can be from 0 to MAX_NUMBER_OF_SAMPLES

        eeprom_read_block(&tEEPROMData, &EEPROMDataArray[i], sizeof(tEEPROMData));
        tVoltageMillivolt += tEEPROMData.DeltaMillivolt;
        tMilliampere += tEEPROMData.DeltaMilliampere;
        tMilliohm += tEEPROMData.DeltaESRMilliohm;
        tCapacityAccumulator += tMilliampere; // putting this into printEEPROMChartAndPlotterGraph() increases program size

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

        if (!sTesterInfo.inLoggerModeAndFlags) {
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
                    && tVoltageMillivolt < BatteryTypeInfoArray[ChartStartValues.BatteryTypeIndex].CutoffVoltageMillivoltHigh) {
                tCapacityMilliampereHourStandardValueState = CAPACITY_COMPLETED;
                /*
                 * We reached CutoffVoltageMillivoltHigh, now we can compute standard capacity, if we started above nominal full voltage
                 * Standard capacity is always computed from EEPROM data, while capacity is taken from ChartStartValues.
                 * If we had power down before appending more than 12 percent discharge, this amount is missing in ChartStartValues.capacity.
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
        if (!aInitializeValuesForDisplayAndAppend) // Check is only required for SUPPORT_BLUEDISPLAY_CHART
#endif
        {
            printEEPROMChartAndPlotterGraph(tVoltageForPrint, tMilliampere, tMilliohm, tChartValueArrayIndex,
                    i == (tFirstNonWrittenIndex - 1));
            tChartValueArrayIndex++;

#if !defined(SUPPRESS_SERIAL_PRINT) // 270 bytes programming space
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
                    if (sTesterInfo.inLoggerModeAndFlags) {
                        Serial.print(F("capacity="));
                    } else {
                        Serial.print(F("capacity at high cut off="));
                    }
                    Serial.print(sTesterInfo.StandardCapacityMilliampereHour);
                    Serial.print(F(" mAh"));
                }
            }
#endif
            Serial.println(); // for line from printEEPROMChartAndPlotterGraph plus optional tPrintDelayed text above
        }
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
         * Print standard capacity between NominalFullVoltageMillivolt and CutoffVoltageMillivoltHigh,
         * if we have both values.
         */
        if (!sTesterInfo.inLoggerModeAndFlags && sTesterInfo.StandardCapacityMilliampereHour != 0
                && sTesterInfo.StandardCapacityMilliampereHour != tCurrentCapacityMilliampereHourComputed) {
            if (sTesterInfo.isStandardCapacityAvailable) {
                Serial.print(F("Standard "));
            }
            Serial.print(F("computed capacity between "));
            if (sTesterInfo.isStandardCapacityAvailable) {
                Serial.print(BatteryTypeInfoArray[sBatteryOrLoggerInfo.BatteryTypeIndex].NominalFullVoltageMillivolt);
            } else {
                Serial.print(ChartStartValues.initialMillivolt);
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
    /****************************************
     * APPEND by BDButton press is done here
     ****************************************/
    setBatteryTypeIndex(ChartStartValues.BatteryTypeIndex); // Restore original type index. This sets cutoff level accordingly
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
    if (millis() - sMillisOfLastRefreshOrChangeBrightness < TIMEOUT_FOR_BRIGHTNESS_MILLIS) {
        changeBrightness();
    }
    redrawDisplay();
    sMillisOfLastRefreshOrChangeBrightness = millis();
}

//void doRedrawChart(BDButton *aTheTouchedButton, int16_t aValue) {
//    (void) aTheTouchedButton;
//    (void) aValue;
//    readAndDrawEEPROMValues();
//}

void setBatteryLoggerButtonTextAndDrawButton() {
    if (sTesterInfo.inLoggerModeAndFlags) {
        TouchButtonBatteryLogger.setText(F("Logger"), true);
    } else {
        TouchButtonBatteryLogger.setText(F("Battery"), true);
    }
}

void doBatteryLogger(BDButton *aTheTouchedButton, int16_t aValue) {
    (void) aTheTouchedButton;
    (void) aValue;
    sTesterInfo.inLoggerModeAndFlags = !sTesterInfo.inLoggerModeAndFlags;
    sCurrentLoadResistorAverage = LOGGER_SHUNT_RESISTOR_MILLIOHM;
    redrawDisplay();
}

void setCutoffHighLowZeroButtonTextAndDrawButton() {
    if (BlueDisplay1.isConnectionEstablished()) {

        char tString[20];
        if (sTesterInfo.inLoggerModeAndFlags) {
            // CUTOFF_LEVEL_HIGH = 50%, LOW = 25% and ZERO = 12.5%
            if ((sTesterInfo.inLoggerModeAndFlags & LOGGER_EXTERNAL_CURRENT_DETECTED)
                    && sTesterInfo.MeasurementState == STATE_SAMPLE_AND_STORE_TO_EEPROM) {
                snprintf_P(tString, sizeof(tString), PSTR("Cutoff %u mA"),
                        ChartStartValues.initialMilliampere >> (sBatteryOrLoggerInfo.CutoffLevel + 1));
            } else {
                snprintf_P(tString, sizeof(tString), PSTR("Cutoff %2u%% I"), 100 >> (sBatteryOrLoggerInfo.CutoffLevel + 1));
            }
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
    forceDisplayOfCurrentValues();
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
// currently we have portrait orientation -> change to landscape, which is requested below by BD_FLAG_SCREEN_ORIENTATION_LOCK_SENSOR_LANDSCAPE
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

    tBDButtonPGMParameterStruct.aPositionX = BUTTONS_START_X; // Reset x position
//    TouchButtonAppend.init(BUTTONS_START_X + BUTTON_WIDTH + (BASE_TEXT_SIZE / 8), tBDButtonPGMParameterStruct.aPositionY,
//    BUTTON_WIDTH - BASE_TEXT_SIZE, BASE_TEXT_SIZE + BASE_TEXT_SIZE / 3, COLOR16_GREEN, "Append", 44, FLAG_BUTTON_DO_BEEP_ON_TOUCH,
//            sTesterInfo.inLoggerModeAndFlags, &doAppend);

// 58 bytes
    tBDButtonPGMParameterStruct.aWidthX = (2 * BUTTON_WIDTH) + (BASE_TEXT_SIZE / 4) - BASE_TEXT_SIZE; // size + distance of state and cutoff button
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

/*
 * Content and labels of chart is written in printEEPROMChartAndPlotterGraph()
 */
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
    if (!sTesterInfo.inLoggerModeAndFlags) {
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
 * It removes display of count
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
        if (tDeltaMillivoltPerGrid < (tMillivoltPerGrid * 2)) {
            tMillivoltPerGrid *= 2; // < 200, 2000 etc.
            break;
        }
        tMillivoltPerGrid *= 5;
        if (tDeltaMillivoltPerGrid < tMillivoltPerGrid) {
            break; // < 500, 5000 etc.
        }
        tMillivoltPerGrid *= 2;
    }

    sVoltageChartCompressionFactor = tMillivoltPerGrid / 20;
#if defined(LOCAL_TRACE)
    Serial.print(F("Max="));
    Serial.print(ValuesForChartScaling.maxVoltageNoLoadMillivolt);
    Serial.print(F("V, min="));
    Serial.print(ValuesForChartScaling.minVoltageNoLoadMillivolt);
    Serial.print(F(", mV/grid="));
    Serial.println(tMillivoltPerGrid);
#endif

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

// Parameter: YFactor = sCompressionFactorTimes10/1000 = 10/1000 = 0.01 = * 100 -> input is 100 for 1.0 (volt) and resolution is 10 mV
// Parameter: 4, 1 gives e.g. " 3.5" as label
    VoltageChart.initYLabel(sCompressionOffsetMillivolt / 1000.0, tMillivoltPerGrid / 1000.0,
            sVoltageChartCompressionFactor / 1000.0, 4, 1);

    VoltageChart.setDataColor(CHART_VOLTAGE_COLOR);
    sChartReadValueArrayType = TYPE_VOLTAGE; // process voltages
    readAndProcessEEPROMData(false);

    /*
     * Show ESR if we are just after boot and data stored was battery data
     * OR we are just not after boot and in battery mode
     */
    if ((sTesterInfo.MeasurementState == STATE_SETUP_AND_READ_EEPROM && !ChartStartValues.inLoggerModeAndFlags)
            || (sTesterInfo.MeasurementState != STATE_SETUP_AND_READ_EEPROM && !ChartStartValues.inLoggerModeAndFlags)) {

        _delay(HELPFUL_DELAY_BETWEEN_DRAWING_CHART_LINES_TO_STABILIZE_USB_CONNECTION);
        /*
         * Process ESR
         * Find right compression factor
         * Compression factor of 5 gives maximum of display of 700 mX, 10 -> 1400 mX.
         * To have full scale we can compare with sVoltageChartCompressionFactor * 140
         * We choose 120 to avoid that the maximum ESR chart value is higher than the maximum chart value of voltage.
         */
        if (ValuesForChartScaling.maxMilliohm > (sVoltageChartCompressionFactor * 1200L)) {
            sCompressionFactorTimes10 = sVoltageChartCompressionFactor * 1000; // E.g. for low battery voltage and high ESR
        } else if (ValuesForChartScaling.maxMilliohm > ((uint16_t) sVoltageChartCompressionFactor * 120)) {
            sCompressionFactorTimes10 = sVoltageChartCompressionFactor * 100;
        } else if ((ValuesForChartScaling.maxMilliohm > ((uint16_t) sVoltageChartCompressionFactor * 12))
                || sVoltageChartCompressionFactor < 10) {
            sCompressionFactorTimes10 = sVoltageChartCompressionFactor * 10;
        } else {
            sCompressionFactorTimes10 = sVoltageChartCompressionFactor;
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
    if (ValuesForChartScaling.maxMilliampere > ((uint16_t) sVoltageChartCompressionFactor * 120L)) {
        sCompressionFactorTimes10 = sVoltageChartCompressionFactor * 100;
    } else if ((ValuesForChartScaling.maxMilliampere > ((uint16_t) sVoltageChartCompressionFactor * 12))
            || sVoltageChartCompressionFactor < 10) {
        sCompressionFactorTimes10 = sVoltageChartCompressionFactor * 10;
    } else {
        sCompressionFactorTimes10 = sVoltageChartCompressionFactor;
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
    if (sTesterInfo.isStandardCapacityAvailable && ValuesForDeltaStorage.DeltaArrayIndex >= 0) {
// 70 bytes
        snprintf_P(tString, sizeof(tString), PSTR("%5u | %u mAh"), sBatteryOrLoggerInfo.CapacityMilliampereHour,
                sTesterInfo.StandardCapacityMilliampereHour);
    } else {
        snprintf_P(tString, sizeof(tString), PSTR("%5u mAh        "), sBatteryOrLoggerInfo.CapacityMilliampereHour);
    }
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, BlueDisplay1.getRequestedDisplayHeight() - ((sChartDataTextSize + 1) * 9),
            tString, sChartDataTextSize, sTextColor, sBackgroundColor);
}

/*
 * Print all chart related values at the lower right of screen display
 */
void printChartValues() {
    char tStringBuffer[30];
    uint_fast8_t tChartDataTextHeight = sChartDataTextSize + 1; // Quick hack to avoid deleting of top of character 'l' in Samples by capacity printing.
    uint16_t tYPosition = (BlueDisplay1.getRequestedDisplayHeight() - (tChartDataTextHeight * 10));

// Battery type
    const char *aBatteryTypePtr;
    if (ChartStartValues.inLoggerModeAndFlags) {
        aBatteryTypePtr = "Logger";
    } else {
        aBatteryTypePtr = BatteryTypeInfoArray[ChartStartValues.BatteryTypeIndex].TypeName;
    }
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X + (BASE_TEXT_WIDTH * 6) - 3, tYPosition, aBatteryTypePtr, sChartDataTextSize,
            sTextColor, sBackgroundColor);

// Capacity
    printCapacityValue();

// Samples use 5u to have the same spacing as mAh
    snprintf(tStringBuffer, sizeof(tStringBuffer), "%5u Samples %u mn", ValuesForDeltaStorage.DeltaArrayIndex + 1,
            ChartStartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE); // Samples + start sample
    tYPosition += 2 * tChartDataTextHeight;
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, sTextColor, sBackgroundColor);

// Duration + VCC
    sVCCVoltageMillivolt = getVCCVoltageMillivolt();
    uint16_t tDurationMinutes = ValuesForDeltaStorage.DeltaArrayIndex
            * (ChartStartValues.NumberOfSecondsPerStorage / SECONDS_IN_ONE_MINUTE);
    snprintf(tStringBuffer, sizeof(tStringBuffer), "%2u:%02u h       VCC", tDurationMinutes / MINUTES_IN_ONE_HOUR,
            tDurationMinutes % MINUTES_IN_ONE_HOUR);
    dtostrf(sVCCVoltageMillivolt / 1000.0, 5, 2, &tStringBuffer[8]);
    tStringBuffer[13] = ' '; // overwrite terminating null
    tYPosition += tChartDataTextHeight;
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, sTextColor, sBackgroundColor);

// Load and cutoff
    dtostrf(ChartStartValues.LoadResistorMilliohm / 1000.0, 5, 2, tStringBuffer);
    if (ChartStartValues.inLoggerModeAndFlags) {
        snprintf(&tStringBuffer[5], sizeof(tStringBuffer), " \x81 Shunt  "); // strcat(tStringBuffer, " \x81 Shunt  ") requires more program space
    } else {
        snprintf(&tStringBuffer[5], sizeof(tStringBuffer), " \x81 Load   "); // strcat(tStringBuffer, " \x81 Load   ") requires more program space
    }
    tStringBuffer[14] = sBatteryOrLoggerInfo.CutoffLevelCharacter;
    tYPosition += tChartDataTextHeight;
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, sTextColor, sBackgroundColor);

// Voltage
    dtostrf(ChartStartValues.initialMillivolt / 1000.0, 5, 2, tStringBuffer);
    snprintf(&tStringBuffer[5], sizeof(tStringBuffer), " V ->      V");
    dtostrf(sLastChartData.Millivolt / 1000.0, 5, 2, &tStringBuffer[10]);
    tStringBuffer[15] = ' '; // overwrite terminating null
    tYPosition += tChartDataTextHeight;
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, CHART_VOLTAGE_COLOR,
            sBackgroundColor);

// ESR
    if (!ChartStartValues.inLoggerModeAndFlags) {
        snprintf(tStringBuffer, sizeof(tStringBuffer), "%5u m\x81->%4u m\x81", ChartStartValues.initialDischargingMilliohm,
                sLastChartData.ESRMilliohm);
        tYPosition += tChartDataTextHeight;
        BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, CHART_ESR_COLOR,
                sBackgroundColor);
    }

// Current
    snprintf(tStringBuffer, sizeof(tStringBuffer), "%5u mA->%4u mA", ChartStartValues.initialMilliampere,
            sLastChartData.Milliampere);
    tYPosition += tChartDataTextHeight;
    BlueDisplay1.drawText(CHART_VALUES_POSITION_X, tYPosition, tStringBuffer, sChartDataTextSize, CHART_CURRENT_COLOR,
            sBackgroundColor);
}
#endif // defined(SUPPORT_BLUEDISPLAY_CHART)

/*
 * Ideas for Version 8.0
 * - no arduino serial, only BD status line
 * - constants like shunt resistor as variables (float?), to be set by user
 *
 * Version 7.0 - 10/2025
 *  - Changed analog pin assignments for two additional voltage ranges using internal reference.
 *  - Fixed some bugs in logger mode.
 *  - Detection if powered by USB removed to save space.
 *
 * Version 6.0 - 9/2025
 *  - Improved logger handling.
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
