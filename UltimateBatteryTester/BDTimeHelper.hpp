/*
 * BDTimeHelper.hpp
 *
 *  Utility functions to display the BlueDisplay timestamp provided by the BD Host in the Arduino program.
 *  !!! It requires the Arduino files Time.cpp and TimeLib.h, so it is not directly part of the BD library. It is only included in some of the examples !!!
 *  Requires up to 1640 bytes program memory and even more in Arduino IDE, because the "-mrelax" linker option is not set there.
 *
 *  Copyright (C) 2025  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of BlueDisplay https://github.com/ArminJo/Arduino-BlueDisplay.
 *
 *  BlueDisplay is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 */

#include <Arduino.h>

#include <Colors.h> // for color16_t
#include <BlueDisplayProtocol.h> // for ByteShortLongFloatUnion

#ifndef _BD_TIME_HELPER_HPP
#define _BD_TIME_HELPER_HPP

#if !defined(MILLIS_IN_ONE_SECOND)
#define MILLIS_IN_ONE_SECOND    1000U
#endif
#if !defined(SECONDS_IN_ONE_MINUTE)
#define SECONDS_IN_ONE_MINUTE   60U
#endif
#if !defined(SECONDS_IN_ONE_HOUR)
#define SECONDS_IN_ONE_HOUR     3600U
#endif
#if !defined(SECONDS_IN_ONE_DAY)
#define SECONDS_IN_ONE_DAY      86400UL
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

#if !defined(TIME_EVENTCALLBACK_FUNCTION)
#define TIME_EVENTCALLBACK_FUNCTION     getTimeEventMinimalCallback
#endif
#if !defined(BD_TIME_SYNCHRONISATION_INTERVAL_SECONDS)
#define BD_TIME_SYNCHRONISATION_INTERVAL_SECONDS    SECONDS_IN_ONE_DAY // get a fresh timestamp every day
#endif
#define WAIT_FOR_TIME_SYNC_MAX_MILLIS               150 // Wait for requested time event but terminate at least after 150 ms

#if false
#define USE_C_TIME
/*
 * Here, we must always get a new timestamp from host if we are calling now().
 * There is no usage of the Arduino millis() function like in Time.cpp :-(
 */
#include "time.h"
struct tm *sTimeInfo;
bool sTimeInfoWasJustUpdated = false;
/* Useful Constants */
#define DAYS_PER_WEEK ((time_t)(7UL))
#define SECS_PER_WEEK ((time_t)(SECONDS_IN_ONE_DAY * DAYS_PER_WEEK))
#define SECS_PER_YEAR ((time_t)(SECONDS_IN_ONE_DAY * 365UL)) // TODO: ought to handle leap years
#define SECS_YR_2000  ((time_t)(946684800UL)) // the time at the start of y2k

uint16_t waitUntilTimeWasUpdated(uint16_t aMaxWaitMillis);
#else
#include "Time.hpp"
time_t requestHostUnixTimestamp();
#endif

void initLocalTimeHandling();
void getTimeEventMinimalCallback(uint8_t aSubcommand, uint8_t aByteInfo, uint16_t aShortInfo, ByteShortLongFloatUnion aLongInfo);

/*
 * To be called once in initDisplay().
 * Sets requestHostUnixTimestamp() as sync provider function and call it.
 * Set interval for calling sync / requestHostUnixTimestamp() to 1 day.
 * This interval is checked at every call to now().
 * Wait maximum 150 ms for the response event
 */
void initLocalTimeHandling() {
#  if defined(USE_C_TIME)
    // initialize sTimeInfo
    sTimeInfo = localtime((const time_t*) BlueDisplay1.getHostUnixTimestamp());
#  else
    /*
     * Set function to call when time sync required, and sync interval (default: after 300 seconds)
     */
    setSyncProvider(requestHostUnixTimestamp); // This always calls requestHostUnixTimestamp() -> getInfo() immediately, which will set time again.
    setSyncInterval(BD_TIME_SYNCHRONISATION_INTERVAL_SECONDS); // Sync time with BD host. Checked at every call to now().
#  endif
    delayMillisAndCheckForEvent(WAIT_FOR_TIME_SYNC_MAX_MILLIS); // Wait for requested time event but terminate at least after 150 ms
}

void printTimeAtOneLine(uint16_t const aPositionX, uint16_t const aPositionY, uint16_t const aFontSize, color16_t const aTextColor,
        color16_t const aBackgroundColor) {
    char tStringBuffer[20];
#  if defined(USE_C_TIME)
    BlueDisplay1.getInfo(SUBFUNCTION_GET_INFO_LOCAL_TIME, TIME_EVENTCALLBACK_FUNCTION);
#    if defined(DEBUG)
    uint16_t tElapsedTime = waitUntilTimeWasUpdated(WAIT_FOR_TIME_SYNC_MAX_MILLIS); // 150 ms
    BlueDisplay1.debug("Time sync lasts ", tElapsedTime, " ms"); // timeout if 0
#    else
    waitUntilTimeWasUpdated(WAIT_FOR_TIME_SYNC_MAX_MILLIS); // 150 ms
#    endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
    snprintf_P(tStringBuffer, sizeof(tStringBuffer), PSTR("%02d.%02d.%4d %02d:%02d:%02d"), sTimeInfo->tm_mday, sTimeInfo->tm_mon,
            sTimeInfo->tm_year + 1900, sTimeInfo->tm_hour, sTimeInfo->tm_min, sTimeInfo->tm_sec);
#pragma GCC diagnostic pop
#  else
    time_t tTimestamp = now(); // use this timestamp for all prints below
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-overflow"
    // We can ignore warning: '%02d' directive writing between 2 and 11 bytes into a region of size between 0 and 1 [-Wformat-overflow=]
    snprintf_P(tStringBuffer, sizeof(tStringBuffer), PSTR("%02d.%02d.%4d %02d:%02d:%02d"), day(tTimestamp), month(tTimestamp),
            year(tTimestamp), hour(tTimestamp), minute(tTimestamp), second(tTimestamp));
#pragma GCC diagnostic pop
#  endif //defined(USE_C_TIME)
    BlueDisplay1.drawText(aPositionX, aPositionY, tStringBuffer, aFontSize, aTextColor, aBackgroundColor);
}

void printTimeAtTwoLines(uint16_t const aPositionX, uint16_t const aPositionY, uint16_t const aFontSize, color16_t const aTextColor,
        color16_t const aBackgroundColor) {
    char tStringBuffer[20];
#  if defined(USE_C_TIME)
    BlueDisplay1.getInfo(SUBFUNCTION_GET_INFO_LOCAL_TIME, TIME_EVENTCALLBACK_FUNCTION);
#    if defined(DEBUG)
    uint16_t tElapsedTime = waitUntilTimeWasUpdated(WAIT_FOR_TIME_SYNC_MAX_MILLIS); // 150 ms
    BlueDisplay1.debug("Time sync lasts ", tElapsedTime, " ms"); // timeout if 0
#    else
    waitUntilTimeWasUpdated(WAIT_FOR_TIME_SYNC_MAX_MILLIS); // 150 ms
#    endif
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"
    snprintf_P(tStringBuffer, sizeof(tStringBuffer), PSTR("%02d.%02d.%4d\n%02d:%02d:%02d"), sTimeInfo->tm_mday, sTimeInfo->tm_mon,
            sTimeInfo->tm_year + 1900, sTimeInfo->tm_hour, sTimeInfo->tm_min, sTimeInfo->tm_sec);
#pragma GCC diagnostic pop
#  else
    time_t tTimestamp = now(); // use this timestamp for all prints below
// We can ignore warning: '%02d' directive writing between 2 and 11 bytes into a region of size between 0 and 1 [-Wformat-overflow=]
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-overflow"
    snprintf_P(tStringBuffer, sizeof(tStringBuffer), PSTR("%02d.%02d.%4d\n%02d:%02d:%02d"), day(tTimestamp), month(tTimestamp),
            year(tTimestamp), hour(tTimestamp), minute(tTimestamp), second(tTimestamp));
#pragma GCC diagnostic pop
#  endif //defined(USE_C_TIME)
    BlueDisplay1.drawText(aPositionX, aPositionY, tStringBuffer, aFontSize, aTextColor, aBackgroundColor);
}

#  if defined(USE_C_TIME)
/*
 * @return 0 if time was not updated in aMaxWaitMillis milliseconds, else millis used for update
 */
uint16_t waitUntilTimeWasUpdated(uint16_t aMaxWaitMillis) {
    unsigned long tStartMillis = millis();
    while (millis() - tStartMillis < aMaxWaitMillis) {
        checkAndHandleEvents(); // BlueDisplay function, which may call getTimeEventCallback() if SUBFUNCTION_GET_INFO_LOCAL_TIME event received
        if (sTimeInfoWasJustUpdated) {
            return millis() - tStartMillis;
        }
    }
    return 0;
}
#  else
/*
 * Is set as SyncProvider by initLocalTimeHandling() and then cyclically called from TimeLib
 */
time_t requestHostUnixTimestamp() {
    BlueDisplay1.getInfo(SUBFUNCTION_GET_INFO_LOCAL_TIME, TIME_EVENTCALLBACK_FUNCTION);
    return 0; // the time will be sent later in response to getInfo and must be copied manually
}
#  endif // #if defined(USE_C_TIME)

/*
 * Is called at startup and every time the actual time is required
 */
void getTimeEventMinimalCallback(uint8_t aSubcommand, uint8_t aByteInfo, uint16_t aShortInfo, ByteShortLongFloatUnion aLongInfo) {
    (void) aSubcommand;
    (void) aByteInfo;
    (void) aShortInfo;
#  if defined(USE_C_TIME)
        BlueDisplay1.setHostUnixTimestamp(aLongInfo.uint32Value);
        sTimeInfoWasJustUpdated = true;
        sTimeInfo = localtime((const time_t*) &aLongInfo.uint32Value); // Compute minutes, hours, day, month etc. for later use in printTime
#  else
    setTime(aLongInfo.uint32Value);
#  endif // defined(USE_C_TIME)
}

#if defined(_CHART_H)
// Here time_float_union is declared
/*
 * Here we take the long part of the value and return strings like "27" or "2:27"
 */
int convertUnixTimestampToDateString(char *aLabelStringBuffer, time_float_union aXvalue) {
#  if defined(USE_C_TIME)
    sTimeInfo = localtime((const time_t*) aXvalue.TimeValue);
    return snprintf(aLabelStringBuffer, CHART_LABEL_STRING_BUFFER_SIZE, "%d.%d", sTimeInfo->tm_mday, sTimeInfo->tm_mon);
#  else
    return snprintf(aLabelStringBuffer, CHART_LABEL_STRING_BUFFER_SIZE, "%d.%d", day(aXvalue.TimeValue), month(aXvalue.TimeValue));
#  endif
}

int convertUnixTimestampToHourString(char *aLabelStringBuffer, time_float_union aXvalue) {
#  if defined(USE_C_TIME)
    sTimeInfo = localtime((const time_t*) aXvalue.TimeValue);
    return snprintf(aLabelStringBuffer, CHART_LABEL_STRING_BUFFER_SIZE, "%d", sTimeInfo->tm_hour);
#  else
    return snprintf(aLabelStringBuffer, CHART_LABEL_STRING_BUFFER_SIZE, "%d", hour(aXvalue.TimeValue));
#  endif
}

int convertUnixTimestampToHourAndMinuteString(char *aLabelStringBuffer, time_float_union aXvalue) {
#  if defined(USE_C_TIME)
    sTimeInfo = localtime((const time_t*) aXvalue.TimeValue);
    return snprintf(aLabelStringBuffer, CHART_LABEL_STRING_BUFFER_SIZE, "%2d:%02d", sTimeInfo->tm_hour, sTimeInfo->tm_min);
#  else
    return snprintf(aLabelStringBuffer, CHART_LABEL_STRING_BUFFER_SIZE, "%2d:%02d", hour(aXvalue.TimeValue),
            minute(aXvalue.TimeValue));
#  endif
}
#endif
#endif // _BD_TIME_HELPER_HPP

