// MIT License
//
// Copyright (c) 2022 Istvan Zilizi
// Copyright (c) 2025 Ysard
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "RV8263.h"

#if defined (CONFIG_I2C_RV2863_0_ENABLED)
#define I2C_MASTER_NUM    I2C_NUM_0
#endif
#if defined (CONFIG_I2C_RV2863_1_ENABLED)
#define I2C_MASTER_NUM    I2C_NUM_1
#endif

static const char *TAG = "RV8263";


RV8263::RV8263(fncPntr preadI2CFnc, fncPntrConst pwriteI2CFnc) {
    this->_fp_readi2c  = preadI2CFnc;
    this->_fp_writei2c = pwriteI2CFnc;
    this->timezone = TIME_ZONE;
}


RV8263::RV8263() {
    this->_fp_readi2c  = &i2c_manager_read;
    this->_fp_writei2c = &i2c_manager_write;
    this->timezone = TIME_ZONE;
}


const char* RV8263::getTimezone() const {
    return this->timezone;
}


void RV8263::setTimezone(const char* newTimezone) {
    this->timezone = newTimezone;
}


/**
 * @brief Test the clock integrity
 * If the oscillator is not working, a reset should be made.
 * @note ~5 tests are made in the 2.5s allowed
 */
esp_err_t RV8263::isOscillatorRunning(bool *oscillator_not_working) {
    esp_err_t ret;
    uint8_t   reg;

    *oscillator_not_working = true;

    // Stable oscillation is obtained in the range of 200ms to 2s max
    uint32_t timeout = esp_timer_get_time() + 2500;
    do
    {
        ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_SECS, &reg, 1);
        // true if the flag is set
        *oscillator_not_working = ((reg & FLAG_SECONDS_OS) == FLAG_SECONDS_OS);
        if (!*oscillator_not_working) {
            return ret;
        }

        vTaskDelay(pdMS_TO_TICKS(220));

        if (ret == ESP_OK) {
            // Try to clear all the register
            reg = 0;
            ret = this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_SECS, &reg, 1);
        } else {
            ESP_LOGE(TAG, "Communication failed");
        }

        vTaskDelay(pdMS_TO_TICKS(220));
    } while (esp_timer_get_time() < timeout && (ret != ESP_OK || *oscillator_not_working));

    if (ret == ESP_OK) {
        if (*oscillator_not_working) {
            ESP_LOGE(TAG, "Oscillator is not working! Need a reset!");
        } else {
            ESP_LOGI(TAG, "Oscillator is working! Integrity is guaranteed!");
        }
    } else {
        ESP_LOGE(TAG, "Communication failed");
    }
    return ret;
}


esp_err_t RV8263::readAllRegsFromRTC(void) {
    return this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_ALL, (uint8_t *)(&(this->sttime)), sizeof(_ttime) / sizeof(uint8_t));
}


esp_err_t RV8263::writeAllRegsToRTC(void) {
    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_ALL, (uint8_t *)(&(this->sttime)), sizeof(_ttime) / sizeof(uint8_t));
}


esp_err_t RV8263::isTimerWakeUp(bool *bReturn, bool updateRequired) {
    esp_err_t ret;
    uint8_t   reg = 0;

    if (updateRequired) {
        ret = this->readControl2Reg(&reg);
        if (ret != ESP_OK) {
            return ret;
        }
    } else {
        //register filled from internal structure variable
        reg = this->sttime.Control2;
        ret = ESP_OK;
    }
    // Checking the bit; true if the timer is wake up
    *bReturn = (FLAG_CONTROL2_TF_CHECK(reg) == 1);
    return ret;
}


esp_err_t RV8263::isAlarmWakeUp(bool *bReturn, bool updateRequired) {
    esp_err_t ret;
    uint8_t   reg = 0;

    if (updateRequired) {
        ret = this->readControl2Reg(&reg);
        if (ret != ESP_OK) {
            return ret;
        }
    } else {
        //register filled from internal structure variable
        reg = this->sttime.Control2;
        ret = ESP_OK;
    }
    // Checking the bit; true if the timer is wake up
    *bReturn = (FLAG_CONTROL2_AF_CHECK(reg) == 1);
    return ret;
}


esp_err_t RV8263::clearTimerWakeUp() {
    // Clear the bit
    return this->setBit(REG_ADDR_CONTROL2, FLAG_CONTROL2_TF, false);
}


esp_err_t RV8263::clearAlarmWakeUp() {
    // Clear the bit
    return this->setBit(REG_ADDR_CONTROL2, FLAG_CONTROL2_AF, false);
}


esp_err_t RV8263::setAlarmInterrupt(bool enable) { // AIE
    esp_err_t ret;

    return this->setBit(REG_ADDR_CONTROL2, FLAG_CONTROL2_AIE, enable);
}


esp_err_t RV8263::setCountdownInterrupt(bool enable) { // TIE
    esp_err_t ret;

    return this->setBit(REG_ADDR_TIMER_MODE, FLAG_TIMER_MODE_TIE, enable);
}


esp_err_t RV8263::configureCountdownCounter(uint8_t value, eTimeClockFreq clock_freq, uint8_t pulsed) { // value, TD, TI_TP
    esp_err_t ret;
    uint8_t   reg;

    // Configure timer mode register
    // TODO: use internal structure if we are certain that it is synchronized...
    ret = this->readTimerModeFromRTC(&reg);
    if (ret != ESP_OK) {
        return ret;
    }

    reg = FLAG_TIMER_MODE_TD_CLEAR(reg) | clock_freq;
    // Disable TE before changing the timer value
    reg = FLAG_TIMER_MODE_TE_CLEAR(reg);

    if (pulsed) {
        reg |= FLAG_TIMER_MODE_TI_TP;
    } else {
        reg = FLAG_TIMER_MODE_TI_TP_CLEAR(reg);
    }

    ret = this->writeTimerModeToRTC(reg);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set timer value once countdown is configured and TE is disabled
    ret = this->writeTimerValueToRTC(value);
    return ret;
}


esp_err_t RV8263::configureAlarms(uint8_t reg_addr, bool enable, uint8_t value) {
    // Guess the internal structure field with the register name
    uint8_t *sttime_field = NULL;

    switch (reg_addr) {
        case REG_ADDR_SECS_ALARM:
            sttime_field = (uint8_t *)(&(this->sttime.Seconds_Alarm));
            break;

        case REG_ADDR_MINS_ALARM:
            sttime_field = (uint8_t *)(&(this->sttime.Minutes_Alarm));
            break;

        case REG_ADDR_HOURS_ALARM:
            sttime_field = (uint8_t *)(&(this->sttime.Hours_Alarm));
            // TODO: 12hours not handled
            break;

        case REG_ADDR_DATE_ALARM:
            sttime_field = (uint8_t *)(&(this->sttime.Date_Alarm));
            break;

        case REG_ADDR_WDAY_ALARM:
            sttime_field = (uint8_t *)(&(this->sttime.Weekday_Alarm));
            break;

        default:
            ESP_LOGE(TAG, "Register not expected for configuring alarms!");
            return ESP_ERR_INVALID_ARG;
    }

    // Process value
    *sttime_field = RV8263::intToBCD(value);

    if (enable) {
        // Clear bit
        *sttime_field &= ~FLAG_ALARM_DISABLE;
    } else {
        // Set bit
        *sttime_field |= FLAG_ALARM_DISABLE;
    }

    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, reg_addr, sttime_field, 1);
}


esp_err_t RV8263::resetAlarms() {
    this->sttime.Seconds_Alarm = FLAG_ALARM_DISABLE;
    this->sttime.Minutes_Alarm = FLAG_ALARM_DISABLE;
    this->sttime.Hours_Alarm   = FLAG_ALARM_DISABLE;
    this->sttime.Date_Alarm    = FLAG_ALARM_DISABLE;
    this->sttime.Weekday_Alarm = FLAG_ALARM_DISABLE;

    // Write 5 bytes: seconds, minutes, hours, date, weekday alarms
    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_ALARMS, (uint8_t *)(&(this->sttime.Seconds_Alarm)), 5);
}


esp_err_t RV8263::setCountdownCounter(bool enable) {
    esp_err_t ret;

    return this->setBit(REG_ADDR_TIMER_MODE, FLAG_TIMER_MODE_TE, enable);
}


esp_err_t RV8263::setPulseGeneration(bool enable) {
    esp_err_t ret;

    return this->setBit(REG_ADDR_TIMER_MODE, FLAG_TIMER_MODE_TI_TP, enable);
}


esp_err_t RV8263::setMinuteCounter(bool enable) { // MI check mode 0
    esp_err_t ret;
    // uint8_t reg;
    bool fastmode = false;

    ret = this->isOffsetFastModeEnabled(&fastmode);
    if (ret != ESP_OK) {
        return ret;
    }
    if (fastmode) {
        ESP_LOGE(TAG, "Fast mode should not be used with HMI/MI counters! Disable it before.");
        return ret;
    }

    return this->setBit(REG_ADDR_CONTROL2, FLAG_CONTROL2_MI, enable);
}


esp_err_t RV8263::setHalfMinuteCounter(bool enable) { // HMI check mode 0
    esp_err_t ret;
    // uint8_t reg;
    bool fastmode = false;

    ret = this->isOffsetFastModeEnabled(&fastmode);
    if (ret != ESP_OK) {
        return ret;
    }
    if (fastmode) {
        ESP_LOGE(TAG, "Fast mode should not be used with HMI/MI counters! Disable it before.");
        return ret;
    }

    return this->setBit(REG_ADDR_CONTROL2, FLAG_CONTROL2_HMI, enable);
}


esp_err_t RV8263::setBit(uint8_t reg_addr, uint8_t flag, bool enable) {
    esp_err_t ret;

    // Guess the internal structure field with the register name
    uint8_t *sttime_field = NULL;

    switch (reg_addr) {
        case REG_ADDR_CONTROL2:
            sttime_field = (uint8_t *)(&(this->sttime.Control2));
            break;

        case REG_ADDR_TIMER_MODE:
            sttime_field = (uint8_t *)(&(this->sttime.Timer_Mode));
            break;

        default:
            ESP_LOGE(TAG, "Register not expected for updating bit!");
            return ESP_ERR_INVALID_ARG;
    }

    // TODO: use internal structure if we are certain that it is synchronized...
    ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, reg_addr, sttime_field, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    if (enable) {
        *sttime_field |= flag;
    } else {
        *sttime_field &= ~flag;
    }

    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, reg_addr, sttime_field, 1);
}


esp_err_t RV8263::isOffsetFastModeEnabled(bool *bReturn, bool updateRequired) {
    esp_err_t ret;
    uint8_t   reg = 0;

    if (updateRequired) {
        ret = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_OFFSET, (uint8_t *)(&(this->sttime.Offset)), 1);
        reg = (this->sttime.Offset);
        if (ret != ESP_OK) {
            return ret;
        }
    } else {
        //register filled from internal structure variable
        reg = this->sttime.Offset;
        ret = ESP_OK;
    }
    // Checking the bit; true if the timer is wake up
    *bReturn = (FLAG_OFFSET_MODE_CHECK(reg) == 1);
    return ret;
}


esp_err_t RV8263::readControl1Reg(uint8_t *reg) {
    esp_err_t ret;

    ret  = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_CONTROL1, (uint8_t *)(&(this->sttime.Control1)), 1);
    *reg = (this->sttime.Control1);
    return ret;
}


esp_err_t RV8263::writeControl1Reg(uint8_t reg) {
    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_CONTROL1, &reg, 1);
}


esp_err_t RV8263::resetRTC(void) {
    return writeControl1Reg(RESET_RTC);
}


esp_err_t RV8263::readControl2Reg(uint8_t *reg) {
    esp_err_t ret;

    ret  = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_CONTROL2, (uint8_t *)(&(this->sttime.Control2)), 1);
    *reg = (this->sttime.Control2);
    return ret;
}


esp_err_t RV8263::writeControl2Reg(uint8_t reg) {
    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_CONTROL2, &reg, 1);
}


esp_err_t RV8263::readTimeFromRTC(void) {
    // Read 7 bytes: seconds, minutes, hours, date, weekday, month, year
    return this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_TIME, (uint8_t *)(&(this->sttime.Seconds)), 7);
}


esp_err_t RV8263::writeTimeToRTC(void) {
    // Write 7 bytes: seconds, minutes, hours, date, weekday, month, year
    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_TIME, (uint8_t *)(&(this->sttime.Seconds)), 7);
}


esp_err_t RV8263::writeYearToRTC(uint16_t year) {
    uint16_t ny = year - RTC_BIAS_YEAR;
    uint8_t  _year;

    _year             = RV8263::intToBCD((uint8_t)ny);
    this->sttime.Year = _year;
    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_YEAR, &_year, 1);
}


esp_err_t RV8263::readYearFromRTC(uint16_t *pyear) {
    esp_err_t ret;

    ret     = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_YEAR, (uint8_t *)(&(this->sttime.Year)), 1);
    *pyear  = RV8263::bcdToInt(this->sttime.Year & FILTER_YEAR);
    *pyear += RTC_BIAS_YEAR;
    return ret;
}


esp_err_t RV8263::writeMonthToRTC(uint8_t month) {
    uint8_t _month;

    _month             = RV8263::intToBCD(month);
    this->sttime.Month = _month;
    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_MONTH, &_month, 1);
}


esp_err_t RV8263::readMonthFromRTC(uint8_t *pmonth) {
    esp_err_t ret;

    ret     = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_MONTH, (uint8_t *)(&(this->sttime.Month)), 1);
    *pmonth = RV8263::bcdToInt(this->sttime.Month & FILTER_MONTH);
    return ret;
}


esp_err_t RV8263::writeDateToRTC(uint8_t date) {
    uint8_t _date;

    _date = RV8263::intToBCD(date);
    this->sttime.Month = _date;
    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_DATE, &_date, 1);
}


esp_err_t RV8263::readDateFromRTC(uint8_t *pdate) {
    esp_err_t ret;

    ret    = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_DATE, (uint8_t *)(&(this->sttime.Date)), 1);
    *pdate = RV8263::bcdToInt(this->sttime.Date & FILTER_DATE);
    return ret;
}


esp_err_t RV8263::writeSecondsToRTC(uint8_t secs) {
    uint8_t _secs;

    _secs = RV8263::intToBCD(secs);
    this->sttime.Seconds = _secs;
    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_SECS, &_secs, 1);
}


esp_err_t RV8263::readSecondsFromRTC(uint8_t *psecs) {
    esp_err_t ret;

    ret    = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_SECS, (uint8_t *)(&(this->sttime.Seconds)), 1);
    *psecs = RV8263::bcdToInt(this->sttime.Seconds & FILTER_SECS);
    return ret;
}


esp_err_t RV8263::writeMinutesToRTC(uint8_t mins) {
    uint8_t _mins;

    _mins = RV8263::intToBCD(mins);
    this->sttime.Minutes = _mins;
    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_MINS, &_mins, 1);
}


esp_err_t RV8263::readMinutesFromRTC(uint8_t *pmins) {
    esp_err_t ret;

    ret    = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_MINS, (uint8_t *)(&(this->sttime.Minutes)), 1);
    *pmins = RV8263::bcdToInt(this->sttime.Minutes & FILTER_MINS);
    return ret;
}


// TODO: 12hours not handled
esp_err_t RV8263::writeHoursToRTC(uint8_t hours) {
    uint8_t _hours;

    _hours             = RV8263::intToBCD(hours);
    this->sttime.Hours = _hours;
    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_HOURS, &_hours, 1);
}


esp_err_t RV8263::readHoursFromRTC(uint8_t *phours) {
    esp_err_t ret;

    ret     = this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_HOURS, (uint8_t *)(&(this->sttime.Hours)), 1);
    *phours = RV8263::bcdToInt(this->sttime.Hours & FILTER_HOURS);
    return ret;
}


esp_err_t RV8263::getEpoch(time_t *epoch, bool updateRequired) {
    struct tm timeinfo;

    // Get UTC time from the chip
    setenv("TZ", "UTC0", 1);
    tzset();

    if (updateRequired) {
        esp_err_t ret = this->readAllRegsFromRTC();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Read registers failed");
            return ret;
        }
    }

    // This is year-1900, so RTC store from 2000
    // According to https://en.cppreference.com/w/cpp/chrono/c/mktime subtract is required
    // Note: tm_year stores 125 for 2025, we need 25
    // So... 125 = 25 + 2000 - 1900
    timeinfo.tm_year = RV8263::bcdToInt(this->sttime.Year & FILTER_YEAR) + RTC_BIAS_YEAR - EPOCH_YEAR;
    timeinfo.tm_mon  = RV8263::bcdToInt(this->sttime.Month & FILTER_MONTH) - EPOCH_BIAS_MONTH;
    timeinfo.tm_mday = RV8263::bcdToInt(this->sttime.Date & FILTER_DATE);
    timeinfo.tm_hour = RV8263::bcdToInt(this->sttime.Hours & FILTER_HOURS);
    timeinfo.tm_min  = RV8263::bcdToInt(this->sttime.Minutes & FILTER_MINS);
    timeinfo.tm_sec  = RV8263::bcdToInt(this->sttime.Seconds & FILTER_SECS);
    // Disable DST, assume local time is UTC+0 without DST
    // DST will be set during the timezone conversion in another function
    timeinfo.tm_isdst = -1;

    *epoch = mktime(&timeinfo);

    // Apply the current timezone
    setenv("TZ", this->timezone, 1);
    tzset();

    if (_ESP_LOG_ENABLED(ESP_LOG_INFO)) {
        char strftime_buf[64];
        timeinfo = *localtime(epoch);
        strftime(strftime_buf, sizeof(strftime_buf), "%c %Z", &timeinfo);
        ESP_LOGI(TAG, "Get UTC time: %s", strftime_buf);
    }
    return ESP_OK;
}


esp_err_t RV8263::writeTimeFromEpochToRTC(const time_t epoch) {
    struct tm timeinfo;

    // Set UTC time for the chip
    setenv("TZ", "UTC0", 1);
    tzset();

    timeinfo = *localtime(&epoch);

    if (_ESP_LOG_ENABLED(ESP_LOG_INFO)) {
        char strftime_buf[64];
        strftime(strftime_buf, sizeof(strftime_buf), "%c %Z", &timeinfo);
        ESP_LOGI(TAG, "Get UTC time: %s", strftime_buf);
    }

    // Note: tm_year stores 125 for 2025, we need 25
    // So... 125 + 1900 - 2000 = 25
    this->sttime.Year    = RV8263::intToBCD(timeinfo.tm_year + EPOCH_YEAR - RTC_BIAS_YEAR);
    this->sttime.Month   = RV8263::intToBCD(timeinfo.tm_mon + EPOCH_BIAS_MONTH);
    this->sttime.Weekday = RV8263::intToBCD(timeinfo.tm_wday);
    this->sttime.Date    = RV8263::intToBCD(timeinfo.tm_mday);
    this->sttime.Hours   = RV8263::intToBCD(timeinfo.tm_hour);
    this->sttime.Minutes = RV8263::intToBCD(timeinfo.tm_min);
    this->sttime.Seconds = RV8263::intToBCD(timeinfo.tm_sec);

    // Restore the current timezone
    setenv("TZ", this->timezone, 1);
    tzset();

    return this->writeTimeToRTC();
}


esp_err_t RV8263::readTimerValueFromRTC(uint8_t *pTimerValue) {
    return this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_TIMER_VALUE, pTimerValue, 1);
}


esp_err_t RV8263::writeTimerValueToRTC(uint8_t timerValue) {
    esp_err_t ret;

    ret = this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_TIMER_VALUE, &timerValue, 1);
    this->sttime.Timer_Value = timerValue;
    return ret;
}


esp_err_t RV8263::readTimerModeFromRTC(uint8_t *pTimerMode) {
    return this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_TIMER_MODE, pTimerMode, 1);
}


esp_err_t RV8263::writeTimerModeToRTC(uint8_t timerMode) {
    esp_err_t ret;

    ret = this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_TIMER_MODE, &timerMode, 1);
    this->sttime.Timer_Mode = timerMode;
    return ret;
}


esp_err_t RV8263::readRAMFromRTC(uint8_t *pData) {
    return this->_fp_readi2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_RAM, pData, 1);
}


esp_err_t RV8263::writeRAMToRTC(uint8_t data) {
    return this->_fp_writei2c(I2C_MASTER_NUM, ADDRESS_RTC, REG_ADDR_RAM, &data, 1);
    // TODO sync RAM
}


esp_err_t RV8263::printAllRegs(bool updateRequired) {
    esp_err_t ret;

    if (updateRequired) {
        ret = this->RV8263::readAllRegsFromRTC();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    ESP_LOGI(TAG, "Data Content :");
    uint8_t *p = (uint8_t *)(&(this->sttime));
    for (uint8_t i = 0; i < (sizeof(_ttime) / sizeof(uint8_t)); i++)
    {
        ESP_LOGI(TAG, "0x%02X : %02X\n", i, *(p + i));
    }
    return ESP_OK;
}


RV8263::~RV8263() {
}


uint8_t RV8263::intToBCD(uint8_t num) {
    return ((num / 10) << 4) | (num % 10);
}


uint8_t RV8263::bcdToInt(uint8_t bcd) {
    // 0x10
    return ((bcd >> 4) * 10) + (bcd & 0x0f);
}


esp_err_t RV8263::getFormattedDateTime(const char *formatter, char *buffer, size_t len, bool updateRequired) {
    time_t epoch;

    this->getEpoch(&epoch, updateRequired);
    struct tm timeinfo = *localtime(&epoch);

    if (strftime(buffer, len, formatter, &timeinfo) == 0) {
        ESP_LOGE(TAG, "Insufficient buffer size allocation");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}


char * RV8263::getFormattedDateTime(bool updateRequired) {
    static char buffer[16]; // Max of YYYYMMDD_HHMMSS with \0 terminator

    this->getFormattedDateTime("%Y%m%d_%H%M%S", buffer, sizeof(buffer), updateRequired);
    return buffer;
}


bool RV8263::isInDSTime(int day, int month, int dow) {
    if ((month < 3) || (month > 10)) {
        return false;
    }
    if ((month > 3) && (month < 10)) {
        return true;
    }

    int previousSunday = day - dow;

    if (month == 3) {
        return previousSunday >= 25;
    }
    if (month == 10) {
        return previousSunday < 25;
    }

    ESP_LOGE(TAG, "has reached an unexpected place!");
    return false;     // this line never gonna happen
}


bool RV8263::isInDSTime(bool updateRequired) {
    time_t epoch;

    this->getEpoch(&epoch, updateRequired);
    struct tm timeinfo = *localtime(&epoch);

    return (timeinfo.tm_isdst == 1);
}
