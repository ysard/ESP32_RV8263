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
#ifndef __RV8263__
#define __RV8263__

#include <cstring>
#include <time.h>
#include <esp_log.h>
#include "esp_err.h"
#include "esp_timer.h"
// Only here to get the I2C_NUM_0 and I2C_NUM_1 defines.
#include <driver/i2c.h>
// I2C Manager driver
#include <i2c_manager.h>

// Default from kconfig
// #define CONFIG_EPOCH_YEAR    1900
// #define CONFIG_TIME_ZONE     CET-1CEST,M3.5.0,M10.5.0/3 (Europe/Paris)
// #define I2C_MASTER_NUM       I2C_NUM_0

// i2c Address
#define ADDRESS_RTC                       0x51

// Register Addresses
#define REG_ADDR_ALL                      0x00
#define REG_ADDR_CONTROL1                 0x00
#define REG_ADDR_CONTROL2                 0x01
#define REG_ADDR_OFFSET                   0x02
#define REG_ADDR_RAM                      0x03
#define REG_ADDR_TIME                     0x04 // Internal use: alias for secs addr
#define REG_ADDR_SECS                     0x04
#define REG_ADDR_MINS                     0x05
#define REG_ADDR_HOURS                    0x06
#define REG_ADDR_DATE                     0x07
#define REG_ADDR_WEEKDAY                  0x08
#define REG_ADDR_MONTH                    0x09
#define REG_ADDR_YEAR                     0x0A
#define REG_ADDR_ALARMS                   0x0B // Internal use: alias for secs alarm addr
#define REG_ADDR_SECS_ALARM               0x0B
#define REG_ADDR_MINS_ALARM               0x0C
#define REG_ADDR_HOURS_ALARM              0x0D
#define REG_ADDR_DATE_ALARM               0x0E
#define REG_ADDR_WDAY_ALARM               0x0F
#define REG_ADDR_TIMER_VALUE              0x10
#define REG_ADDR_TIMER_MODE               0x11

// Control 1 register
#define RESET_RTC                         0x58
#define FLAG_CONTROL1_12_24               0x02
#define FLAG_CONTROL1_STOP                0x20

// Control 2 register
#define FLAG_CONTROL2_AIE                 0x80 /*Alarm Interrupt Enable*/
#define FLAG_CONTROL2_AIE_CLEAR(x)        (x & ~(FLAG_CONTROL2_AIE))
#define FLAG_CONTROL2_AF                  0x40 /*Alarm Flag*/
#define FLAG_CONTROL2_AF_CHECK(x)         ((x >> 6) & 1)
#define FLAG_CONTROL2_AF_CLEAR(x)         (x & ~(FLAG_CONTROL2_AF))
#define FLAG_CONTROL2_MI                  0x20 /*Minute Interrupt Enable*/
#define FLAG_CONTROL2_MI_CLEAR(x)         (x & ~(FLAG_CONTROL2_MI))
#define FLAG_CONTROL2_HMI                 0x10 /*Half Minute Interrupt Enable*/
#define FLAG_CONTROL2_HMI_CLEAR(x)        (x & ~(FLAG_CONTROL2_HMI))
#define FLAG_CONTROL2_TF                  0x08 /*Timer Flag*/
#define FLAG_CONTROL2_TF_CHECK(x)         ((x >> 3) & 1)
#define FLAG_CONTROL2_TF_CLEAR(x)         (x & ~(FLAG_CONTROL2_TF))
#define FLAG_CONTROL2_FD                  0x07 /*CLKOUT Frequency*/
#define FD_32kHz                          0x00 /*32.768 kHz – Default value*/
#define FD_16kHz                          0x01 /*16.384 kHz*/
#define FD_8kHz                           0x02 /* 8.192 kHz*/
#define FD_4kHz                           0x03 /* 4.096 kHz*/
#define FD_2kHz                           0x04 /* 2.048 kHz*/
#define FD_1kHz                           0x05 /* 1.024 kHz*/
#define FD_1Hz                            0x06 /* 1      Hz*/
#define FD_CLKOUT_LOW                     0x07 /*CLKOUT = LOW*/

// Offset register
#define FLAG_OFFSET_MODE                  0x80
#define FLAG_OFFSET_MODE_CHECK(x)         ((x >> 7) & 1)
#define FLAG_OFFSET_OFFSET                0x7F

// Seconds register
#define FLAG_SECONDS_OS                   0x80

// Filter default values registers 0x04-0x0A
#define FILTER_SECS                       0x7F
#define FILTER_MINS                       0x7F
#define FILTER_HOURS                      0x7F // AM/PM distinction included
#define FILTER_DATE                       0x3F
#define FILTER_WEEKDAY                    0x07
#define FILTER_MONTH                      0x1F
#define FILTER_YEAR                       0xFF

#define FILTER_TIMER_VALUE                0xFF

// Seconds Alarm register
#define FLAG_SECS_ALARM_AE_S              0x80
#define FLAG_SECS_ALARM                   0x7F
// Minutes Alarm register
#define FLAG_MINS_ALARM_AE_M              0x80
#define FLAG_MINS_ALARM                   0x7F
// Hours Alarm register
#define FLAG_HOURS_ALARM_AE_H             0x80
#define FLAG_HOURS_ALARM                  0x3F // AM/PM distinction included
// Date Alarm register
#define FLAG_DATE_ALARM_AE_D              0x80
#define FLAG_DATE_ALARM                   0x3F
// Weekday Alarm register
#define FLAG_WDAY_ALARM_AE_W              0x80
#define FLAG_WDAY_ALARM                   0x07
#define FLAG_ALARM_DISABLE                0x80                // Disabled value (default)

// Timer mode registers
#define FLAG_TIMER_MODE_TD                0x18               /* Timer Clock Frequency */
#define FLAG_TIMER_MODE_TD_CLEAR(x)       (x & ~(FLAG_TIMER_MODE_TD))
#define FLAG_TIMER_MODE_TE                0x04               /* Timer Enable*/
#define FLAG_TIMER_MODE_TE_CHECK(x)       ((x >> 2) & 1)
#define FLAG_TIMER_MODE_TE_CLEAR(x)       (x & ~(FLAG_TIMER_MODE_TE))
#define FLAG_TIMER_MODE_TIE               0x02               /* Timer Interrupt Enable */
#define FLAG_TIMER_MODE_TIE_CHECK(x)      ((x >> 1) & 1)
#define FLAG_TIMER_MODE_TIE_CLEAR(x)      (x & ~(FLAG_TIMER_MODE_TIE))
#define FLAG_TIMER_MODE_TI_TP             0x01               /* Timer Interrupt Mode */
#define FLAG_TIMER_MODE_TI_TP_CHECK(x)    (x & 1)
#define FLAG_TIMER_MODE_TI_TP_CLEAR(x)    (x & ~FLAG_TIMER_MODE_TI_TP)

// FLAG_TIMER_MODE_TD values
#define TD_4kHz                           0x00       /* 4.096 kHz */
#define TD_64Hz                           0x01       /* 64    Hz */
#define TD_1Hz                            0x02       /* 1     Hz */
#define TD_1_60Hz                         0x03       /* 1/60  Hz - Default value */

// Misc
#define EPOCH_YEAR                        CONFIG_EPOCH_YEAR
#define EPOCH_BIAS_MONTH                  1
#define TIME_ZONE                         CONFIG_TIME_ZONE
#define HOURS_SECS                        3600
#define RTC_BIAS_YEAR                     2000


enum eTimeClockFreq
{
    _4kHz = TD_4kHz, _64Hz = TD_64Hz, SEC = TD_1Hz, MIN = TD_1_60Hz
};


// Stucture to store the RTC values in memory, Coded with BCD
// DONE operator [] overload for struct
typedef struct  __attribute__ ((packed))
{
    uint8_t Control1;           // 0x00
    uint8_t Control2;           // 0x01
    uint8_t Offset;             // 0x02
    uint8_t RAM;                // 0x03
    uint8_t Seconds;            // 0x04
    uint8_t Minutes;            // 0x05
    uint8_t Hours;              // 0x06
    uint8_t Date;               // 0x07
    uint8_t Weekday;            // 0x08
    uint8_t Month;              // 0x09
    uint8_t Year;               // 0x0A
    uint8_t Seconds_Alarm;      // 0x0B
    uint8_t Minutes_Alarm;      // 0x0C
    uint8_t Hours_Alarm;        // 0x0D
    uint8_t Date_Alarm;         // 0x0E
    uint8_t Weekday_Alarm;      // 0x0F
    uint8_t Timer_Value;        // 0x10
    uint8_t Timer_Mode;         // 0x11
    uint8_t& operator[](std::size_t idx) {
        return *(uint8_t *)((size_t)this + idx * sizeof(uint8_t));
    }
} _ttime;

// Typedef for the function pointers --> seems both the read and write use the similar definition...
// typedef esp_err_t (*fncPntr)(i2c_port_t, uint8_t, uint8_t, uint8_t *, size_t );

// Typedef for i2c_manager funcs
typedef esp_err_t (*fncPntr)(i2c_port_t, uint16_t, uint32_t, uint8_t *, uint16_t);
typedef esp_err_t (*fncPntrConst)(i2c_port_t, uint16_t, uint32_t, const uint8_t *, uint16_t);


class RV8263 {
private:
    uint8_t intToBCD(uint8_t num);
    uint8_t bcdToInt(uint8_t bcd);

    fncPntrConst _fp_writei2c = nullptr;
    fncPntr _fp_readi2c       = nullptr;
    const char *timezone      = nullptr;
    _ttime sttime             = {};

public:
    RV8263(fncPntr readI2CFnc, fncPntrConst writeI2CFnc);
    RV8263();

    /**
     * @brief Test the clock integrity
     * If the oscillator is not working, a reset should be made.
     * @param[out] oscillator_not_working True if the oscillator is not working
     * @note ~5 tests are made in the 2.5s allowed
     */
    esp_err_t isOscillatorRunning(bool *oscillator_not_working);

    /**
     * @brief Sync the internal structure with the RTC chip
     * @warning This function should be used right after the object initialization
     */
    esp_err_t readAllRegsFromRTC(void);
    esp_err_t writeAllRegsToRTC(void);

    /**
     * @brief Reset all the registers of the RTC
     * @note Can be required if Power On Reset process
     *  ends up with corrupted registers, or if the oscillator
     *  has stopped due to power supply instability.
     */
    esp_err_t resetRTC(void);


    /**
     * @brief Get the UNIX timestamp
     * @warning The local time displayed is an offset of the display
     *  and not an offset of the Unix time on the Unix machine.
     *  Thus the timestamp IS NOT and SHOULD NOT be influenced by
     *  any timezone.
     */
    time_t getEpoch(void);

    /**
     * @brief Set the time to the RTC
     * @param[in] epoch UNIX timestamp
     * @see `getEpoch`
     */
    esp_err_t writeTimeFromEpochToRTC(const time_t epoch);

    /**
     * @brief Get the current timezone in POSIX format
     * @see `setTimezone`
     */
    const char * getTimezone() const;

    /**
     * @brief Set the current timezone in POSIX format
     * @param[in] newTimezone Timezone in POSIX format
     * @see `getTimezone`
     */
    void setTimezone(const char *newTimezone);

    /**
     * @brief Get a formatted date
     * @param[in]  formatter Date formatter used by `strftime`
     * @param[out] buffer    Buffer in which the date string will be writen
     * @param[in]  len       Size of the given buffer
     * @return ESP_ERR_NO_MEM if the buffer was too small
     */
    esp_err_t getFormattedDateTime(const char *formatter, char *buffer, size_t len);

    /**
     * @brief Get the date in the YYYYMMDD_HHMMSS format
     */
    char * getFormattedDateTime();

    /**
     * @brief Check if the given date is concerned by the Daylight Saving Time
     * @param dow Day of week (0 is sunday)
     * @return True if it's summer time
     */
    bool isInDLSTime(int day, int month, int dow);

    /**
     * @brief Get the Daylight Saving Time status of the current time
     * @return True if it's summer time
     */
    bool isInDLSTime();

    esp_err_t writeYearToRTC(uint16_t);
    esp_err_t readYearFromRTC(uint16_t *);
    esp_err_t writeMonthToRTC(uint8_t);
    esp_err_t readMonthFromRTC(uint8_t *);
    esp_err_t writeDateToRTC(uint8_t);
    esp_err_t readDateFromRTC(uint8_t *);
    esp_err_t writeSecondsToRTC(uint8_t);
    esp_err_t readSecondsFromRTC(uint8_t *);
    esp_err_t writeMinutesToRTC(uint8_t);
    esp_err_t readMinutesFromRTC(uint8_t *);
    esp_err_t writeHoursToRTC(uint8_t);
    esp_err_t readHoursFromRTC(uint8_t *);

    /* Timers */

    /**
     * @brief Get the status of the TF flag associated to the MI, HMI, countdown counters
     * @param[out] bReturn        True if the interrupt was triggered
     * @param[in]  updateRequired Set to True to directly interrogate the chip
     */
    esp_err_t isTimerWakeUp(bool *bReturn, bool updateRequired = true); // TF

    /**
     * @brief Clear the TF flag associated to the MI, HMI, countdown counters
     */
    esp_err_t clearTimerWakeUp(); // TF

    /**
     * @brief Configure the output of countdown timer as an interrupt
     * @param enable If True, the TIE flag is not set, the output will not be triggered.
     */
    esp_err_t setCountdownInterrupt(bool enable = true); // TIE

    /**
     * @brief Configure the countdown counter
     * @param value      Configure the number of countdown periods (1-255)
     * @param clock_freq Configure the countdown period duration (TD flag)
     * @param pulsed     If True, enable the pulse generator for this timer (TI_TP flag)
     * @note This will stop the TE countdown counter to avoid a corruption of the counter.
     */
    esp_err_t configureCountdownCounter(uint8_t value, eTimeClockFreq clock_freq = MIN, uint8_t pulsed = false); // value, TD, TI_TP

    /**
     * @brief Enable pulse generation on MI, HMI, countdown interrupts
     * Update the TI_TP flag.
     * @param enable If True, when an interrupt is triggered, the counter
     *  automatically re-loads and starts the next timer period.
     *  If False, the timer is in one-shot mode.
     */
    esp_err_t setPulseGeneration(bool enable = true); // TI_TP

    /**
     * @brief Set the status of the countdown counter
     * Update the TE flag.
     */
    esp_err_t setCountdownCounter(bool enable = true); // TE

    /**
     * @brief Set the status of the Minute counter
     * Update the MI flag.
     * @note Can't be used with OFFSET Fast Mode.
     */
    esp_err_t setMinuteCounter(bool enable = true); // MI

    /**
     * @brief Set the status of the Half Minute counter
     * Update the HMI flag.
     * @note Can't be used with OFFSET Fast Mode.
     */
    esp_err_t setHalfMinuteCounter(bool enable = true); // HMI

    // updateRequired is false because we do not provide a function to modify this bit
    esp_err_t isOffsetFastModeEnabled(bool *, bool = false);

    /**
     * @brief Set or unset a flag in the given register
     * For internal use, only for CONTROL2 & TIMER_MODE registers.
     */
    esp_err_t setBit(uint8_t reg_addr, uint8_t flag, bool enable);

    /**
     * @brief Get the timer value
     */
    esp_err_t readTimerValueFromRTC(uint8_t *);

    /**
     * @brief Set the timer value
     * @see Prefer to use `configureCountdownCounter`
     */
    esp_err_t writeTimerValueToRTC(uint8_t);

    /**
     * @brief Get the timer mode register
     * For internal use, low level function.
     * @see `configureCountdownCounter`, `setCountdownCounter`,
     *  `setCountdownInterrupt`, `setPulseGeneration`
     */
    esp_err_t readTimerModeFromRTC(uint8_t *);

    /**
     * @brief Set the timer mode register
     * For internal use, low level function.
     * @see `configureCountdownCounter`, `setCountdownCounter`,
     *  `setCountdownInterrupt`, `setPulseGeneration`
     */
    esp_err_t writeTimerModeToRTC(uint8_t);


    /* Alarms */

    /**
     * @brief Get the status of the AF flag associated to the alarms
     * @param[out] bReturn        True if the interrupt was triggered
     * @param[in]  updateRequired Set to True to directly interrogate the chip
     */
    esp_err_t isAlarmWakeUp(bool *bReturn, bool updateRequired = true); // AF

    /**
     * @brief Clear the AF flag associated to the alarms
     */
    esp_err_t clearAlarmWakeUp(); // AF

    /**
     * @brief Configure the output of the alarms as an interrupt
     * If the AIE flag is not set, the output will not be triggered.
     */
    esp_err_t setAlarmInterrupt(bool enable = true); // AIE

    /**
     * @brief Configure any of the alarm registers
     * @param reg_addr Choose FLAG_SECS_ALARM, FLAG_MINS_ALARM, FLAG_HOURS_ALARM,
     *  FLAG_DATE_ALARM, FLAG_WDAY_ALARM
     * @param enable   If False, the alarm is disabled & it's value is reset
     * @param value    Set seconds, minutes, hours (24h), date, weekday values
     * @note For now, hours can only be in 24h format
     */
    esp_err_t configureAlarms(uint8_t reg_addr, bool enable, uint8_t value = 0);

    /**
     * @brief Reset & disable all the alarms
     */
    esp_err_t resetAlarms();


    // RAM byte implementation
    esp_err_t readRAMFromRTC(uint8_t *);
    esp_err_t writeRAMToRTC(uint8_t);

    // Low level functions
    esp_err_t readControl1Reg(uint8_t *);
    esp_err_t writeControl1Reg(uint8_t);
    esp_err_t readControl2Reg(uint8_t *);
    esp_err_t writeControl2Reg(uint8_t);

    /**
     * @brief Fill the internal time structure with the RTC time related data
     *  Equivalent to a synchronisation.
     */
    esp_err_t readTimeFromRTC(void);

    /**
     * @brief Synchronize the RTC time related registers with the internal time structure
     */
    esp_err_t writeTimeToRTC(void);


    // Testing purpose functions
    esp_err_t printAllRegs(bool);

    virtual ~RV8263();
};

#endif /* __RV8263__ */
