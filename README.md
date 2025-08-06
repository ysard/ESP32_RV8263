# RV-8263-C7 driver for ESP32

An ESP-IDF/PlatformIO library for the real time clock RV-8263-C7.

This library is based on the work of [Istvan Zilizi](https://github.com/zilizii/RTC_MS/).

The RV-8263-C7 is a CMOS real-time clock/calendar module optimized for low power consumption.
This ultra small RTC module has been specially designed for miniature and cost sensitive applications.

Read the Application Manual of Micro Crystal Switzerland!

## What's new?

- The library was extracted from its original project.
- Functions and codes no longer required are removed.
- Still thread safe but using the ([i2c manager project](https://github.com/ropg/i2c_manager))to centralize the i2c configuration & usage.
- Alarms, counters, pulse generation are added.

## Caution

This project is NOT extensively tested, there can be many bugs although normally easy to fix.
Issues, bugfixes & pull requests are welcome!

## Configure the project

Tested with ESP-IDF framework version of PlatformIO (@ 3.50401.0 (5.4.1)).

```bash
$ idf.py menuconfig
# or
$ pio run -t menuconfig
```

Global configuration variables are reachable here: `Component config / RTC / RV-8263 RTC Driver`

SCL, SDA GPIO pins, I2C port and frequency are configured via the `I2C Port Settings` of the I2C manager component.

## Quick example

```C++
#include <inttypes.h>
#include <esp_log.h>
#include "esp_err.h"

#include "RV8263.h"

static const char *TAG = "RTC";

RV8263 *rtc = NULL;

void init_rtc() {
    rtc = new RV8263();
}

/**
* @brief Set the given epoch to the RTC memory
* @warning Should be used only 1 time (during the configuration phase)
* @note A reset is initiated with the software reset command.
*  It must be done when power is stable (if initial startup VDD is not 0).
*  Two communication tries are made; a reset is made before each try.
*/
esp_err_t configure_rtc_first_boot(unsigned long epoch) {
    esp_err_t ret;
    uint8_t tries = 2;
    bool oscillator_not_working = true;

    do {
        // First boot or reset: reset the RTC
        rtc->resetRTC();
        ret = rtc->isOscillatorRunning(&oscillator_not_working);
        vTaskDelay(pdMS_TO_TICKS(400));
        tries--;
    } while (tries && oscillator_not_working);

    if (oscillator_not_working)
        return ESP_ERR_INVALID_RESPONSE;

    ret = rtc->readAllRegsFromRTC();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read Failed");
        return ret;
    }

    ret = rtc->writeTimeFromEpochToRTC(epoch);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write epoch to RTC");
    }
    return ret;
}


extern "C" void app_main() {
    // Get epoch from an NTP server or from a browser
    unsigned long epoch = 1751744629;
    // time_t epoch = my_ntp_client->getEpochTime();

    init_rtc();
    // Do not forget to test RTC before use, if not working: try a reset.
    configure_rtc_first_boot(epoch);
    // Set a timezone which was not selected at compilation time
    // Ex: Europe/London
    rtc->setTimezone("GMT0BST,M3.5.0/1,M10.5.0");
    // Get the UTC timestamp (should be equal)
    rtc->getEpoch(&epoch);
    ESP_LOGI(TAG, "UTC epoch: %lu", epoch);

    // Get date in YYYYMMDD_HHMMSS format
    char *datetime_str = rtc->getFormattedDateTime();
    ESP_LOGI(TAG, "Date str from epoch: %s", datetime_str);

    // Get date in YYYYMMDD format
    char date[9]; // Do not forget '\0' terminator
    rtc->getFormattedDateTime("%Y%m%d", date, sizeof(date));
    ESP_LOGI(TAG, "Date str from epoch: %s", date);

    // Wake up every 1st day of month at midday
    rtc->resetAlarms();
    rtc->configureAlarms(REG_ADDR_DATE_ALARM, true, 1);
    rtc->configureAlarms(REG_ADDR_HOURS_ALARM, true, 12);
    rtc->setAlarmInterrupt(true);
}
```


# License

This program is licensed under the MIT License.

Copyright (c) 2022 Istvan Zilizi
Copyright (c) 2025 Ysard
