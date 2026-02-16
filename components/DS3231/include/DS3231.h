#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

// DS3231 I2C Address
#define DS3231_I2C_ADDR 0x68

// DS3231 Register Addresses
#define DS3231_REG_SECONDS 0x00
#define DS3231_REG_MINUTES 0x01
#define DS3231_REG_HOURS 0x02
#define DS3231_REG_DAY 0x03
#define DS3231_REG_DATE 0x04
#define DS3231_REG_MONTH 0x05
#define DS3231_REG_YEAR 0x06
#define DS3231_REG_ALARM1_SEC 0x07
#define DS3231_REG_ALARM1_MIN 0x08
#define DS3231_REG_ALARM1_HOUR 0x09
#define DS3231_REG_ALARM1_DATE 0x0A
#define DS3231_REG_ALARM2_MIN 0x0B
#define DS3231_REG_ALARM2_HOUR 0x0C
#define DS3231_REG_ALARM2_DATE 0x0D
#define DS3231_REG_CONTROL 0x0E
#define DS3231_REG_STATUS 0x0F
#define DS3231_REG_TEMP_MSB 0x11
#define DS3231_REG_TEMP_LSB 0x12

// Control Register Bits
#define DS3231_CTRL_EOSC 0x80
#define DS3231_CTRL_BBSQW 0x40
#define DS3231_CTRL_CONV 0x20
#define DS3231_CTRL_RS2 0x10
#define DS3231_CTRL_RS1 0x08
#define DS3231_CTRL_INTCN 0x04
#define DS3231_CTRL_A2IE 0x02
#define DS3231_CTRL_A1IE 0x01

// Status Register Bits
#define DS3231_STAT_OSF 0x80
#define DS3231_STAT_EN32KHZ 0x08
#define DS3231_STAT_BSY 0x04
#define DS3231_STAT_A2F 0x02
#define DS3231_STAT_A1F 0x01

/**
 * @brief DS3231 device handle
 */
typedef struct {
  i2c_master_dev_handle_t i2c_handle;
  uint8_t address;
} ds3231_handle_t;

/**
 * @brief Initialize DS3231 RTC device
 *
 * @param bus_handle I2C master bus handle
 * @param address I2C address of the DS3231 device (default: DS3231_I2C_ADDR)
 * @param out_handle Pointer to store the created handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t DS3231_init(i2c_master_bus_handle_t bus_handle, uint8_t address,
                      ds3231_handle_t *out_handle);

/**
 * @brief Set the current time on the DS3231
 *
 * @param handle DS3231 device handle
 * @param time Pointer to struct tm with time to set
 * @return esp_err_t ESP_OK on success
 */
esp_err_t DS3231_set_time(ds3231_handle_t *handle, const struct tm *time);

/**
 * @brief Get the current time from the DS3231
 *
 * @param handle DS3231 device handle
 * @param time Pointer to struct tm to store the time
 * @return esp_err_t ESP_OK on success
 */
esp_err_t DS3231_get_time(ds3231_handle_t *handle, struct tm *time);

/**
 * @brief Get temperature from DS3231
 *
 * @param handle DS3231 device handle
 * @param temp Pointer to store temperature in degrees Celsius
 * @return esp_err_t ESP_OK on success
 */
esp_err_t DS3231_get_temperature(ds3231_handle_t *handle, float *temp);

/**
 * @brief Check if oscillator stop flag is set
 *
 * @param handle DS3231 device handle
 * @param stopped Pointer to store flag status (true if oscillator was stopped)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t DS3231_check_oscillator_stop(ds3231_handle_t *handle, bool *stopped);

/**
 * @brief Clear oscillator stop flag
 *
 * @param handle DS3231 device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t DS3231_clear_oscillator_stop_flag(ds3231_handle_t *handle);

/**
 * @brief Enable or disable the 32kHz output
 *
 * @param handle DS3231 device handle
 * @param enable true to enable, false to disable
 * @return esp_err_t ESP_OK on success
 */
esp_err_t DS3231_enable_32khz(ds3231_handle_t *handle, bool enable);

#ifdef __cplusplus
}
#endif
