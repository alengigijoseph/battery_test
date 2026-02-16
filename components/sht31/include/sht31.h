/**
 * @file sht31.h
 * @brief SHT31 Temperature and Humidity Sensor Driver for ESP-IDF
 * 
 * This driver supports the SHT31-DIS sensor via I2C communication
 */

#ifndef SHT31_H
#define SHT31_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SHT31 I2C addresses
 */
#define SHT31_I2C_ADDR_LOW   0x44  // ADDR pin connected to VSS (default)
#define SHT31_I2C_ADDR_HIGH  0x45  // ADDR pin connected to VDD

/**
 * @brief SHT31 measurement repeatability modes
 */
typedef enum {
    SHT31_REPEATABILITY_HIGH = 0,    // 15ms measurement time
    SHT31_REPEATABILITY_MEDIUM,      // 6ms measurement time
    SHT31_REPEATABILITY_LOW          // 4ms measurement time
} sht31_repeatability_t;

/**
 * @brief SHT31 configuration structure
 */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;        // I2C master bus handle
    uint8_t i2c_addr;                       // I2C device address
    sht31_repeatability_t repeatability;    // Measurement repeatability
} sht31_config_t;

/**
 * @brief SHT31 device handle
 */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;       // I2C device handle
    uint8_t i2c_addr;
    sht31_repeatability_t repeatability;
} sht31_handle_t;

/**
 * @brief Initialize SHT31 sensor
 * 
 * @param config Pointer to configuration structure
 * @param handle Pointer to device handle (will be allocated)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sht31_init(const sht31_config_t *config, sht31_handle_t **handle);

/**
 * @brief Deinitialize SHT31 sensor
 * 
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t sht31_deinit(sht31_handle_t *handle);

/**
 * @brief Read temperature and humidity from SHT31
 * 
 * @param handle Device handle
 * @param temperature Pointer to store temperature in Celsius
 * @param humidity Pointer to store relative humidity in %RH
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sht31_read_temperature_humidity(sht31_handle_t *handle, 
                                          float *temperature, 
                                          float *humidity);

/**
 * @brief Perform soft reset on SHT31
 * 
 * @param handle Device handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sht31_soft_reset(sht31_handle_t *handle);

/**
 * @brief Enable/disable internal heater
 * 
 * @param handle Device handle
 * @param enable true to enable heater, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sht31_heater_control(sht31_handle_t *handle, bool enable);

/**
 * @brief Read status register
 * 
 * @param handle Device handle
 * @param status Pointer to store status value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sht31_read_status(sht31_handle_t *handle, uint16_t *status);

/**
 * @brief Clear status register
 * 
 * @param handle Device handle
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sht31_clear_status(sht31_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // SHT31_H