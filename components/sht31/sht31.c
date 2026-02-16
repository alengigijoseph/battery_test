/**
 * @file sht31.c
 * @brief SHT31 Temperature and Humidity Sensor Driver Implementation
 */

#include "sht31.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "SHT31";

// SHT31 Commands (Single Shot Mode, Clock Stretching Disabled)
#define SHT31_CMD_MEASURE_HIGH_REP    0x2400
#define SHT31_CMD_MEASURE_MED_REP     0x240B
#define SHT31_CMD_MEASURE_LOW_REP     0x2416
#define SHT31_CMD_SOFT_RESET          0x30A2
#define SHT31_CMD_HEATER_ENABLE       0x306D
#define SHT31_CMD_HEATER_DISABLE      0x3066
#define SHT31_CMD_READ_STATUS         0xF32D
#define SHT31_CMD_CLEAR_STATUS        0x3041

#define SHT31_I2C_TIMEOUT_MS          1000
#define SHT31_RESET_DELAY_MS          2
#define SHT31_MEAS_DELAY_HIGH_MS      16
#define SHT31_MEAS_DELAY_MED_MS       7
#define SHT31_MEAS_DELAY_LOW_MS       5

/**
 * @brief Calculate CRC-8 checksum for SHT31 data
 */
static uint8_t sht31_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief Write command to SHT31
 */
static esp_err_t sht31_write_command(sht31_handle_t *handle, uint16_t cmd) {
    uint8_t cmd_buf[2];
    cmd_buf[0] = (cmd >> 8) & 0xFF;  // MSB
    cmd_buf[1] = cmd & 0xFF;         // LSB
    
    esp_err_t ret = i2c_master_transmit(handle->i2c_dev, cmd_buf, 2, -1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write command 0x%04X: %s", cmd, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Read data from SHT31
 */
static esp_err_t sht31_read_data(sht31_handle_t *handle, uint8_t *data, size_t len) {
    ESP_LOGD(TAG, "Attempting to read %d bytes from device 0x%02X", len, handle->i2c_addr);
    
    // Retry loop for reading with timeout
    int max_retries = 20;
    for (int i = 0; i < max_retries; i++) {
        esp_err_t ret = i2c_master_receive(handle->i2c_dev, data, len, -1);
        
        if (ret == ESP_OK) {
            ESP_LOGD(TAG, "Read data: %02X %02X %02X %02X %02X %02X", 
                     data[0], data[1], data[2], data[3], data[4], data[5]);
            return ESP_OK;
        }
        
        // If NACK or transaction failed, retry after short delay
        if (i < max_retries - 1) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    
    ESP_LOGE(TAG, "Failed to read data after retries (error code: 0x%x)", ESP_ERR_INVALID_STATE);
    return ESP_ERR_INVALID_STATE;
}

esp_err_t sht31_init(const sht31_config_t *config, sht31_handle_t **handle) {
    if (config == NULL || handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Allocate handle
    *handle = (sht31_handle_t *)malloc(sizeof(sht31_handle_t));
    if (*handle == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for handle");
        return ESP_ERR_NO_MEM;
    }
    
    // Configure I2C device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->i2c_addr,
        .scl_speed_hz = 100000,  // 100kHz
    };
    
    esp_err_t ret = i2c_master_bus_add_device(config->i2c_bus, &dev_cfg, &(*handle)->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        free(*handle);
        *handle = NULL;
        return ret;
    }
    
    // Initialize handle
    (*handle)->i2c_addr = config->i2c_addr;
    (*handle)->repeatability = config->repeatability;
    
    // Perform soft reset
    ret = sht31_soft_reset(*handle);
    if (ret != ESP_OK) {
        i2c_master_bus_rm_device((*handle)->i2c_dev);
        free(*handle);
        *handle = NULL;
        return ret;
    }
    
    ESP_LOGI(TAG, "SHT31 initialized successfully at address 0x%02X", config->i2c_addr);
    return ESP_OK;
}

esp_err_t sht31_deinit(sht31_handle_t *handle) {
    if (handle != NULL) {
        i2c_master_bus_rm_device(handle->i2c_dev);
        free(handle);
    }
    return ESP_OK;
}

esp_err_t sht31_read_temperature_humidity(sht31_handle_t *handle, 
                                          float *temperature, 
                                          float *humidity) {
    if (handle == NULL || temperature == NULL || humidity == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Select measurement command based on repeatability
    uint16_t cmd;
    uint32_t delay_ms;
    
    switch (handle->repeatability) {
        case SHT31_REPEATABILITY_HIGH:
            cmd = SHT31_CMD_MEASURE_HIGH_REP;
            delay_ms = SHT31_MEAS_DELAY_HIGH_MS;
            break;
        case SHT31_REPEATABILITY_MEDIUM:
            cmd = SHT31_CMD_MEASURE_MED_REP;
            delay_ms = SHT31_MEAS_DELAY_MED_MS;
            break;
        case SHT31_REPEATABILITY_LOW:
            cmd = SHT31_CMD_MEASURE_LOW_REP;
            delay_ms = SHT31_MEAS_DELAY_LOW_MS;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    // Send measurement command
    esp_err_t ret = sht31_write_command(handle, cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for measurement to complete - add extra margin
    vTaskDelay(pdMS_TO_TICKS(delay_ms + 5));
    
    // Read 6 bytes: temp MSB, temp LSB, temp CRC, hum MSB, hum LSB, hum CRC
    uint8_t data[6];
    ret = sht31_read_data(handle, data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data");
        return ret;
    }
    
    // Verify temperature CRC
    uint8_t temp_crc = sht31_crc8(data, 2);
    if (temp_crc != data[2]) {
        ESP_LOGE(TAG, "Temperature CRC mismatch: expected 0x%02X, got 0x%02X", 
                 temp_crc, data[2]);
        return ESP_ERR_INVALID_CRC;
    }
    
    // Verify humidity CRC
    uint8_t hum_crc = sht31_crc8(data + 3, 2);
    if (hum_crc != data[5]) {
        ESP_LOGE(TAG, "Humidity CRC mismatch: expected 0x%02X, got 0x%02X", 
                 hum_crc, data[5]);
        return ESP_ERR_INVALID_CRC;
    }
    
    // Convert raw values to physical units
    uint16_t temp_raw = (data[0] << 8) | data[1];
    uint16_t hum_raw = (data[3] << 8) | data[4];
    
    // Temperature conversion: T = -45 + 175 * (S_T / 2^16)
    *temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    
    // Humidity conversion: RH = 100 * (S_RH / 2^16)
    *humidity = 100.0f * ((float)hum_raw / 65535.0f);
    
    // Clamp humidity to valid range
    if (*humidity > 100.0f) *humidity = 100.0f;
    if (*humidity < 0.0f) *humidity = 0.0f;
    
    return ESP_OK;
}

esp_err_t sht31_soft_reset(sht31_handle_t *handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = sht31_write_command(handle, SHT31_CMD_SOFT_RESET);
    if (ret == ESP_OK) {
        // Wait for reset to complete
        vTaskDelay(pdMS_TO_TICKS(SHT31_RESET_DELAY_MS));
        ESP_LOGI(TAG, "Soft reset completed");
    }
    
    return ret;
}

esp_err_t sht31_heater_control(sht31_handle_t *handle, bool enable) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint16_t cmd = enable ? SHT31_CMD_HEATER_ENABLE : SHT31_CMD_HEATER_DISABLE;
    esp_err_t ret = sht31_write_command(handle, cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Heater %s", enable ? "enabled" : "disabled");
    }
    
    return ret;
}

esp_err_t sht31_read_status(sht31_handle_t *handle, uint16_t *status) {
    if (handle == NULL || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Send read status command
    esp_err_t ret = sht31_write_command(handle, SHT31_CMD_READ_STATUS);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Read status (2 bytes + 1 CRC)
    uint8_t data[3];
    ret = sht31_read_data(handle, data, 3);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Verify CRC
    uint8_t crc = sht31_crc8(data, 2);
    if (crc != data[2]) {
        ESP_LOGE(TAG, "Status CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    
    *status = (data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t sht31_clear_status(sht31_handle_t *handle) {
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return sht31_write_command(handle, SHT31_CMD_CLEAR_STATUS);
}