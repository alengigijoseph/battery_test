#include "DS3231.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "DS3231";

// Helper function to convert BCD to decimal
static uint8_t bcd_to_dec(uint8_t bcd) {
  return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// Helper function to convert decimal to BCD
static uint8_t dec_to_bcd(uint8_t dec) {
  return ((dec / 10) << 4) | (dec % 10);
}

esp_err_t DS3231_init(i2c_master_bus_handle_t bus_handle, uint8_t address,
                      ds3231_handle_t *out_handle) {
  if (out_handle == NULL) {
    ESP_LOGE(TAG, "Invalid handle pointer");
    return ESP_ERR_INVALID_ARG;
  }

  // Configure I2C device
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = address,
      .scl_speed_hz = 100000, // 100kHz for DS3231
  };

  esp_err_t ret =
      i2c_master_bus_add_device(bus_handle, &dev_cfg, &out_handle->i2c_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add DS3231 device: %s", esp_err_to_name(ret));
    return ret;
  }

  out_handle->address = address;

  // Clear oscillator stop flag if set
  uint8_t status;
  uint8_t reg_addr = DS3231_REG_STATUS;
  ret = i2c_master_transmit_receive(out_handle->i2c_handle, &reg_addr, 1,
                                    &status, 1, -1);
  if (ret == ESP_OK && (status & DS3231_STAT_OSF)) {
    ESP_LOGW(TAG, "Oscillator stop flag was set, clearing it");
    status &= ~DS3231_STAT_OSF;
    uint8_t write_buf[2] = {DS3231_REG_STATUS, status};
    i2c_master_transmit(out_handle->i2c_handle, write_buf, 2, -1);
  }

  ESP_LOGI(TAG, "DS3231 initialized at address 0x%02X", address);
  return ESP_OK;
}

esp_err_t DS3231_set_time(ds3231_handle_t *handle, const struct tm *time) {
  if (handle == NULL || time == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t data[8];
  data[0] = DS3231_REG_SECONDS;
  data[1] = dec_to_bcd(time->tm_sec);
  data[2] = dec_to_bcd(time->tm_min);
  data[3] = dec_to_bcd(time->tm_hour);     // 24-hour format
  data[4] = dec_to_bcd(time->tm_wday + 1); // DS3231 uses 1-7, tm uses 0-6
  data[5] = dec_to_bcd(time->tm_mday);
  data[6] = dec_to_bcd(time->tm_mon + 1);    // DS3231 uses 1-12, tm uses 0-11
  data[7] = dec_to_bcd(time->tm_year - 100); // DS3231 uses 0-99 for 2000-2099

  esp_err_t ret = i2c_master_transmit(handle->i2c_handle, data, 8, -1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set time: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "Time set: %04d-%02d-%02d %02d:%02d:%02d", time->tm_year + 1900,
           time->tm_mon + 1, time->tm_mday, time->tm_hour, time->tm_min,
           time->tm_sec);

  return ESP_OK;
}

esp_err_t DS3231_get_time(ds3231_handle_t *handle, struct tm *time) {
  if (handle == NULL || time == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t reg_addr = DS3231_REG_SECONDS;
  uint8_t data[7];

  esp_err_t ret = i2c_master_transmit_receive(handle->i2c_handle, &reg_addr, 1,
                                              data, 7, -1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read time: %s", esp_err_to_name(ret));
    return ret;
  }

  memset(time, 0, sizeof(struct tm));
  time->tm_sec = bcd_to_dec(data[0] & 0x7F);
  time->tm_min = bcd_to_dec(data[1] & 0x7F);
  time->tm_hour = bcd_to_dec(data[2] & 0x3F);     // 24-hour format
  time->tm_wday = bcd_to_dec(data[3] & 0x07) - 1; // Convert 1-7 to 0-6
  time->tm_mday = bcd_to_dec(data[4] & 0x3F);
  time->tm_mon = bcd_to_dec(data[5] & 0x1F) - 1; // Convert 1-12 to 0-11
  time->tm_year = bcd_to_dec(data[6]) + 100; // Convert 0-99 to years since 1900

  return ESP_OK;
}

esp_err_t DS3231_get_temperature(ds3231_handle_t *handle, float *temp) {
  if (handle == NULL || temp == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t reg_addr = DS3231_REG_TEMP_MSB;
  uint8_t data[2];

  esp_err_t ret = i2c_master_transmit_receive(handle->i2c_handle, &reg_addr, 1,
                                              data, 2, -1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read temperature: %s", esp_err_to_name(ret));
    return ret;
  }

  // Temperature is in MSB.LSB format, resolution 0.25°C
  int16_t temp_raw = (data[0] << 8) | data[1];
  *temp = temp_raw / 256.0f;

  return ESP_OK;
}

esp_err_t DS3231_check_oscillator_stop(ds3231_handle_t *handle, bool *stopped) {
  if (handle == NULL || stopped == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t reg_addr = DS3231_REG_STATUS;
  uint8_t status;

  esp_err_t ret = i2c_master_transmit_receive(handle->i2c_handle, &reg_addr, 1,
                                              &status, 1, -1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read status: %s", esp_err_to_name(ret));
    return ret;
  }

  *stopped = (status & DS3231_STAT_OSF) != 0;
  return ESP_OK;
}

esp_err_t DS3231_clear_oscillator_stop_flag(ds3231_handle_t *handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t reg_addr = DS3231_REG_STATUS;
  uint8_t status;

  esp_err_t ret = i2c_master_transmit_receive(handle->i2c_handle, &reg_addr, 1,
                                              &status, 1, -1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read status: %s", esp_err_to_name(ret));
    return ret;
  }

  status &= ~DS3231_STAT_OSF;
  uint8_t write_buf[2] = {DS3231_REG_STATUS, status};
  ret = i2c_master_transmit(handle->i2c_handle, write_buf, 2, -1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to clear OSF: %s", esp_err_to_name(ret));
    return ret;
  }

  return ESP_OK;
}

esp_err_t DS3231_enable_32khz(ds3231_handle_t *handle, bool enable) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t reg_addr = DS3231_REG_STATUS;
  uint8_t status;

  esp_err_t ret = i2c_master_transmit_receive(handle->i2c_handle, &reg_addr, 1,
                                              &status, 1, -1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read status: %s", esp_err_to_name(ret));
    return ret;
  }

  if (enable) {
    status |= DS3231_STAT_EN32KHZ;
  } else {
    status &= ~DS3231_STAT_EN32KHZ;
  }

  uint8_t write_buf[2] = {DS3231_REG_STATUS, status};
  ret = i2c_master_transmit(handle->i2c_handle, write_buf, 2, -1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set 32kHz output: %s", esp_err_to_name(ret));
    return ret;
  }

  return ESP_OK;
}
