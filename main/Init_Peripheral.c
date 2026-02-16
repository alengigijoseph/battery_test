#include "Init_Peripheral.h"
#include "DS3231.h"
#include "esp_log.h"

static const char *TAG = "Init_Peripheral";


static const i2c_pins_t pmbus_pins = {
    .sda = GPIO_NUM_2,
    .scl = GPIO_NUM_1,
};

ds3231_handle_t DS3231_handle;
i2c_master_bus_handle_t PMBus_handle;


i2c_master_bus_config_t PMbus_cfg = {
    .i2c_port = 0,
    .sda_io_num = pmbus_pins.sda,
    .scl_io_num = pmbus_pins.scl,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,
};


void Init_RTC(void) {

  esp_err_t ret = i2c_new_master_bus(&PMbus_cfg, &PMBus_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize PMBus: %s", esp_err_to_name(ret));
  } else {
    ESP_LOGI(TAG, "PMBus initialized successfully");
  }

  ESP_LOGI(TAG, "Initializing DS3231 RTC...");

  ret = DS3231_init(PMBus_handle, DS3231_I2C_ADDR, &DS3231_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize DS3231 RTC: %s", esp_err_to_name(ret));
    return;
  }

  // Check if oscillator was stopped
  bool osc_stopped = false;
  ret = DS3231_check_oscillator_stop(&DS3231_handle, &osc_stopped);
  if (ret == ESP_OK && osc_stopped) {
    ESP_LOGW(TAG, "RTC oscillator was stopped - time may be invalid");
  }

  // Read and log current time
  struct tm current_time;
  ret = DS3231_get_time(&DS3231_handle, &current_time);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Current RTC time: %04d-%02d-%02d %02d:%02d:%02d",
             current_time.tm_year + 1900, current_time.tm_mon + 1,
             current_time.tm_mday, current_time.tm_hour, current_time.tm_min,
             current_time.tm_sec);
  }

  // Read and log temperature
  float temperature;
  ret = DS3231_get_temperature(&DS3231_handle, &temperature);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "RTC temperature: %.2f°C", temperature);
  }

  ESP_LOGI(TAG, "DS3231 RTC initialized successfully");
}