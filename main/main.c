#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "sht31.h"
#include "i2c_scanner.h"
#include "Init_Peripheral.h"

static const char *TAG = "MAIN";

// I2C Configuration
#define I2C_MASTER_SDA_IO       2        // Change to your SDA pin
#define I2C_MASTER_SCL_IO       1        // Change to your SCL pin
#define I2C_MASTER_FREQ_HZ      100000   // 100kHz

static i2c_master_bus_handle_t bus_handle = NULL;

/**
 * @brief Initialize I2C master
 */
static esp_err_t i2c_master_init(void) {
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    esp_err_t err = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus initialization failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

/**
 * @brief Main application task
 */
void app_main(void) {

    Init_RTC();
    // Initialize I2C
    //ESP_ERROR_CHECK(i2c_master_init());
    
    // Scan for I2C devices
    //ESP_LOGI(TAG, "Scanning I2C bus...");
    //i2c_scan(bus_handle);
    
    // Configure SHT31
    // sht31_config_t sht31_cfg = {
    //     .i2c_bus = bus_handle,
    //     .i2c_addr = SHT31_I2C_ADDR_LOW,  // Use default address (0x44)
    //     .repeatability = SHT31_REPEATABILITY_HIGH,
    // };
    
    // Initialize SHT31
    //sht31_handle_t *sht31 = NULL;
    // esp_err_t ret = sht31_init(&sht31_cfg, &sht31);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to initialize SHT31: %s", esp_err_to_name(ret));
    //     return;
    // }
    
    // Read status register
    //uint16_t status;
    //ret = sht31_read_status(sht31, &status);
    // if (ret == ESP_OK) {
    //     ESP_LOGI(TAG, "Status register: 0x%04X", status);
    // }
    
    // Main loop - read sensor data every 2 seconds
    //float temperature, humidity;
    while (1) {
        //ret = sht31_read_temperature_humidity(sht31, &temperature, &humidity);
        
        // if (ret == ESP_OK) {
        //     ESP_LOGI(TAG, "Temperature: %.2f°C, Humidity: %.2f%%RH", 
        //              temperature, humidity);
        // } else {
        //     ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
        // }

        struct tm rtc_time;
        bool rtc_success = false;
        if (DS3231_get_time(&DS3231_handle, &rtc_time) == ESP_OK) {
            rtc_success = true;
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    // Cleanup (never reached in this example)
    //sht31_deinit(sht31);
}