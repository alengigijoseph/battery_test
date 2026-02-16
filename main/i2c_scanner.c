/**
 * @file i2c_scanner.c
 * @brief Simple I2C device scanner
 */

#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "I2C_SCANNER";

void i2c_scan(i2c_master_bus_handle_t bus_handle) {
    ESP_LOGI(TAG, "Starting I2C scan...");
    
    int devices_found = 0;
    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = 100000,
        };
        
        i2c_master_dev_handle_t dev_handle;
        esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
        if (ret != ESP_OK) continue;
        
        uint8_t data;
        ret = i2c_master_receive(dev_handle, &data, 1, -1);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
            devices_found++;
        }
        
        i2c_master_bus_rm_device(dev_handle);
    }
    
    if (devices_found == 0) {
        ESP_LOGW(TAG, "No I2C devices found!");
    } else {
        ESP_LOGI(TAG, "I2C scan complete. Found %d device(s)", devices_found);
    }
}
