#ifndef INIT_H_
#define INIT_H_

#include "DS3231.h"


extern ds3231_handle_t DS3231_handle;

typedef struct {
    gpio_num_t sda;
    gpio_num_t scl;
} i2c_pins_t;

typedef struct
{

  struct tm current_time;  // Current time from DS3231
  bool time_valid;         // True if time was successfully read
  uint32_t last_update_ms; // Timestamp of last RTC read

} rtc;



void Init_RTC(void);

#endif /* INIT_H_ */