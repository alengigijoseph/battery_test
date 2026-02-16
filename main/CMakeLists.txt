idf_component_register(
    SRCS "Init_Peripheral.c" "main.c" "i2c_scanner.c"
    INCLUDE_DIRS "."
    REQUIRES sht31 DS3231
)