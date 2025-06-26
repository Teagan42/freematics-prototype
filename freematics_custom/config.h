#ifndef CONFIG_H
#define CONFIG_H

// Data collection intervals (milliseconds)
#define OBD_INTERVAL 1000      // OBD data collection interval
#define GPS_INTERVAL 2000      // GPS data collection interval  
#define MEMS_INTERVAL 500      // MEMS data collection interval
#define BLE_INTERVAL 1000      // BLE data transmission interval

// Storage settings
#define STORAGE_SIZE 1024      // Storage buffer size
#define MAX_LOG_ENTRIES 100    // Maximum log entries to keep

// BLE settings
#define BLE_DEVICE_NAME "FreematicsCustom"
#define BLE_SERVICE_UUID "12345678-1234-1234-1234-123456789abc"

// Debug settings
#define DEBUG_ENABLED 1
#define SERIAL_BAUDRATE 115200

// Pin definitions (adjust based on your hardware)
#define PIN_LED 2
#define PIN_BUZZER 25

// OBD PIDs to monitor
#define PID_RPM 0x0C
#define PID_SPEED 0x0D
#define PID_COOLANT_TEMP 0x05
#define PID_ENGINE_LOAD 0x04
#define PID_THROTTLE_POS 0x11

#endif // CONFIG_H
