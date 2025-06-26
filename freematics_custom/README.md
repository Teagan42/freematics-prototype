# Freematics Custom Sketch

This is a custom Arduino sketch based on the Freematics examples, designed to work with the Freematics BLE Android application for debugging.

## Hardware Requirements

- Freematics ONE+ (ESP32 based) or compatible ESP32 development board
- OBD-II connector and cable
- GPS module (if not integrated)
- MEMS sensor (accelerometer/gyroscope)

## Software Requirements

- Arduino IDE 1.8.x or newer
- ESP32 Arduino Core
- Freematics libraries (see libraries.txt)

## Setup Instructions

1. **Install Arduino IDE and ESP32 Core**
   - Download and install Arduino IDE
   - Add ESP32 board support via Board Manager

2. **Install Freematics Libraries**
   - Copy the Freematics libraries from the main repository to your Arduino libraries folder
   - Install additional libraries listed in libraries.txt

3. **Configure Hardware**
   - Connect your Freematics device to your computer via USB
   - Select the correct board and port in Arduino IDE

4. **Upload Sketch**
   - Open freematics_custom.ino in Arduino IDE
   - Verify and upload to your device

## Features

- **OBD-II Data Collection**: Reads engine RPM, speed, coolant temperature
- **GPS Tracking**: Collects location data and satellite information
- **MEMS Sensors**: Reads accelerometer data
- **Data Storage**: Logs data to internal storage
- **BLE Communication**: Sends data for debugging with Android app
- **Serial Debug**: Outputs debug information to serial monitor

## Configuration

Edit `config.h` to adjust:
- Data collection intervals
- Storage settings
- BLE configuration
- Pin definitions
- OBD PIDs to monitor

## Debugging with Android App

1. Install the Freematics BLE Android application
2. Enable Bluetooth on your Android device
3. Power on your Freematics device
4. Connect to the device via the Android app
5. Monitor real-time data and debug information

## Data Format

The sketch formats data for BLE transmission in the format expected by the Freematics Android app:
- Timestamp,RPM:value;SPD:value;GPS:lat,lng;

## Troubleshooting

- **OBD Connection Issues**: Check OBD-II cable and vehicle compatibility
- **GPS Not Working**: Ensure GPS antenna is connected and has clear sky view
- **BLE Connection Problems**: Check Bluetooth settings and device pairing
- **Compilation Errors**: Verify all required libraries are installed

## Customization

You can extend this sketch by:
- Adding more OBD PIDs to monitor
- Implementing additional sensors
- Modifying data storage format
- Adding WiFi connectivity
- Implementing custom BLE protocols
