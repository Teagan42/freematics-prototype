# Freematics Custom Sketch

This is a custom Arduino sketch for ESP32-based vehicle telemetry, designed to work with both the included web dashboard and Freematics BLE Android application for debugging.

## Hardware Requirements

- Freematics ONE+ (ESP32 based) or compatible ESP32 development board
- OBD-II connector and cable (optional - simulation mode available)
- Built-in GPS and sensors (or external modules)

## Software Requirements

- Arduino CLI (recommended) or Arduino IDE 1.8.x or newer
- ESP32 Arduino Core 3.2.0+
- Standard ESP32 BLE libraries (built-in)

## Setup Instructions

### Quick Start (Recommended)

1. **Install Arduino CLI**
   ```bash
   # macOS
   brew install arduino-cli
   
   # Or download from: https://arduino.github.io/arduino-cli/
   ```

2. **Deploy with Script**
   ```bash
   chmod +x deploy.sh
   ./deploy.sh
   ```

The deployment script will automatically:
- Install ESP32 board package if needed
- Verify your Freematics ONE+ connection
- Check all dependencies
- Compile and upload the sketch
- Start serial monitoring

3. **Deploy Web Dashboard (Optional)**
   ```bash
   chmod +x deploy-web.sh
   ./deploy-web.sh your-server.com
   ```

### Manual Setup

1. **Install Arduino CLI and ESP32 Core**
   ```bash
   arduino-cli core update-index
   arduino-cli core install esp32:esp32
   ```

2. **Configure Hardware**
   - Connect your Freematics device to your computer via USB
   - Identify the port: `arduino-cli board list`

3. **Compile and Upload**
   ```bash
   arduino-cli compile --fqbn esp32:esp32:esp32 freematics_custom.ino
   arduino-cli upload -p /dev/cu.usbserial-* --fqbn esp32:esp32:esp32 freematics_custom.ino
   ```

## Features

- **OBD-II Data Collection**: Reads engine RPM, speed, coolant temperature, engine pressure
- **Hardware Sensors**: Battery voltage, ambient temperature via built-in sensors
- **GPS Tracking**: Collects location data and satellite information with averaging
- **Simulation Mode**: Generate test data when no vehicle is connected
- **Data Storage**: Logs data to internal circular buffer (100 entries)
- **BLE Communication**: Advanced protocol with command handling and API versioning
- **Web Dashboard**: React-based real-time monitoring interface
- **Serial Debug**: Comprehensive debug output with message categorization

## Configuration

Edit `config.h` to adjust:
- Data collection intervals
- Storage settings
- BLE configuration
- Pin definitions
- OBD PIDs to monitor

## Monitoring Options

### Web Dashboard (Recommended)
1. Open `dashboard.html` in Chrome/Edge browser (requires HTTPS for Web Bluetooth)
2. Click "Connect to Freematics" button
3. Select your device from Bluetooth pairing dialog
4. Monitor real-time data with charts and controls
5. Use simulation controls to test without vehicle

### Android App (Alternative)
1. Install the Freematics BLE Android application
2. Enable Bluetooth on your Android device
3. Power on your Freematics device
4. Connect to the device via the Android app
5. Monitor real-time data and debug information

## BLE Protocol

### Data Format
Messages include message counter and follow this format:
```
messageId:timestamp,RPM:value;SPD:value;GPS:lat,lng;SAT:count;MODE:status;
```

### Command Protocol
Send commands to device:
```
CMD:SIM_ON     - Enable simulation mode
CMD:SIM_OFF    - Disable simulation mode  
CMD:STATUS     - Request system status
CMD:PING       - Test connection
```

### API Versioning
- Current API version: 1
- Dashboard validates compatibility automatically
- Firmware reports version on connection

## Troubleshooting

- **OBD Connection Issues**: Use simulation mode for testing without vehicle
- **GPS Not Working**: Simulated GPS data available for development
- **BLE Connection Problems**: Use Chrome/Edge with HTTPS for web dashboard
- **Compilation Errors**: Ensure ESP32 core 3.2.0+ is installed
- **Web Bluetooth Issues**: Requires HTTPS (use localhost for testing)
- **No Data**: Enable simulation mode via dashboard or BLE commands

### ⚠️ Legacy Hardware Issues
This project targets **older ESP32 boards** with **custom implementations**:
- **Function not found errors**: Newer ESP32 functions may not exist on older boards
- **Sensor reading failures**: Uses custom ADC/sensor implementations, not standard libraries
- **Temperature reading issues**: Check `readAmbientTemperature()` custom implementation
- **BLE connection problems**: Uses basic ESP32 BLE, not newer advanced features
- **When modifying code**: Follow existing custom patterns rather than using modern ESP32 APIs

## Customization

You can extend this sketch by:
- Adding more OBD PIDs in `config.h` and implementing in `SimpleOBD::readPID()`
- Implementing additional hardware sensors in `readHardwareSensor()`
- Modifying data collection intervals in `config.h`
- Extending BLE command protocol in `MyCharacteristicCallbacks::onWrite()`
- Adding new dashboard features in `dashboard.html`
- Implementing data export/logging features

## File Structure

- `freematics_custom.ino` - Main sketch with BLE and sensor logic
- `config.h` - Configuration constants and pin definitions
- `telestore.h` - Data logging and storage implementation
- `dashboard.html` - Web-based monitoring interface
- `deploy.sh` - Automated deployment script
- `deploy-web.sh` - Web dashboard deployment script
